"""
基于双目深度估计的点云生成器
"""

import numpy as np
import open3d as o3d
import torch
from typing import Tuple, Optional, Dict, Any


class StereoPointCloudGenerator:
    """基于双目深度估计的点云生成器"""
    
    def __init__(self):
        """初始化"""
        pass
    
    def generate(self,
                 depth: np.ndarray,
                 rgb: np.ndarray,
                 camera_params: Dict[str, float],
                 pose: Any,
                 max_depth: float = 100.0) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        从深度图生成点云
        
        Args:
            depth: 深度图 (H, W), float32, 单位：米
            rgb: RGB图像 (H, W, 3), uint8
            camera_params: 相机内参 {'fx', 'fy', 'cx', 'cy'}
            pose: 位姿（pypose.SE3对象或4x4矩阵）
            max_depth: 最大深度阈值（米）
            
        Returns:
            points: 点云坐标 (N, 3), float32
            colors: 点云颜色 (N, 3), float32, range [0, 1]
        """
        h, w = depth.shape
        fx, fy = camera_params['fx'], camera_params['fy']
        cx, cy = camera_params['cx'], camera_params['cy']
        
        # 创建网格坐标
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # 有效深度掩码
        valid_mask = (depth > 0.1) & (depth < max_depth) & np.isfinite(depth)
        
        # 计算3D坐标（相机坐标系）
        Z = depth[valid_mask] * 5.0  # 深度缩放因子
        X = (u[valid_mask] - cx) * Z / fx
        Y = (v[valid_mask] - cy) * Z / fy
        
        points_cam = np.stack([X, Y, Z], axis=-1)
        
        # 转换到世界坐标系
        if hasattr(pose, 'translation') and hasattr(pose, 'rotation'):
            # 使用pypose进行变换
            points_cam_tensor = torch.tensor(points_cam, dtype=torch.float32)
            device = pose.device
            points_cam_tensor = points_cam_tensor.to(device)
            points_world_tensor = pose * points_cam_tensor
            points_world = points_world_tensor.detach().cpu().numpy()
        else:
            # 使用numpy矩阵变换
            points_world = self._transform_points(points_cam, pose)
        
        # 提取颜色
        colors = None
        if rgb is not None:
            if rgb.shape[:2] != (h, w):
                import cv2
                rgb = cv2.resize(rgb, (w, h))
            colors = rgb[valid_mask] / 255.0
        
        return points_world, colors
    
    def filter(self,
               points: np.ndarray,
               colors: Optional[np.ndarray],
               filter_params: Dict[str, Any]) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        点云过滤
        
        Args:
            points: 输入点云 (N, 3) - 世界坐标系中的点
            colors: 输入颜色 (N, 3)
            filter_params: 过滤参数
                {
                    'height_filter_mode': 'relative' or 'absolute',
                    'height_ratio_range': [ratio_min, ratio_max],  # 相对模式
                    'height_range': [min, max]  # 绝对模式
                }
            
        Returns:
            filtered_points: 过滤后的点云
            filtered_colors: 过滤后的颜色
        """
        if len(points) == 0:
            return points, colors
        
        mask = np.ones(len(points), dtype=bool)
        
        # 高度范围过滤
        height_filter_mode = filter_params.get('height_filter_mode', 'relative')
        
        if height_filter_mode == 'relative':
            # 相对模式：基于点云自身高度范围的百分比
            if 'height_ratio_range' in filter_params:
                ratio_min, ratio_max = filter_params['height_ratio_range']
                y_min = points[:, 1].min()
                y_max = points[:, 1].max()
                y_range = y_max - y_min
                
                if y_range >= 0.01:  # 避免除零
                    height_min_abs = y_min + y_range * ratio_min
                    height_max_abs = y_min + y_range * ratio_max
                    height_mask = (points[:, 1] >= height_min_abs) & (points[:, 1] <= height_max_abs)
                    mask &= height_mask
        else:
            # 绝对模式：使用固定高度范围
            if 'height_range' in filter_params:
                min_h, max_h = filter_params['height_range']
                height_mask = (points[:, 1] >= min_h) & (points[:, 1] <= max_h)
                mask &= height_mask
        
        # 应用过滤
        filtered_points = points[mask]
        filtered_colors = colors[mask] if colors is not None else None
        
        return filtered_points, filtered_colors
    
    def downsample(self,
                   points: np.ndarray,
                   colors: Optional[np.ndarray],
                   voxel_size: float) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        体素下采样
        
        Args:
            points: 输入点云 (N, 3)
            colors: 输入颜色 (N, 3)
            voxel_size: 体素大小（米）
            
        Returns:
            downsampled_points: 下采样后的点云
            downsampled_colors: 下采样后的颜色
        """
        if len(points) == 0:
            return points, colors
        
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 体素下采样
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        
        # 提取结果
        points_down = np.asarray(pcd_down.points)
        colors_down = np.asarray(pcd_down.colors) if pcd_down.has_colors() else None
        
        return points_down, colors_down
    
    def _transform_points(self, points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """
        坐标变换（使用齐次坐标）
        
        Args:
            points: 点云 (N, 3)
            pose: 变换矩阵 (4, 4)
            
        Returns:
            transformed_points: 变换后的点云 (N, 3)
        """
        ones = np.ones((len(points), 1))
        points_homo = np.hstack([points, ones])
        points_world_homo = points_homo @ pose.T
        return points_world_homo[:, :3]
    
    def to_open3d_pointcloud(self, 
                            points: np.ndarray, 
                            colors: Optional[np.ndarray] = None) -> o3d.geometry.PointCloud:
        """
        转换为Open3D点云对象
        
        Args:
            points: 点云坐标 (N, 3)
            colors: 点云颜色 (N, 3)
            
        Returns:
            pcd: Open3D点云对象
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd