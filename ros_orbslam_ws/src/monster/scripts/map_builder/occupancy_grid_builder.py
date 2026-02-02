"""
基于占用栅格的地图构建器实现
"""

import numpy as np
import open3d as o3d
from typing import Dict, Tuple, Optional, Any, List


class OccupancyGridBuilder:
    """基于占用栅格的地图构建器"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化
        
        Args:
            config: 配置字典
                {
                    'sliding_window': {
                        'enabled': bool,
                        'size': int
                    },
                    'max_points': int  # 最大点数限制（可选）
                }
        """
        self.config = config
        
        # 滑动窗口配置
        sliding_window_config = config.get('sliding_window', {})
        self.enable_sliding_window = sliding_window_config.get('enabled', True)
        self.sliding_window_size = sliding_window_config.get('size', 3)
        
        # 点云存储
        if self.enable_sliding_window:
            # 滑动窗口模式：存储每帧点云
            self.point_cloud_frames: List[o3d.geometry.PointCloud] = []
        else:
            # 累积模式：存储单个累积点云
            self.accumulated_cloud = o3d.geometry.PointCloud()
        
        # 最大点数限制
        self.max_points = config.get('max_points', None)
        
        print(f"✓ OccupancyGridBuilder 初始化完成")
        print(f"  - 滑动窗口: {'启用' if self.enable_sliding_window else '禁用'}")
        if self.enable_sliding_window:
            print(f"  - 窗口大小: {self.sliding_window_size} 帧")
        if self.max_points:
            print(f"  - 最大点数: {self.max_points}")
    
    def update(self, 
               points: np.ndarray, 
               colors: Optional[np.ndarray] = None) -> None:
        """
        更新地图
        
        Args:
            points: 新的点云 (N, 3)
            colors: 点云颜色 (N, 3)
        """
        if len(points) == 0:
            return
        
        # 创建点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if self.enable_sliding_window:
            # 滑动窗口模式
            self.point_cloud_frames.append(pcd)
            
            # 如果超过窗口大小，移除最旧的帧
            if len(self.point_cloud_frames) > self.sliding_window_size:
                self.point_cloud_frames.pop(0)
        else:
            # 累积模式
            self.accumulated_cloud += pcd
            
            # 检查点数限制
            if self.max_points and len(self.accumulated_cloud.points) > self.max_points:
                # 下采样到最大点数
                ratio = self.max_points / len(self.accumulated_cloud.points)
                self.accumulated_cloud = self.accumulated_cloud.random_down_sample(ratio)
    
    def get_occupancy_grid(self, 
                          resolution: float,
                          height_range: Tuple[float, float],
                          occupied_thresh: int = 5,
                          use_ratio: bool = True) -> Optional[Dict[str, Any]]:
        """
        获取2D占用栅格地图
        
        Args:
            resolution: 网格分辨率（米/格）
            height_range: 高度范围
                - 如果 use_ratio=True: (ratio_min, ratio_max) 百分比
                - 如果 use_ratio=False: (height_min, height_max) 绝对高度（米）
            occupied_thresh: 占用阈值（点数）
            use_ratio: 是否使用百分比模式
            
        Returns:
            grid_map: 地图字典或None
        """
        # 获取当前点云
        points, _ = self.get_point_cloud()
        
        if len(points) == 0:
            return None
        
        # 高度过滤
        if use_ratio:
            # 百分比模式
            ratio_min, ratio_max = height_range
            y_min = points[:, 1].min()
            y_max = points[:, 1].max()
            y_range = y_max - y_min
            
            if y_range < 0.01:
                # 高度范围太小，使用全部点
                mask = np.ones(len(points), dtype=bool)
            else:
                height_min_abs = y_min + y_range * ratio_min
                height_max_abs = y_min + y_range * ratio_max
                mask = (points[:, 1] >= height_min_abs) & (points[:, 1] <= height_max_abs)
        else:
            # 绝对高度模式
            height_min, height_max = height_range
            mask = (points[:, 1] >= height_min) & (points[:, 1] <= height_max)
        
        # 提取XZ平面的点（ROS坐标系：X前，Y上，Z右）
        points_xy = np.column_stack((points[mask, 0], points[mask, 2]))
        
        if len(points_xy) < 50:
            return None
        
        # 计算地图边界（加1m margin）
        x_min, y_min = points_xy.min(axis=0) - 1.0
        x_max, y_max = points_xy.max(axis=0) + 1.0
        
        width = int(np.ceil((x_max - x_min) / resolution))
        height = int(np.ceil((y_max - y_min) / resolution))
        
        # 计数网格
        grid_counts = np.zeros((height, width), dtype=np.int16)
        
        # 向量化映射
        ix = np.floor((points_xy[:, 0] - x_min) / resolution).astype(int)
        iy = np.floor((points_xy[:, 1] - y_min) / resolution).astype(int)
        
        valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        ix, iy = ix[valid], iy[valid]
        np.add.at(grid_counts, (-iy, ix), 1)  # Y轴取反
        
        # 生成 occupancy 数据
        data = np.zeros((height, width), dtype=np.int8)
        data[grid_counts >= occupied_thresh] = 100      # occupied
        data[(grid_counts > 0) & (grid_counts < occupied_thresh)] = -1  # unknown
        # 其余为 0 (free)
        
        # ROS OccupancyGrid 要求从左下角开始，Y轴向上 → 翻转
        data = data[::-1, :]
        
        return {
            'data': data,
            'resolution': resolution,
            'origin': (x_min, y_min),
            'width': width,
            'height': height
        }
    
    def get_point_cloud(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        获取完整点云
        
        Returns:
            points: 点云坐标 (N, 3)
            colors: 点云颜色 (N, 3)
        """
        if self.enable_sliding_window:
            # 合并滑动窗口内的所有点云
            if len(self.point_cloud_frames) == 0:
                return np.array([]), None
            
            merged_cloud = o3d.geometry.PointCloud()
            for frame_cloud in self.point_cloud_frames:
                merged_cloud += frame_cloud
            
            points = np.asarray(merged_cloud.points)
            colors = np.asarray(merged_cloud.colors) if merged_cloud.has_colors() else None
        else:
            # 返回累积点云
            points = np.asarray(self.accumulated_cloud.points)
            colors = np.asarray(self.accumulated_cloud.colors) if self.accumulated_cloud.has_colors() else None
        
        return points, colors
    
    def get_open3d_pointcloud(self) -> o3d.geometry.PointCloud:
        """
        获取Open3D点云对象
        
        Returns:
            pcd: Open3D点云对象
        """
        if self.enable_sliding_window:
            merged_cloud = o3d.geometry.PointCloud()
            for frame_cloud in self.point_cloud_frames:
                merged_cloud += frame_cloud
            return merged_cloud
        else:
            return self.accumulated_cloud
    
    def clear(self) -> None:
        """清空地图"""
        if self.enable_sliding_window:
            self.point_cloud_frames.clear()
        else:
            self.accumulated_cloud.clear()
    
    def save(self, filepath: str) -> None:
        """
        保存点云地图
        
        Args:
            filepath: 保存路径（支持 .ply, .pcd 等格式）
        """
        pcd = self.get_open3d_pointcloud()
        o3d.io.write_point_cloud(filepath, pcd)
        print(f"✓ 点云地图已保存到: {filepath}")
    
    def get_frame_count(self) -> int:
        """
        获取当前帧数
        
        Returns:
            count: 帧数（滑动窗口模式）或 1（累积模式）
        """
        if self.enable_sliding_window:
            return len(self.point_cloud_frames)
        else:
            return 1 if len(self.accumulated_cloud.points) > 0 else 0