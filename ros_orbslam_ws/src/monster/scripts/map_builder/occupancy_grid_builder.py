"""
基于占用栅格的地图构建器实现
支持视野内点云实时更新 + 历史点云保留的混合模式
"""

import numpy as np
import open3d as o3d
from typing import Dict, Tuple, Optional, Any, List


class OccupancyGridBuilder:
    """基于占用栅格的地图构建器 - 支持FOV感知的点云管理"""
    
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
                    'fov_aware': {
                        'enabled': bool,
                        'horizontal_fov': float,  # 水平视场角（度）
                        'vertical_fov': float,    # 垂直视场角（度）
                        'max_distance': float,    # 最大可见距离（米）
                        'voxel_grid_size': float  # 体素网格大小（米）
                    },
                    'max_points': int,  # 最大点数限制（可选）
                    'max_history_points': int  # 历史点云最大点数
                }
        """
        self.config = config
        
        # 滑动窗口配置
        sliding_window_config = config.get('sliding_window', {})
        self.enable_sliding_window = sliding_window_config.get('enabled', True)
        self.sliding_window_size = sliding_window_config.get('size', 3)
        
        # FOV感知配置
        fov_config = config.get('fov_aware', {})
        self.enable_fov_aware = fov_config.get('enabled', False)
        self.fov_params = {
            'horizontal_fov': fov_config.get('horizontal_fov', 90.0),
            'vertical_fov': fov_config.get('vertical_fov', 60.0),
            'max_distance': fov_config.get('max_distance', 5.0),
            'voxel_grid_size': fov_config.get('voxel_grid_size', 0.2)
        }
        
        # 点云存储
        if self.enable_fov_aware:
            # FOV感知模式：视野内点云 + 历史点云
            self.fov_point_cloud = o3d.geometry.PointCloud()
            self.history_point_cloud = o3d.geometry.PointCloud()
            self.current_camera_pose = None
            self.max_history_points = config.get('max_history_points', 100000)
        elif self.enable_sliding_window:
            # 滑动窗口模式：存储每帧点云
            self.point_cloud_frames: List[o3d.geometry.PointCloud] = []
        else:
            # 累积模式：存储单个累积点云
            self.accumulated_cloud = o3d.geometry.PointCloud()
        
        # 最大点数限制
        self.max_points = config.get('max_points', None)
        
        print(f"✓ OccupancyGridBuilder 初始化完成")
        if self.enable_fov_aware:
            print(f"  - 模式: FOV感知（视野内更新 + 历史保留）")
            print(f"  - 水平FOV: {self.fov_params['horizontal_fov']}°")
            print(f"  - 垂直FOV: {self.fov_params['vertical_fov']}°")
            print(f"  - 最大距离: {self.fov_params['max_distance']}m")
            print(f"  - 体素网格: {self.fov_params['voxel_grid_size']}m")
            print(f"  - 历史点云限制: {self.max_history_points} 点")
        else:
            print(f"  - 滑动窗口: {'启用' if self.enable_sliding_window else '禁用'}")
            if self.enable_sliding_window:
                print(f"  - 窗口大小: {self.sliding_window_size} 帧")
            if self.max_points:
                print(f"  - 最大点数: {self.max_points}")
    
    def _transform_to_camera_frame(self, points: np.ndarray, camera_pose: np.ndarray) -> np.ndarray:
        """
        将世界坐标系中的点转换到相机坐标系
        
        Args:
            points: 世界坐标系中的点 (N, 3)
            camera_pose: 相机位姿 (4x4 变换矩阵 Twc)
        
        Returns:
            points_cam: 相机坐标系中的点 (N, 3)
        """
        # Twc -> Tcw (求逆)
        R_wc = camera_pose[:3, :3]
        t_wc = camera_pose[:3, 3]
        
        R_cw = R_wc.T
        t_cw = -R_cw @ t_wc
        
        # 转换点云
        points_cam = (R_cw @ points.T).T + t_cw
        return points_cam
    
    def _compute_fov_mask_fast(self, points: np.ndarray, camera_pose: np.ndarray) -> np.ndarray:
        """
        快速计算哪些点在相机FOV内（使用体素网格加速）
        
        Args:
            points: 世界坐标系中的点 (N, 3)
            camera_pose: 相机位姿 (4x4 变换矩阵 Twc)
        
        Returns:
            mask: 布尔数组，True表示在FOV内 (N,)
        """
        if len(points) == 0:
            return np.array([], dtype=bool)
        
        # 转换到相机坐标系
        points_cam = self._transform_to_camera_frame(points, camera_pose)
        
        # 深度检查
        depth = points_cam[:, 2]
        depth_mask = (depth > 0.1) & (depth <= self.fov_params['max_distance'])
        
        # 角度检查
        h_fov_half = np.deg2rad(self.fov_params['horizontal_fov'] / 2.0)
        v_fov_half = np.deg2rad(self.fov_params['vertical_fov'] / 2.0)
        
        # 计算水平和垂直角度
        horizontal_angle = np.abs(np.arctan2(points_cam[:, 0], points_cam[:, 2]))
        vertical_angle = np.abs(np.arctan2(points_cam[:, 1], points_cam[:, 2]))
        
        angle_mask = (horizontal_angle <= h_fov_half) & (vertical_angle <= v_fov_half)
        
        # 组合所有条件
        fov_mask = depth_mask & angle_mask
        
        return fov_mask
    
    def update(self,
               points: np.ndarray,
               colors: Optional[np.ndarray] = None,
               camera_pose: Optional[np.ndarray] = None) -> None:
        """
        更新地图
        
        Args:
            points: 新的点云 (N, 3) - 世界坐标系
            colors: 点云颜色 (N, 3)
            camera_pose: 相机位姿 (4x4 变换矩阵 Twc)，FOV感知模式必需
        """
        if len(points) == 0:
            return
        
        # 创建点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if self.enable_fov_aware:
            # FOV感知模式：视野内更新 + 历史保留
            if camera_pose is None:
                print("⚠️ FOV感知模式需要相机位姿，跳过更新")
                return
            
            self._update_fov_aware(points, colors, camera_pose)
            
        elif self.enable_sliding_window:
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
    
    def _update_fov_aware(self, points: np.ndarray, colors: Optional[np.ndarray],
                          camera_pose: np.ndarray) -> None:
        """
        FOV感知模式的更新逻辑：视野内更新，视野外保留
        
        Args:
            points: 新的点云 (N, 3)
            colors: 点云颜色 (N, 3)
            camera_pose: 相机位姿 (4x4 变换矩阵)
        """
        # === 步骤1: 分类新点云 ===
        new_fov_mask = self._compute_fov_mask_fast(points, camera_pose)
        new_fov_points = points[new_fov_mask]
        new_fov_colors = colors[new_fov_mask] if colors is not None else None
        
        # === 步骤2: 处理旧的视野内点云 ===
        if len(self.fov_point_cloud.points) > 0:
            old_fov_points = np.asarray(self.fov_point_cloud.points)
            old_fov_mask = self._compute_fov_mask_fast(old_fov_points, camera_pose)
            
            # 离开视野的点移到历史
            leaving_mask = ~old_fov_mask
            if np.any(leaving_mask):
                leaving_points = old_fov_points[leaving_mask]
                leaving_pcd = o3d.geometry.PointCloud()
                leaving_pcd.points = o3d.utility.Vector3dVector(leaving_points)
                
                if self.fov_point_cloud.has_colors():
                    old_colors = np.asarray(self.fov_point_cloud.colors)
                    leaving_pcd.colors = o3d.utility.Vector3dVector(old_colors[leaving_mask])
                
                self.history_point_cloud += leaving_pcd
        
        # === 步骤3: 清理历史点云中的FOV区域（避免重复）===
        if len(self.history_point_cloud.points) > 0:
            history_points = np.asarray(self.history_point_cloud.points)
            history_fov_mask = self._compute_fov_mask_fast(history_points, camera_pose)
            
            # 只保留视野外的历史点
            history_out_mask = ~history_fov_mask
            if np.any(history_out_mask):
                self.history_point_cloud.points = o3d.utility.Vector3dVector(
                    history_points[history_out_mask]
                )
                if self.history_point_cloud.has_colors():
                    history_colors = np.asarray(self.history_point_cloud.colors)
                    self.history_point_cloud.colors = o3d.utility.Vector3dVector(
                        history_colors[history_out_mask]
                    )
            else:
                # 所有历史点都在FOV内，清空历史
                self.history_point_cloud.clear()
        
        # === 步骤4: 完全替换视野内点云（不累加）===
        self.fov_point_cloud.clear()
        if len(new_fov_points) > 0:
            self.fov_point_cloud.points = o3d.utility.Vector3dVector(new_fov_points)
            if new_fov_colors is not None:
                self.fov_point_cloud.colors = o3d.utility.Vector3dVector(new_fov_colors)
        
        # === 步骤5: 限制历史点云数量 ===
        if len(self.history_point_cloud.points) > self.max_history_points:
            ratio = self.max_history_points / len(self.history_point_cloud.points)
            self.history_point_cloud = self.history_point_cloud.random_down_sample(ratio)
            print(f"📉 历史点云下采样: {len(self.history_point_cloud.points)} 点")
        
        # 更新相机位姿
        self.current_camera_pose = camera_pose
    
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
        if self.enable_fov_aware:
            # FOV感知模式：合并视野内 + 历史点云
            merged_cloud = o3d.geometry.PointCloud()
            merged_cloud += self.fov_point_cloud
            merged_cloud += self.history_point_cloud
            
            points = np.asarray(merged_cloud.points)
            colors = np.asarray(merged_cloud.colors) if merged_cloud.has_colors() else None
            
        elif self.enable_sliding_window:
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
        if self.enable_fov_aware:
            merged_cloud = o3d.geometry.PointCloud()
            merged_cloud += self.fov_point_cloud
            merged_cloud += self.history_point_cloud
            return merged_cloud
        elif self.enable_sliding_window:
            merged_cloud = o3d.geometry.PointCloud()
            for frame_cloud in self.point_cloud_frames:
                merged_cloud += frame_cloud
            return merged_cloud
        else:
            return self.accumulated_cloud
    
    def clear(self) -> None:
        """清空地图"""
        if self.enable_fov_aware:
            self.fov_point_cloud.clear()
            self.history_point_cloud.clear()
            self.current_camera_pose = None
        elif self.enable_sliding_window:
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
        if self.enable_fov_aware:
            fov_count = len(self.fov_point_cloud.points)
            history_count = len(self.history_point_cloud.points)
            return 1 if (fov_count > 0 or history_count > 0) else 0
        elif self.enable_sliding_window:
            return len(self.point_cloud_frames)
        else:
            return 1 if len(self.accumulated_cloud.points) > 0 else 0
    
    def get_fov_statistics(self) -> Dict[str, int]:
        """
        获取FOV感知模式的统计信息
        
        Returns:
            stats: 统计字典 {'fov_points': int, 'history_points': int, 'total_points': int}
        """
        if not self.enable_fov_aware:
            return {'fov_points': 0, 'history_points': 0, 'total_points': 0}
        
        fov_count = len(self.fov_point_cloud.points)
        history_count = len(self.history_point_cloud.points)
        
        return {
            'fov_points': fov_count,
            'history_points': history_count,
            'total_points': fov_count + history_count
        }