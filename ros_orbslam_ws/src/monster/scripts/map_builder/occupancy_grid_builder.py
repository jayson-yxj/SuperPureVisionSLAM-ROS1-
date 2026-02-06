"""
基于占用栅格的地图构建器实现
支持视野内点云实时更新 + 历史点云保留的混合模式
支持 TSDF 融合模式：视野内加权融合 + 视野外保留
"""

import numpy as np
import open3d as o3d
from typing import Dict, Tuple, Optional, Any, List
from collections import defaultdict


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
        
        # TSDF融合配置
        tsdf_config = fov_config.get('tsdf_fusion', {})
        self.enable_tsdf_fusion = tsdf_config.get('enabled', False) and self.enable_fov_aware
        self.tsdf_params = {
            'max_weight': tsdf_config.get('max_weight', 10.0),
            'truncation_distance': tsdf_config.get('truncation_distance', 0.1),
            'weight_params': tsdf_config.get('weight_params', {})
        }
        
        # 点云存储
        if self.enable_fov_aware:
            # FOV感知模式：视野内点云 + 历史点云
            self.fov_point_cloud = o3d.geometry.PointCloud()
            self.history_point_cloud = o3d.geometry.PointCloud()
            self.current_camera_pose = None
            self.max_history_points = config.get('max_history_points', 100000)
            
            # TSDF融合模式额外存储
            if self.enable_tsdf_fusion:
                # 体素字典：key=(voxel_x, voxel_y, voxel_z), value={'tsdf': float, 'weight': float, 'color': [r,g,b]}
                self.tsdf_volume = defaultdict(lambda: {'tsdf': 0.0, 'weight': 0.0, 'color': np.array([0.0, 0.0, 0.0])})
                self.voxel_size = self.fov_params['voxel_grid_size']
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
            if self.enable_tsdf_fusion:
                print(f"  - 模式: TSDF融合（视野内加权融合 + 历史保留）")
                print(f"  - 最大权重: {self.tsdf_params['max_weight']}")
                print(f"  - 截断距离: {self.tsdf_params['truncation_distance']}m")
            else:
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
    
    def _compute_observation_weights(self, points: np.ndarray, colors: Optional[np.ndarray],
                                    camera_pose: np.ndarray, image_width: int = 640,
                                    image_height: int = 480) -> np.ndarray:
        """
        计算观测权重（基于边缘、距离、角度）
        
        Args:
            points: 世界坐标系中的点 (N, 3)
            colors: 点云颜色 (N, 3)
            camera_pose: 相机位姿 (4x4 变换矩阵)
            image_width: 图像宽度（用于边缘权重计算）
            image_height: 图像高度
        
        Returns:
            weights: 观测权重 (N,)，范围 [0, 1]
        """
        if len(points) == 0:
            return np.array([])
        
        # 转换到相机坐标系
        points_cam = self._transform_to_camera_frame(points, camera_pose)
        
        # 获取权重参数
        weight_params = self.tsdf_params['weight_params']
        
        # 初始化总权重
        weights = np.ones(len(points))
        
        # === 1. 边缘权重 ===
        edge_config = weight_params.get('edge_weight', {})
        if edge_config.get('enabled', True):
            # 投影到图像平面
            fx = fy = 365.485  # 可以从配置读取
            cx = image_width / 2.0
            cy = image_height / 2.0
            
            u = fx * points_cam[:, 0] / points_cam[:, 2] + cx
            v = fy * points_cam[:, 1] / points_cam[:, 2] + cy
            
            # 归一化到 [-1, 1]
            u_norm = 2 * (u - cx) / image_width
            v_norm = 2 * (v - cy) / image_height
            
            # 到中心的距离
            dist_to_center = np.sqrt(u_norm**2 + v_norm**2)
            
            # 高斯衰减
            sigma = edge_config.get('sigma', 0.7)
            edge_weight = np.exp(-(dist_to_center**2) / (2 * sigma**2))
            weights *= edge_weight
        
        # === 2. 距离权重 ===
        distance_config = weight_params.get('distance_weight', {})
        if distance_config.get('enabled', True):
            depth = points_cam[:, 2]
            optimal_depth = distance_config.get('optimal_depth', 0.9)
            sigma = distance_config.get('sigma', 0.3)
            min_depth = distance_config.get('min_depth', 0.3)
            max_depth = distance_config.get('max_depth', 25.0)
            
            # 高斯函数，峰值在 optimal_depth
            distance_weight = np.exp(-((depth - optimal_depth)**2) / (2 * sigma**2))
            
            # 超出范围的点权重为0
            distance_weight[depth < min_depth] = 0.0
            distance_weight[depth > max_depth] = 0.0
            
            weights *= distance_weight
        
        # === 3. 角度权重 ===
        angle_config = weight_params.get('angle_weight', {})
        if angle_config.get('enabled', True):
            # 计算观测方向（相机到点的向量）
            view_dirs = points_cam / np.linalg.norm(points_cam, axis=1, keepdims=True)
            
            # 假设表面法向量指向相机（简化，实际可以从点云法向量计算）
            # 这里使用观测角度：与Z轴的夹角
            cos_angle = np.abs(view_dirs[:, 2])  # Z分量就是cos(angle)
            
            max_angle = np.deg2rad(angle_config.get('max_angle', 60.0))
            falloff_power = angle_config.get('falloff_power', 2.0)
            
            # 角度权重：正面观测权重高
            angle_weight = np.power(cos_angle, falloff_power)
            angle_weight[cos_angle < np.cos(max_angle)] = 0.0
            
            weights *= angle_weight
        
        # 归一化到 [0, 1]
        weights = np.clip(weights, 0.0, 1.0)
        
        return weights
    
    def _point_to_voxel(self, point: np.ndarray) -> Tuple[int, int, int]:
        """将点坐标转换为体素索引"""
        return tuple((point / self.voxel_size).astype(int))
    
    def _voxel_to_point(self, voxel: Tuple[int, int, int]) -> np.ndarray:
        """将体素索引转换为点坐标（体素中心）"""
        return np.array(voxel) * self.voxel_size + self.voxel_size / 2.0
    
    def _update_tsdf_fusion(self, points: np.ndarray, colors: Optional[np.ndarray],
                           camera_pose: np.ndarray) -> None:
        """
        TSDF融合模式的更新逻辑：视野内加权融合，视野外保留（向量化优化版本）
        
        Args:
            points: 新的点云 (N, 3)
            colors: 点云颜色 (N, 3)
            camera_pose: 相机位姿 (4x4 变换矩阵)
        """
        if len(points) == 0:
            return
        
        # === 步骤1: 计算观测权重（向量化） ===
        weights = self._compute_observation_weights(points, colors, camera_pose)
        
        # 过滤权重为0的点
        valid_mask = weights > 0.0
        if not np.any(valid_mask):
            print("⚠️ 所有点的权重为0，跳过TSDF更新")
            return
        
        points = points[valid_mask]
        colors = colors[valid_mask] if colors is not None else None
        weights = weights[valid_mask]
        
        # === 步骤2: 向量化体素索引计算 ===
        truncation = self.tsdf_params['truncation_distance']
        max_weight = self.tsdf_params['max_weight']
        
        # 批量计算体素索引 (N, 3)
        voxel_indices = (points / self.voxel_size).astype(int)
        
        # 批量计算体素中心 (N, 3)
        voxel_centers = voxel_indices * self.voxel_size + self.voxel_size / 2.0
        
        # 批量计算SDF值 (N,)
        sdf_values = np.linalg.norm(points - voxel_centers, axis=1)
        
        # 批量截断TSDF (N,)
        tsdf_values = np.clip(sdf_values, -truncation, truncation) / truncation
        
        # === 步骤3: 按唯一体素聚合观测（向量化） ===
        # 找到唯一体素及其逆索引
        voxel_tuples = [tuple(v) for v in voxel_indices]
        unique_voxels_dict = {}
        
        for i, voxel_tuple in enumerate(voxel_tuples):
            if voxel_tuple not in unique_voxels_dict:
                unique_voxels_dict[voxel_tuple] = []
            unique_voxels_dict[voxel_tuple].append(i)
        
        updated_voxels = set()
        
        # 对每个唯一体素进行融合
        for voxel_idx, point_indices in unique_voxels_dict.items():
            # 该体素的所有观测
            voxel_tsdf_values = tsdf_values[point_indices]
            voxel_weights = weights[point_indices]
            
            # 加权平均新观测
            new_tsdf = np.average(voxel_tsdf_values, weights=voxel_weights)
            new_weight = np.sum(voxel_weights)
            
            # 获取旧值
            voxel_data = self.tsdf_volume[voxel_idx]
            old_tsdf = voxel_data['tsdf']
            old_weight = voxel_data['weight']
            
            # 融合
            total_weight = old_weight + new_weight
            if total_weight > 0:
                fused_tsdf = (old_tsdf * old_weight + new_tsdf * new_weight) / total_weight
                fused_weight = min(total_weight, max_weight)
                
                # 更新体素
                self.tsdf_volume[voxel_idx]['tsdf'] = fused_tsdf
                self.tsdf_volume[voxel_idx]['weight'] = fused_weight
                
                # 融合颜色（向量化）
                if colors is not None:
                    old_color = voxel_data['color']
                    voxel_colors = colors[point_indices]
                    new_color = np.average(voxel_colors, axis=0, weights=voxel_weights)
                    fused_color = (old_color * old_weight + new_color * new_weight) / total_weight
                    self.tsdf_volume[voxel_idx]['color'] = fused_color
                
                updated_voxels.add(voxel_idx)
        
        # === 步骤4: 向量化重建FOV点云 ===
        weight_threshold = 0.1
        
        if len(updated_voxels) > 0:
            # 批量提取更新体素的数据
            fov_voxel_list = list(updated_voxels)
            fov_weights = np.array([self.tsdf_volume[v]['weight'] for v in fov_voxel_list])
            
            # 过滤低权重体素
            valid_fov_mask = fov_weights >= weight_threshold
            if np.any(valid_fov_mask):
                valid_fov_voxels = [fov_voxel_list[i] for i in range(len(fov_voxel_list)) if valid_fov_mask[i]]
                
                # 批量计算体素中心点
                fov_voxel_array = np.array(valid_fov_voxels)
                fov_points = fov_voxel_array * self.voxel_size + self.voxel_size / 2.0
                fov_colors = np.array([self.tsdf_volume[v]['color'] for v in valid_fov_voxels])
                
                # 更新视野内点云
                self.fov_point_cloud.clear()
                self.fov_point_cloud.points = o3d.utility.Vector3dVector(fov_points)
                self.fov_point_cloud.colors = o3d.utility.Vector3dVector(fov_colors)
            else:
                self.fov_point_cloud.clear()
        else:
            self.fov_point_cloud.clear()
        
        # === 步骤5: 向量化重建历史点云 ===
        if len(self.tsdf_volume) > len(updated_voxels):
            # 提取所有非更新体素
            all_voxels = list(self.tsdf_volume.keys())
            history_voxels = [v for v in all_voxels if v not in updated_voxels]
            
            if len(history_voxels) > 0:
                # 批量提取权重
                history_weights = np.array([self.tsdf_volume[v]['weight'] for v in history_voxels])
                valid_history_mask = history_weights >= weight_threshold
                
                if np.any(valid_history_mask):
                    valid_history_voxels = [history_voxels[i] for i in range(len(history_voxels)) if valid_history_mask[i]]
                    
                    # 批量计算体素中心
                    history_voxel_array = np.array(valid_history_voxels)
                    history_points = history_voxel_array * self.voxel_size + self.voxel_size / 2.0
                    
                    # 批量FOV检查
                    history_fov_mask = self._compute_fov_mask_fast(history_points, camera_pose)
                    history_out_mask = ~history_fov_mask
                    
                    if np.any(history_out_mask):
                        # 只保留FOV外的点
                        final_history_voxels = [valid_history_voxels[i] for i in range(len(valid_history_voxels)) if history_out_mask[i]]
                        final_history_points = history_points[history_out_mask]
                        final_history_colors = np.array([self.tsdf_volume[v]['color'] for v in final_history_voxels])
                        
                        self.history_point_cloud.clear()
                        self.history_point_cloud.points = o3d.utility.Vector3dVector(final_history_points)
                        self.history_point_cloud.colors = o3d.utility.Vector3dVector(final_history_colors)
                    else:
                        self.history_point_cloud.clear()
                else:
                    self.history_point_cloud.clear()
            else:
                self.history_point_cloud.clear()
        else:
            self.history_point_cloud.clear()
        
        # 更新相机位姿
        self.current_camera_pose = camera_pose
        
        # 打印统计信息
        if len(updated_voxels) > 0:
            avg_weight = np.mean([self.tsdf_volume[v]['weight'] for v in updated_voxels])
            print(f"🔄 TSDF更新: {len(updated_voxels)} 体素, 平均权重: {avg_weight:.2f}")
    
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
            
            if self.enable_tsdf_fusion:
                # TSDF融合模式
                self._update_tsdf_fusion(points, colors, camera_pose)
            else:
                # 标准FOV感知模式
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
            if self.enable_tsdf_fusion:
                self.tsdf_volume.clear()
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
    
    def get_fov_statistics(self) -> Dict[str, Any]:
        """
        获取FOV感知模式的统计信息
        
        Returns:
            stats: 统计字典
        """
        if not self.enable_fov_aware:
            return {'fov_points': 0, 'history_points': 0, 'total_points': 0}
        
        fov_count = len(self.fov_point_cloud.points)
        history_count = len(self.history_point_cloud.points)
        
        stats = {
            'fov_points': fov_count,
            'history_points': history_count,
            'total_points': fov_count + history_count
        }
        
        # TSDF模式额外统计
        if self.enable_tsdf_fusion:
            stats['tsdf_voxels'] = len(self.tsdf_volume)
            if len(self.tsdf_volume) > 0:
                weights = [v['weight'] for v in self.tsdf_volume.values()]
                stats['avg_weight'] = np.mean(weights)
                stats['max_weight'] = np.max(weights)
        
        return stats