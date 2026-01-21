"""
Pipeline Manager - 处理流程管理器

协调深度估计、点云生成和地图构建模块
"""

import yaml
import numpy as np
from typing import Dict, Any, Optional
import time

from depth_estimator import BaseDepthEstimator, DepthAnythingV2Estimator
from point_cloud import BasePointCloudGenerator, Open3DPointCloudGenerator
from map_builder import BaseMapBuilder, OccupancyGridBuilder


class PipelineManager:
    """处理流程管理器"""
    
    def __init__(self, config_path: Optional[str] = None, config_dict: Optional[Dict] = None):
        """
        初始化管理器
        
        Args:
            config_path: 配置文件路径（YAML格式）
            config_dict: 配置字典（如果提供，优先使用）
        """
        # 加载配置
        if config_dict is not None:
            self.config = config_dict
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            raise ValueError("必须提供 config_path 或 config_dict")
        
        # 初始化各模块
        self.depth_estimator = self._create_depth_estimator()
        self.point_cloud_generator = self._create_point_cloud_generator()
        self.map_builder = self._create_map_builder()
        
        # 性能监控
        self.enable_profiling = self.config.get('profiling', {}).get('enabled', False)
        self.profiling_data = {
            'depth_estimation': [],
            'point_cloud_generation': [],
            'point_cloud_filtering': [],
            'point_cloud_downsampling': [],
            'map_update': [],
            'total': []
        }
        
        print("=" * 60)
        print("✓ Pipeline Manager 初始化完成")
        print(f"  - 深度估计器: {self.depth_estimator}")
        print(f"  - 点云生成器: Open3DPointCloudGenerator")
        print(f"  - 地图构建器: OccupancyGridBuilder")
        print(f"  - 性能分析: {'启用' if self.enable_profiling else '禁用'}")
        print("=" * 60)
    
    def process_frame(self, 
                     image: np.ndarray,
                     pose: np.ndarray,
                     camera_params: Dict[str, float]) -> Dict[str, Any]:
        """
        处理单帧数据
        
        Args:
            image: RGB图像 (H, W, 3), uint8
            pose: 位姿矩阵 (4, 4) 或 pypose.SE3对象
            camera_params: 相机内参 {'fx', 'fy', 'cx', 'cy'}
            
        Returns:
            result: 处理结果
                {
                    'depth': np.ndarray,           # 深度图
                    'points': np.ndarray,          # 点云坐标
                    'colors': np.ndarray,          # 点云颜色
                    'map': Dict,                   # 2D地图（可选）
                    'profiling': Dict              # 性能数据（如果启用）
                }
        """
        frame_start = time.time()
        profiling = {}
        
        # 1. 深度估计
        t0 = time.time()
        depth = self.depth_estimator.estimate(image)
        # 应用后处理（包括可选的降噪）
        depth = self.depth_estimator.postprocess(depth)
        depth_time = time.time() - t0
        if self.enable_profiling:
            profiling['depth_estimation'] = depth_time
            self.profiling_data['depth_estimation'].append(depth_time)
        
        # 2. 点云生成
        t0 = time.time()
        pc_config = self.config.get('point_cloud', {})
        
        # 检查是否使用裁剪模式
        if 'crop_params' in pc_config:
            points, colors = self.point_cloud_generator.generate_with_crop(
                depth, image, camera_params, pose, pc_config['crop_params']
            )
        else:
            points, colors = self.point_cloud_generator.generate(
                depth, image, camera_params, pose
            )
        pc_gen_time = time.time() - t0
        if self.enable_profiling:
            profiling['point_cloud_generation'] = pc_gen_time
            self.profiling_data['point_cloud_generation'].append(pc_gen_time)
        
        # 3. 点云过滤
        t0 = time.time()
        if 'filter' in pc_config:
            points, colors = self.point_cloud_generator.filter(
                points, colors, pc_config['filter']
            )
        filter_time = time.time() - t0
        if self.enable_profiling:
            profiling['point_cloud_filtering'] = filter_time
            self.profiling_data['point_cloud_filtering'].append(filter_time)
        
        # 4. 点云下采样
        t0 = time.time()
        voxel_size = pc_config.get('voxel_size', 1.0)
        points, colors = self.point_cloud_generator.downsample(
            points, colors, voxel_size
        )
        downsample_time = time.time() - t0
        if self.enable_profiling:
            profiling['point_cloud_downsampling'] = downsample_time
            self.profiling_data['point_cloud_downsampling'].append(downsample_time)
        
        # 5. 更新地图
        t0 = time.time()
        self.map_builder.update(points, colors)
        map_update_time = time.time() - t0
        if self.enable_profiling:
            profiling['map_update'] = map_update_time
            self.profiling_data['map_update'].append(map_update_time)
        
        # 总时间
        total_time = time.time() - frame_start
        if self.enable_profiling:
            profiling['total'] = total_time
            self.profiling_data['total'].append(total_time)
        
        return {
            'depth': depth,
            'points': points,
            'colors': colors,
            'profiling': profiling if self.enable_profiling else None
        }
    
    def get_2d_map(self, 
                   resolution: Optional[float] = None,
                   height_range: Optional[tuple] = None,
                   occupied_thresh: Optional[int] = None,
                   use_ratio: Optional[bool] = None) -> Optional[Dict[str, Any]]:
        """
        获取2D占用栅格地图
        
        Args:
            resolution: 网格分辨率（米/格），None则使用配置
            height_range: 高度范围，None则使用配置
            occupied_thresh: 占用阈值，None则使用配置
            use_ratio: 是否使用百分比模式，None则使用配置
            
        Returns:
            grid_map: 地图字典或None
        """
        map_config = self.config.get('map', {})
        
        # 使用参数或配置
        resolution = resolution or map_config.get('resolution', 0.8)
        height_range = height_range or tuple(map_config.get('height_range', [0.3, 0.7]))
        occupied_thresh = occupied_thresh or map_config.get('occupied_thresh', 5)
        use_ratio = use_ratio if use_ratio is not None else map_config.get('use_ratio', True)
        
        return self.map_builder.get_occupancy_grid(
            resolution=resolution,
            height_range=height_range,
            occupied_thresh=occupied_thresh,
            use_ratio=use_ratio
        )
    
    def get_point_cloud(self):
        """获取完整点云"""
        return self.map_builder.get_point_cloud()
    
    def get_open3d_pointcloud(self):
        """获取Open3D点云对象"""
        return self.map_builder.get_open3d_pointcloud()
    
    def clear_map(self):
        """清空地图"""
        self.map_builder.clear()
    
    def save_map(self, filepath: str):
        """保存地图"""
        self.map_builder.save(filepath)
    
    def get_profiling_summary(self) -> Dict[str, Any]:
        """
        获取性能分析摘要
        
        Returns:
            summary: 性能摘要
        """
        if not self.enable_profiling:
            return None
        
        summary = {}
        for key, values in self.profiling_data.items():
            if len(values) > 0:
                summary[key] = {
                    'mean': np.mean(values),
                    'std': np.std(values),
                    'min': np.min(values),
                    'max': np.max(values),
                    'count': len(values)
                }
        
        return summary
    
    def print_profiling_summary(self):
        """打印性能分析摘要"""
        summary = self.get_profiling_summary()
        if summary is None:
            print("性能分析未启用")
            return
        
        print("\n" + "=" * 60)
        print("性能分析摘要")
        print("=" * 60)
        for key, stats in summary.items():
            print(f"{key}:")
            print(f"  平均: {stats['mean']*1000:.2f}ms")
            print(f"  标准差: {stats['std']*1000:.2f}ms")
            print(f"  最小: {stats['min']*1000:.2f}ms")
            print(f"  最大: {stats['max']*1000:.2f}ms")
            print(f"  样本数: {stats['count']}")
        
        if 'total' in summary:
            avg_fps = 1.0 / summary['total']['mean']
            print(f"\n平均FPS: {avg_fps:.2f}")
        print("=" * 60 + "\n")
    
    def _create_depth_estimator(self) -> BaseDepthEstimator:
        """创建深度估计器"""
        estimator_config = self.config['depth_estimator']
        estimator_type = estimator_config['type']
        
        if estimator_type == 'depth_anything_v2':
            estimator = DepthAnythingV2Estimator()
        else:
            raise ValueError(f"不支持的深度估计器类型: {estimator_type}")
        
        estimator.initialize(estimator_config)
        return estimator
    
    def _create_point_cloud_generator(self) -> BasePointCloudGenerator:
        """创建点云生成器"""
        # 目前只支持Open3D
        return Open3DPointCloudGenerator()
    
    def _create_map_builder(self) -> BaseMapBuilder:
        """创建地图构建器"""
        map_config = self.config.get('map', {})
        builder_type = map_config.get('type', 'occupancy_grid')
        
        if builder_type == 'occupancy_grid':
            return OccupancyGridBuilder(map_config)
        else:
            raise ValueError(f"不支持的地图构建器类型: {builder_type}")
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return config
