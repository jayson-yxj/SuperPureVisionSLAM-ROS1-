"""
Parallel Pipeline Manager - 并行处理流程管理器

使用多线程实现深度估计、点云生成和地图更新的并行处理
"""

import yaml
import numpy as np
import threading
from queue import Queue, Empty, Full
from typing import Dict, Any, Optional
import time

from depth_estimator import BaseDepthEstimator, DepthAnythingV2Estimator
from point_cloud import BasePointCloudGenerator, Open3DPointCloudGenerator
from map_builder import BaseMapBuilder, OccupancyGridBuilder


class ParallelPipelineManager:
    """并行处理流程管理器"""
    
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
        
        # 并行处理配置
        parallel_config = self.config.get('parallel_processing', {})
        self.enable_parallel = parallel_config.get('enabled', False)
        self.queue_size = parallel_config.get('queue_size', 2)
        
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
        
        # 并行处理相关
        if self.enable_parallel:
            self._init_parallel_processing()
        
        print("=" * 60)
        print("✓ Parallel Pipeline Manager 初始化完成")
        print(f"  - 深度估计器: {self.depth_estimator}")
        print(f"  - 点云生成器: Open3DPointCloudGenerator")
        print(f"  - 地图构建器: OccupancyGridBuilder")
        print(f"  - 并行处理: {'启用' if self.enable_parallel else '禁用'}")
        if self.enable_parallel:
            print(f"  - 队列大小: {self.queue_size}")
        print(f"  - 性能分析: {'启用' if self.enable_profiling else '禁用'}")
        print("=" * 60)
    
    def _init_parallel_processing(self):
        """初始化并行处理"""
        # 线程安全的队列
        self.depth_queue = Queue(maxsize=self.queue_size)
        self.pointcloud_queue = Queue(maxsize=self.queue_size)
        self.map_queue = Queue(maxsize=self.queue_size)
        
        # 线程控制
        self.running = True
        self.threads = []
        
        # 启动工作线程
        depth_thread = threading.Thread(target=self._depth_worker, daemon=True, name="DepthWorker")
        pointcloud_thread = threading.Thread(target=self._pointcloud_worker, daemon=True, name="PointCloudWorker")
        map_thread = threading.Thread(target=self._map_worker, daemon=True, name="MapWorker")
        
        depth_thread.start()
        pointcloud_thread.start()
        map_thread.start()
        
        self.threads = [depth_thread, pointcloud_thread, map_thread]
        
        print("✓ 并行处理线程已启动")
    
    def _depth_worker(self):
        """深度估计工作线程"""
        while self.running:
            try:
                # 从队列获取任务（超时1秒）
                task = self.depth_queue.get(timeout=1.0)
                if task is None:  # 停止信号
                    break
                
                image, task_id = task
                
                # 深度估计
                t0 = time.time()
                depth = self.depth_estimator.estimate(image)
                depth = self.depth_estimator.postprocess(depth)
                depth_time = time.time() - t0
                
                # 将结果放入下一个队列
                try:
                    self.pointcloud_queue.put((depth, image, task_id, depth_time), timeout=1.0)
                except Full:
                    print("⚠️  点云队列已满，丢弃任务")
                
                self.depth_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                print(f"❌ 深度估计线程错误: {e}")
                import traceback
                traceback.print_exc()
    
    def _pointcloud_worker(self):
        """点云生成工作线程"""
        while self.running:
            try:
                # 从队列获取任务
                task = self.pointcloud_queue.get(timeout=1.0)
                if task is None:  # 停止信号
                    break
                
                depth, image, task_id, depth_time = task
                camera_params, pose = task_id['camera_params'], task_id['pose']
                
                # 点云生成
                t0 = time.time()
                pc_config = self.config.get('point_cloud', {})
                
                if 'crop_params' in pc_config:
                    points, colors = self.point_cloud_generator.generate_with_crop(
                        depth, image, camera_params, pose, pc_config['crop_params']
                    )
                else:
                    points, colors = self.point_cloud_generator.generate(
                        depth, image, camera_params, pose
                    )
                pc_gen_time = time.time() - t0
                
                # 点云过滤
                t0 = time.time()
                if 'filter' in pc_config:
                    points, colors = self.point_cloud_generator.filter(
                        points, colors, pc_config['filter']
                    )
                filter_time = time.time() - t0
                
                # 点云下采样
                t0 = time.time()
                voxel_size = pc_config.get('voxel_size', 1.0)
                points, colors = self.point_cloud_generator.downsample(
                    points, colors, voxel_size
                )
                downsample_time = time.time() - t0
                
                # 将结果放入地图队列
                try:
                    self.map_queue.put((
                        points, colors, task_id,
                        depth_time, pc_gen_time, filter_time, downsample_time
                    ), timeout=1.0)
                except Full:
                    print("⚠️  地图队列已满，丢弃任务")
                
                self.pointcloud_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                print(f"❌ 点云生成线程错误: {e}")
                import traceback
                traceback.print_exc()
    
    def _map_worker(self):
        """地图更新工作线程"""
        while self.running:
            try:
                # 从队列获取任务
                task = self.map_queue.get(timeout=1.0)
                if task is None:  # 停止信号
                    break
                
                points, colors, task_id, depth_time, pc_gen_time, filter_time, downsample_time = task
                
                # 更新地图
                t0 = time.time()
                self.map_builder.update(points, colors)
                map_update_time = time.time() - t0
                
                # 记录性能数据
                if self.enable_profiling:
                    self.profiling_data['depth_estimation'].append(depth_time)
                    self.profiling_data['point_cloud_generation'].append(pc_gen_time)
                    self.profiling_data['point_cloud_filtering'].append(filter_time)
                    self.profiling_data['point_cloud_downsampling'].append(downsample_time)
                    self.profiling_data['map_update'].append(map_update_time)
                    total_time = depth_time + pc_gen_time + filter_time + downsample_time + map_update_time
                    self.profiling_data['total'].append(total_time)
                
                self.map_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                print(f"❌ 地图更新线程错误: {e}")
                import traceback
                traceback.print_exc()
    
    def process_frame_async(self, 
                           image: np.ndarray,
                           pose: np.ndarray,
                           camera_params: Dict[str, float]) -> bool:
        """
        异步处理单帧数据（并行模式）
        
        Args:
            image: RGB图像 (H, W, 3), uint8
            pose: 位姿矩阵 (4, 4) 或 pypose.SE3对象
            camera_params: 相机内参 {'fx', 'fy', 'cx', 'cy'}
            
        Returns:
            success: 是否成功提交任务
        """
        if not self.enable_parallel:
            raise RuntimeError("并行处理未启用，请使用 process_frame()")
        
        # 创建任务ID
        task_id = {
            'camera_params': camera_params,
            'pose': pose,
            'timestamp': time.time()
        }
        
        # 提交到深度估计队列
        try:
            self.depth_queue.put((image.copy(), task_id), block=False)
            return True
        except Full:
            print("⚠️  深度队列已满，跳过当前帧")
            return False
    
    def process_frame(self, 
                     image: np.ndarray,
                     pose: np.ndarray,
                     camera_params: Dict[str, float]) -> Dict[str, Any]:
        """
        同步处理单帧数据（串行模式）
        
        Args:
            image: RGB图像 (H, W, 3), uint8
            pose: 位姿矩阵 (4, 4) 或 pypose.SE3对象
            camera_params: 相机内参 {'fx', 'fy', 'cx', 'cy'}
            
        Returns:
            result: 处理结果
        """
        frame_start = time.time()
        profiling = {}
        
        # 1. 深度估计
        t0 = time.time()
        depth = self.depth_estimator.estimate(image)
        depth = self.depth_estimator.postprocess(depth)
        depth_time = time.time() - t0
        if self.enable_profiling:
            profiling['depth_estimation'] = depth_time
            self.profiling_data['depth_estimation'].append(depth_time)
        
        # 2. 点云生成
        t0 = time.time()
        pc_config = self.config.get('point_cloud', {})
        
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
        """获取2D占用栅格地图"""
        map_config = self.config.get('map', {})
        
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
        """获取性能分析摘要"""
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
    
    def shutdown(self):
        """关闭并行处理"""
        if self.enable_parallel:
            print("正在关闭并行处理线程...")
            self.running = False
            
            # 发送停止信号
            try:
                self.depth_queue.put(None, timeout=1.0)
                self.pointcloud_queue.put(None, timeout=1.0)
                self.map_queue.put(None, timeout=1.0)
            except:
                pass
            
            # 等待线程结束
            for thread in self.threads:
                thread.join(timeout=2.0)
            
            print("✓ 并行处理线程已关闭")
    
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
    
    def __del__(self):
        """析构函数"""
        self.shutdown()
