#!/usr/bin/env python3
"""
基于 ZED 相机的实时深度估计演示 + Open3D 点云可视化
"""

import sys
import os

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
sys.path.insert(0, f'{dir_path}/Depth-Anything-V2-list3')
sys.path.append(f'{dir_path}/core')

import argparse
import numpy as np
import torch
from pathlib import Path
from core.monster import Monster
from core.utils.utils import InputPadder
import cv2
import time
import threading
import queue

# 导入 pyzed
import pyzed.sl as sl

# 导入 Open3D
try:
    import open3d as o3d
except ImportError:
    print("请安装 Open3D: pip install open3d")
    sys.exit(1)


DEVICE = 'cuda'


def numpy_to_tensor(img_np):
    """将 numpy 图像转换为 tensor"""
    img = torch.from_numpy(img_np).permute(2, 0, 1).float()
    return img[None].to(DEVICE)


def init_zed_camera(resolution=sl.RESOLUTION.VGA, fps=30):
    """初始化 ZED 相机"""
    zed = sl.Camera()
    
    init_params = sl.InitParameters()
    init_params.camera_resolution = resolution
    init_params.camera_fps = fps
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.coordinate_units = sl.UNIT.METER
    
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"相机打开失败: {err}")
        return None
    
    print("ZED 相机已成功打开")
    print(f"分辨率: {zed.get_camera_information().camera_configuration.resolution.width}x"
          f"{zed.get_camera_information().camera_configuration.resolution.height}")
    print(f"帧率: {zed.get_camera_information().camera_configuration.fps}")
    
    return zed


def disparity_to_depth(disparity, baseline, focal_length):
    """
    将视差转换为深度
    
    Args:
        disparity: 视差图 (像素)
        baseline: 基线距离 (米)
        focal_length: 焦距 (像素)
    
    Returns:
        depth: 深度图 (米)
    """
    # 避免除零
    disparity = np.maximum(disparity, 0.1)
    depth = (baseline * focal_length) / disparity
    return depth


class HighQualityPointCloudGenerator:
    def __init__(self, fx, fy, cx, cy, downsample_factor=2):
        """初始化高质量点云生成器
        
        参数:
            fx, fy, cx, cy: 相机内参
            downsample_factor: 下采样因子，减少点数，提高性能
        """
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.downsample_factor = downsample_factor
        self.edge_threshold = 0.05  # 边缘梯度阈值
        self.min_depth = 0.1        # 最小有效深度
        self.max_depth = 20.0       # 最大有效深度
        
    def generate_high_quality_pointcloud(self, depth_map, rgb_image=None, use_advanced_filtering=True):
        """生成高质量点云
        
        参数:
            depth_map: 原始深度图
            rgb_image: RGB图像 (RGB格式)
            use_advanced_filtering: 是否使用高级滤波
        
        返回:
            points: 3D点坐标
            colors: 颜色信息
        """
        if use_advanced_filtering:
            return self._generate_high_quality_advanced(depth_map, rgb_image)
        else:
            return self._generate_high_quality_fast(depth_map, rgb_image)
    
    def _generate_high_quality_fast(self, depth_map, rgb_image=None):
        """快速版本：适合实时处理"""
        # 1. 深度图预处理
        depth_clean = self._preprocess_depth_map_fast(depth_map)
        
        # 2. 生成有效点掩码
        valid_mask = self._create_valid_mask_fast(depth_clean)
        
        if not np.any(valid_mask):
            return None, None
        
        # 3. 生成点云
        points, colors = self._depth_to_pointcloud_fast(depth_clean, rgb_image, valid_mask)
        
        return points, colors
    
    def _generate_high_quality_advanced(self, depth_map, rgb_image=None):
        """高级版本：质量更高，但计算量更大"""
        # 1. 深度图预处理
        depth_clean = self._preprocess_depth_advanced(depth_map, rgb_image)
        
        # 2. 置信度过滤
        depth_filtered, confidence_mask = self._confidence_filter(depth_clean)
        
        # 3. 生成点云
        points, colors = self._depth_to_pointcloud_advanced(depth_filtered, rgb_image, confidence_mask)
        
        # 4. 点云后处理
        if points is not None and len(points) > 100:
            points, colors = self._postprocess_pointcloud(points, colors)
        
        return points, colors
    
    def _preprocess_depth_map_fast(self, depth_map, kernel_size=5, threshold=0.1):
        """
        深度图预处理，减少边缘噪声
        """
        # 1. 中值滤波（去除椒盐噪声）
        depth_filtered = cv2.medianBlur(depth_map.astype(np.float32), kernel_size)
        
        # 2. 双边滤波（保留边缘）
        depth_filtered = cv2.bilateralFilter(depth_filtered, d=5, sigmaColor=0.1, sigmaSpace=5)
        
        # 3. 形态学操作（填充小孔洞）
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        depth_filtered = cv2.morphologyEx(depth_filtered, cv2.MORPH_CLOSE, kernel)
        
        # 4. 深度一致性检查
        depth_gradient = np.abs(cv2.Sobel(depth_filtered, cv2.CV_64F, 1, 1, ksize=3))
        
        # 5. 梯度阈值（过滤边缘高梯度点）
        edge_mask = depth_gradient < threshold
        depth_filtered[~edge_mask] = 0  # 将高梯度点设为无效
        
        return depth_filtered
    
    def _preprocess_depth_advanced(self, depth_map, rgb_image=None):
        """高级深度图预处理"""
        # 确保深度图是float32类型
        if depth_map.dtype != np.float32:
            depth_float = depth_map.astype(np.float32)
        else:
            depth_float = depth_map.copy()
        
        # 1. 中值滤波
        depth_median = cv2.medianBlur(depth_float, 5)
        
        # 2. 高斯滤波
        depth_gaussian = cv2.GaussianBlur(depth_median, (5, 5), 1.0)
        
        # 3. 双边滤波
        depth_bilateral = cv2.bilateralFilter(depth_gaussian, 7, 50, 50)
        
        return depth_bilateral
    
    def _create_valid_mask_fast(self, depth_map):
        """创建有效点掩码（快速版）"""
        # 基本深度范围过滤
        valid_mask = (depth_map > self.min_depth) & (depth_map < self.max_depth)
        
        # 计算深度变化
        if np.any(valid_mask):
            # 计算局部深度方差
            kernel = np.ones((3, 3), np.float32) / 9
            depth_mean = cv2.filter2D(depth_map, -1, kernel)
            depth_diff = np.abs(depth_map - depth_mean)
            
            # 过滤深度变化太大的点
            depth_variance_mask = depth_diff < 0.5
            valid_mask = valid_mask & depth_variance_mask
        
        return valid_mask
    
    def _confidence_filter(self, depth_map):
        """置信度过滤"""
        # 确保深度图是float32类型
        if depth_map.dtype != np.float32:
            depth_float = depth_map.astype(np.float32)
        else:
            depth_float = depth_map.copy()
        
        # 计算深度梯度
        grad_x = cv2.Sobel(depth_float, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_float, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        
        # 计算拉普拉斯响应
        laplacian = cv2.Laplacian(depth_float, cv2.CV_32F, ksize=3)
        
        # 避免除零错误
        eps = 1e-7
        
        # 计算置信度
        confidence = np.exp(-grad_mag / 0.05) * np.exp(-np.abs(laplacian) / (0.1 + eps))
        
        # 阈值化
        confidence_mask = confidence > 0.04
        
        # 应用掩码
        depth_filtered = depth_float.copy()
        depth_filtered[~confidence_mask] = 0
        
        return depth_filtered, confidence_mask
    
    def _depth_to_pointcloud_fast(self, depth_map, rgb_image, valid_mask):
        """深度图转点云（快速版）"""
        h, w = depth_map.shape
        
        # 下采样
        ds_h = h // self.downsample_factor
        ds_w = w // self.downsample_factor
        
        # 生成下采样后的坐标网格
        y, x = np.meshgrid(
            np.arange(0, h, self.downsample_factor),
            np.arange(0, w, self.downsample_factor),
            indexing='ij'
        )
        
        # 获取下采样后的有效点
        valid_ds = valid_mask[y, x]
        if not np.any(valid_ds):
            return None, None
        
        # 计算3D坐标
        Z = depth_map[y[valid_ds], x[valid_ds]]
        X = (x[valid_ds] - self.cx) * Z / self.fx
        Y = (y[valid_ds] - self.cy) * Z / self.fy
        
        points = np.stack([X, Y, Z], axis=-1)
        
        # 获取颜色
        if rgb_image is not None:
            if rgb_image.shape[:2] != (h, w):
                rgb_image = cv2.resize(rgb_image, (w, h))
            colors = rgb_image[y[valid_ds], x[valid_ds]]
            # RGB格式，直接使用
            colors = colors.astype(np.float32) / 255.0
        else:
            colors = np.ones((len(points), 3), dtype=np.float32) * 0.5
        
        return points, colors
    
    def _depth_to_pointcloud_advanced(self, depth_map, rgb_image, valid_mask):
        """深度图转点云（高级版）"""
        h, w = depth_map.shape
        
        # 生成坐标网格
        y, x = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        
        # 应用有效掩码
        x_valid = x[valid_mask]
        y_valid = y[valid_mask]
        z_valid = depth_map[valid_mask]
        
        if len(z_valid) == 0:
            return None, None
        
        # 反投影
        X = (x_valid - self.cx) * z_valid / self.fx
        Y = (y_valid - self.cy) * z_valid / self.fy
        Z = z_valid
        
        points = np.stack([X, Y, Z], axis=-1)
        
        # 获取颜色
        if rgb_image is not None:
            colors = rgb_image[y_valid, x_valid]
            # RGB格式，直接使用
            colors = colors.astype(np.float32) / 255.0
        else:
            colors = np.ones((len(points), 3), dtype=np.float32) * 0.5
        
        return points, colors
    
    def _postprocess_pointcloud(self, points, colors):
        """点云后处理"""
        if len(points) < 100:
            return points, colors
        
        try:
            # 1. 体素下采样
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # 体素下采样
            pcd = pcd.voxel_down_sample(voxel_size=0.01)
            
            # 2. 统计离群点去除
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
            pcd = pcd.select_by_index(ind)
            
            # 3. 半径离群点去除
            cl, ind = pcd.remove_radius_outlier(nb_points=10, radius=0.05)
            pcd = pcd.select_by_index(ind)
            
            return np.asarray(pcd.points), np.asarray(pcd.colors)
        except Exception as e:
            print(f"点云后处理错误: {e}")
            return points, colors


def create_point_cloud(depth, rgb_image, fx, fy, cx, cy, max_depth=1.0):
    """
    从深度图和 RGB 图像创建点云（保留用于兼容性）
    
    Args:
        depth: 深度图 (H, W)
        rgb_image: RGB 图像 (H, W, 3)
        fx, fy: 焦距
        cx, cy: 主点
        max_depth: 最大深度阈值
    
    Returns:
        points: 点云坐标 (N, 3)
        colors: 点云颜色 (N, 3)
    """
    h, w = depth.shape
    
    # 创建像素坐标网格
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    
    # 有效深度掩码
    valid_mask = (depth > 0.1) & (depth < max_depth)
    
    # 计算 3D 坐标
    z = depth[valid_mask]
    x = (u[valid_mask] - cx) * z / fx
    y = (v[valid_mask] - cy) * z / fy
    
    # 组合点云坐标
    points = np.stack([x, y, z], axis=-1)
    
    # 获取颜色
    colors = rgb_image[valid_mask] / 255.0
    
    return points, colors


def filter_point_cloud(pcd, filter_config):
    """
    对点云进行滤波处理，去除噪声和伪影（优化版本）
    
    Args:
        pcd: Open3D点云对象
        filter_config: 滤波配置字典
    
    Returns:
        filtered_pcd: 滤波后的点云
    """
    if len(pcd.points) == 0:
        return pcd
    
    original_count = len(pcd.points)
    
    # 0. 深度范围过滤 (Depth Range Filtering) - 最快的过滤方式
    if filter_config.get('use_depth_filter', False):
        min_depth = filter_config.get('min_depth', 0.1)
        max_depth = filter_config.get('max_depth', 10.0)
        
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        # 计算每个点的深度（Z坐标）
        depths = points[:, 2]
        
        # 创建深度范围掩码
        depth_mask = (depths >= min_depth) & (depths <= max_depth)
        
        # 过滤点云
        filtered_points = points[depth_mask]
        filtered_colors = colors[depth_mask]
        
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
        
        if filter_config.get('verbose', False):
            removed = original_count - len(filtered_points)
            print(f"深度过滤 [{min_depth:.2f}m - {max_depth:.2f}m]: "
                  f"{original_count} -> {len(filtered_points)} 个点 "
                  f"(移除 {removed} 个点)")
    
    # 1. 体素下采样 (Voxel Downsampling) - 优先执行，大幅减少点数
    if filter_config.get('use_voxel', True):
        voxel_size = filter_config.get('voxel_size', 0.01)
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # 2. 统计离群点移除 (Statistical Outlier Removal) - 可选，较慢
    if filter_config.get('use_statistical', False):
        nb_neighbors = filter_config.get('statistical_nb_neighbors', 10)  # 减少邻居数
        std_ratio = filter_config.get('statistical_std_ratio', 2.0)
        pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    
    # 3. 半径离群点移除 (Radius Outlier Removal) - 可选，较慢
    if filter_config.get('use_radius', False):
        nb_points = filter_config.get('radius_nb_points', 10)  # 减少邻居数
        radius = filter_config.get('radius', 0.05)
        pcd, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    
    filtered_count = len(pcd.points)
    
    if filter_config.get('verbose', False):
        print(f"总滤波结果: {original_count} -> {filtered_count} 个点 "
              f"(移除 {original_count - filtered_count} 个点, {(1 - filtered_count/original_count)*100:.1f}%)")
    
    return pcd


class PointCloudVisualizer:
    """Open3D 点云可视化器（在主线程中运行，滤波在后台线程）"""
    
    def __init__(self, filter_config=None):
        self.vis = None
        self.pcd = None
        self.point_cloud_queue = queue.Queue(maxsize=2)
        self.filtered_queue = queue.Queue(maxsize=1)  # 滤波后的点云队列
        self.initialized = False
        self.first_update = True
        self.filter_config = filter_config or {}
        self.filter_thread = None
        self.filter_running = False
        
        # 启动滤波线程
        if self.filter_config.get('enable_filter', False):
            self.filter_running = True
            self.filter_thread = threading.Thread(target=self._filter_worker, daemon=True)
            self.filter_thread.start()
            print("点云滤波后台线程已启动")
    
    def _filter_worker(self):
        """后台滤波工作线程"""
        while self.filter_running:
            try:
                # 从队列获取原始点云数据
                points, colors = self.point_cloud_queue.get(timeout=0.1)
                
                # 创建点云对象
                temp_pcd = o3d.geometry.PointCloud()
                temp_pcd.points = o3d.utility.Vector3dVector(points)
                temp_pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # 应用滤波
                filtered_pcd = filter_point_cloud(temp_pcd, self.filter_config)
                
                # 将滤波后的点云放入输出队列
                if not self.filtered_queue.full():
                    try:
                        self.filtered_queue.put_nowait((
                            np.asarray(filtered_pcd.points),
                            np.asarray(filtered_pcd.colors)
                        ))
                    except queue.Full:
                        pass
                        
            except queue.Empty:
                continue
            except Exception as e:
                print(f"滤波线程错误: {e}")
        
    def initialize(self):
        """初始化可视化窗口（必须在主线程中调用）"""
        if self.initialized:
            return
            
        # 创建可视化窗口
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="RT-MonSter++ Point Cloud", width=1280, height=720)
        
        # 创建初始点云
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        
        # 设置渲染选项
        render_option = self.vis.get_render_option()
        render_option.point_size = 2.0
        render_option.background_color = np.array([0.1, 0.1, 0.1])
        
        self.initialized = True
        print("Open3D 可视化窗口已创建")
        if self.filter_config.get('enable_filter', True):
            print("点云滤波已启用")
        
    def update(self, points, colors):
        """更新点云数据（从主线程调用）"""
        if not self.point_cloud_queue.full():
            try:
                self.point_cloud_queue.put_nowait((points, colors))
            except queue.Full:
                pass
                
    def poll_events(self):
        """轮询事件并更新渲染（必须在主线程中调用）"""
        if not self.initialized:
            return True
            
        try:
            # 尝试获取滤波后的点云数据（如果启用滤波）
            if self.filter_config.get('enable_filter', False):
                try:
                    points, colors = self.filtered_queue.get_nowait()
                    
                    if len(points) > 0:
                        # 直接更新点云（已经滤波过）
                        self.pcd.points = o3d.utility.Vector3dVector(points)
                        self.pcd.colors = o3d.utility.Vector3dVector(colors)
                        
                        # 更新几何体
                        self.vis.update_geometry(self.pcd)
                        
                        # 第一次更新时重置视角
                        if self.first_update:
                            self.vis.reset_view_point(True)
                            self.first_update = False
                            print(f"点云已更新: {len(self.pcd.points)} 个点")
                            
                except queue.Empty:
                    pass
            else:
                # 不启用滤波，直接使用原始点云
                try:
                    points, colors = self.point_cloud_queue.get_nowait()
                    
                    if len(points) > 0:
                        self.pcd.points = o3d.utility.Vector3dVector(points)
                        self.pcd.colors = o3d.utility.Vector3dVector(colors)
                        
                        # 更新几何体
                        self.vis.update_geometry(self.pcd)
                        
                        # 第一次更新时重置视角
                        if self.first_update:
                            self.vis.reset_view_point(True)
                            self.first_update = False
                            print(f"点云已更新: {len(self.pcd.points)} 个点")
                    else:
                        print("警告: 点云为空")
                        
                except queue.Empty:
                    pass
            
            # 更新渲染
            self.vis.poll_events()
            self.vis.update_renderer()
            
            # 检查窗口是否关闭
            return True
            
        except Exception as e:
            print(f"可视化错误: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def destroy(self):
        """销毁可视化窗口"""
        # 停止滤波线程
        if self.filter_thread:
            self.filter_running = False
            self.filter_thread.join(timeout=1.0)
            print("滤波线程已停止")
        
        if self.vis and self.initialized:
            self.vis.destroy_window()
            self.initialized = False

def preprocess_depth_map(depth_map, kernel_size=3, threshold=0.05):
    """
    深度图预处理，减少边缘噪声
    """
    import cv2
    import numpy as np
    
    # 1. 中值滤波（去除椒盐噪声）
    depth_filtered = cv2.medianBlur(depth_map.astype(np.float32), kernel_size)
    
    # 2. 双边滤波（保留边缘）
    depth_filtered = cv2.bilateralFilter(depth_filtered, d=5, sigmaColor=0.1, sigmaSpace=5)
    
    # 3. 形态学操作（填充小孔洞）
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    depth_filtered = cv2.morphologyEx(depth_filtered, cv2.MORPH_CLOSE, kernel)
    
    # 4. 深度一致性检查
    depth_gradient = np.abs(cv2.Sobel(depth_filtered, cv2.CV_64F, 1, 1, ksize=3))
    
    # 5. 梯度阈值（过滤边缘高梯度点）
    edge_mask = depth_gradient < threshold
    depth_filtered[~edge_mask] = 0  # 将高梯度点设为无效
    
    return depth_filtered

def demo_realtime(args):
    """实时深度估计演示"""
    # 加载 MonSter 模型
    print("正在加载 MonSter 模型...")
    model = torch.nn.DataParallel(Monster(args), device_ids=[0])

    assert args.restore_ckpt and Path(args.restore_ckpt).exists(), f"模型文件不存在: {args.restore_ckpt}"
    checkpoint = torch.load(args.restore_ckpt)
    ckpt = dict()
    if 'state_dict' in checkpoint.keys():
        checkpoint = checkpoint['state_dict']
    for key in checkpoint:
        ckpt['module.' + key] = checkpoint[key]

    model.load_state_dict(ckpt, strict=True)

    model = model.module
    model.to(DEVICE)
    model.eval()
    print("模型加载完成")

    # 初始化 ZED 相机
    print("正在初始化 ZED 相机...")
    zed = init_zed_camera(resolution=sl.RESOLUTION.VGA, fps=30)
    if zed is None:
        return

    # 获取相机参数
    cam_info = zed.get_camera_information()
    calibration = cam_info.camera_configuration.calibration_parameters.left_cam
    fx = calibration.fx
    fy = calibration.fy
    cx = calibration.cx
    cy = calibration.cy
    baseline = cam_info.camera_configuration.calibration_parameters.get_camera_baseline()
    
    left_cap = cv2.VideoCapture('/home/sunteng/Desktop/HighTorque_vision/video/left.mp4')
    right_cap = cv2.VideoCapture('/home/sunteng/Desktop/HighTorque_vision/video/right.mp4')

    print(f"\n相机参数:")
    print(f"  焦距 (fx, fy): ({fx:.2f}, {fy:.2f})")
    print(f"  主点 (cx, cy): ({cx:.2f}, {cy:.2f})")
    print(f"  基线: {baseline:.4f} m")

    # 创建图像对象
    left_image = sl.Mat()
    right_image = sl.Mat()
    
    # 运行时参数
    runtime_params = sl.RuntimeParameters()
    
    # 创建输出目录
    output_directory = Path(args.output_directory)
    output_directory.mkdir(exist_ok=True)
    
    # 初始化高质量点云生成器
    pointcloud_generator = None
    if args.enable_pointcloud:
        print("\n正在初始化高质量点云生成器...")
        pointcloud_generator = HighQualityPointCloudGenerator(
            fx=fx, fy=fy, cx=cx, cy=cy,
            downsample_factor=args.downsample_factor
        )
        pointcloud_generator.min_depth = args.min_depth
        pointcloud_generator.max_depth = args.max_depth
        print(f"下采样因子: {args.downsample_factor}")
        print(f"深度范围: {args.min_depth}m - {args.max_depth}m")
        print(f"高级滤波: {'启用' if args.use_advanced_filtering else '禁用'}")
    
    # 初始化 Open3D 可视化器
    visualizer = None
    if args.enable_pointcloud:
        print("\n正在初始化 Open3D 点云可视化...")
        
        # 配置点云滤波参数（用于Open3D后处理）
        filter_config = {
            'enable_filter': args.enable_filter,
            'use_depth_filter': False,  # 深度过滤已在生成器中完成
            'min_depth': args.min_depth,
            'max_depth': args.filter_max_depth,
            'use_statistical': args.use_statistical,
            'statistical_nb_neighbors': args.statistical_nb_neighbors,
            'statistical_std_ratio': args.statistical_std_ratio,
            'use_radius': args.use_radius,
            'radius_nb_points': args.radius_nb_points,
            'radius': args.radius,
            'use_voxel': args.use_voxel,
            'voxel_size': args.voxel_size,
            'verbose': args.filter_verbose
        }
        
        visualizer = PointCloudVisualizer(filter_config=filter_config)
        visualizer.initialize()  # 在主线程中初始化
    
    print("\n开始实时深度估计...")
    print("按 'q' 键退出")
    print("按 's' 键保存当前帧")
    
    frame_count = 0
    fps_list = []
    
    try:
        with torch.no_grad():
            while True:
                start_time = time.time()
                
                # 抓取新帧
                if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                    # 获取左右图像
                    zed.retrieve_image(left_image, sl.VIEW.LEFT)
                    zed.retrieve_image(right_image, sl.VIEW.RIGHT)

                    left_video_ret, left_frame = left_cap.read()
                    right_video_ret, right_frame = right_cap.read()
                    
                    # 转换为 numpy 数组
                    left_img_np = left_image.get_data()[:, :, :3]
                    right_img_np = right_image.get_data()[:, :, :3]
                    
                    # 转换为 RGB
                    left_img_rgb = cv2.cvtColor(left_img_np, cv2.COLOR_BGRA2RGB)
                    right_img_rgb = cv2.cvtColor(right_img_np, cv2.COLOR_BGRA2RGB)
                    
                    # 转换为 tensor
                    image1 = numpy_to_tensor(left_img_rgb)
                    image2 = numpy_to_tensor(right_img_rgb)
                    
                    # Padding
                    padder = InputPadder(image1.shape, divis_by=32)
                    image1, image2 = padder.pad(image1, image2)
                    
                    # 深度估计
                    inference_start = time.time()
                    disp = model(image1, image2, iters=args.valid_iters, test_mode=True)
                    inference_time = time.time() - inference_start
                    
                    # Unpad
                    disp = padder.unpad(disp)
                    
                    # 转换为 numpy
                    disp_np = disp.cpu().numpy().squeeze()
                    
                    # 将视差转换为深度
                    depth = disparity_to_depth(disp_np, baseline, fx)
                    # depth = preprocess_depth_map(depth, kernel_size=5, threshold=0.1)
                    
                    # 创建点云（使用高质量生成器）
                    if args.enable_pointcloud and frame_count % args.pointcloud_skip == 0:
                        points, colors = pointcloud_generator.generate_high_quality_pointcloud(
                            depth, left_img_rgb,
                            use_advanced_filtering=args.use_advanced_filtering
                        )
                        
                        if points is not None and len(points) > 0:
                            if frame_count == 0:
                                print(f"第一帧点云: {len(points)} 个点, 深度范围: {depth.min():.2f} - {depth.max():.2f} m")
                            visualizer.update(points, colors)
                        else:
                            if frame_count == 0:
                                print("警告: 第一帧点云生成失败")
                    
                    # 转换为彩色图用于显示
                    disp_color = cv2.applyColorMap(
                        np.clip(disp_np * 2.0, 0, 255).astype(np.uint8), 
                        cv2.COLORMAP_PLASMA
                    )
                    
                    # 转换左图为 BGR 用于显示
                    left_img_bgr = cv2.cvtColor(left_img_rgb, cv2.COLOR_RGB2BGR)
                    
                    # 左右拼接：原图 + 深度图
                    display_img = np.hstack((left_img_bgr, disp_color))
                    
                    # 计算 FPS
                    total_time = time.time() - start_time
                    fps = 1.0 / total_time
                    fps_list.append(fps)
                    if len(fps_list) > 30:
                        fps_list.pop(0)
                    avg_fps = np.mean(fps_list)
                    
                    # 添加文字信息
                    cv2.putText(display_img, f"Frame: {frame_count}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(display_img, f"FPS: {avg_fps:.1f}", (10, 55), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(display_img, f"Inference: {inference_time*1000:.1f}ms", (10, 80), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    if args.enable_pointcloud:
                        cv2.putText(display_img, "Point Cloud: ON", (10, 105), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # 显示
                    cv2.imshow("ZED + RT-MonSter++ Depth Estimation", display_img)
                    
                    frame_count += 1
                    
                    # 更新 Open3D 可视化（在主线程中）
                    if args.enable_pointcloud:
                        visualizer.poll_events()
                    
                    # 键盘控制
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("\n退出程序...")
                        break
                    elif key == ord('s'):
                        # 保存当前帧
                        save_path = output_directory / f"frame_{frame_count:06d}.png"
                        cv2.imwrite(str(save_path), display_img)
                        
                        # 保存点云
                        if args.enable_pointcloud:
                            points, colors = pointcloud_generator.generate_high_quality_pointcloud(
                                depth, left_img_rgb,
                                use_advanced_filtering=args.use_advanced_filtering
                            )
                            if points is not None and len(points) > 0:
                                pcd = o3d.geometry.PointCloud()
                                pcd.points = o3d.utility.Vector3dVector(points)
                                pcd.colors = o3d.utility.Vector3dVector(colors)
                                pcd_path = output_directory / f"pointcloud_{frame_count:06d}.ply"
                                o3d.io.write_point_cloud(str(pcd_path), pcd)
                                print(f"已保存帧和点云到 {save_path} 和 {pcd_path}")
                            else:
                                print(f"已保存帧 {frame_count} 到 {save_path} (点云生成失败)")
                        else:
                            print(f"已保存帧 {frame_count} 到 {save_path}")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    
    finally:
        # 清理资源
        print("\n正在清理资源...")
        if visualizer:
            visualizer.destroy()
        zed.close()
        cv2.destroyAllWindows()
        print("相机已关闭")
        print(f"总共处理了 {frame_count} 帧")
        if fps_list:
            print(f"平均 FPS: {np.mean(fps_list):.2f}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ZED 相机实时深度估计 + 点云可视化')
    
    # 模型参数
    parser.add_argument('--restore_ckpt', help="模型权重路径", 
                       default="/home/sunteng/Desktop/HighTorque_vision/MonSter-plusplus/RT-MonSter++/checkpoints/Zero_shot.pth")
    parser.add_argument('--output_directory', help="输出目录", 
                       default="zed_output")
    parser.add_argument('--valid_iters', type=int, default=2, 
                       help='前向传播时的迭代次数')
    parser.add_argument('--encoder', type=str, default='vits', 
                       choices=['vits', 'vitb', 'vitl', 'vitg'])
    
    # 点云参数
    parser.add_argument('--enable_pointcloud', action='store_true',
                       help='启用 Open3D 点云可视化')
    parser.add_argument('--pointcloud_skip', type=int, default=1,
                       help='点云更新间隔（帧数）')
    parser.add_argument('--max_depth', type=float, default=10.0,
                       help='最大深度阈值（米）')
    parser.add_argument('--downsample_factor', type=int, default=2,
                       help='点云下采样因子（1=无下采样，2=1/4点数，3=1/9点数）')
    parser.add_argument('--use_advanced_filtering', action='store_true',
                       help='使用高级深度图滤波（质量更高但更慢）')
    
    # 点云滤波参数
    parser.add_argument('--enable_filter', action='store_true',
                       help='启用点云滤波（去除噪声和伪影）')
    parser.add_argument('--use_depth_filter', action='store_true',
                       help='启用深度范围过滤')
    parser.add_argument('--min_depth', type=float, default=0.2,
                       help='最小深度阈值（米）')
    parser.add_argument('--filter_max_depth', type=float, default=0.2,
                       help='滤波最大深度阈值（米）')
    parser.add_argument('--use_statistical', action='store_true',
                       help='使用统计离群点移除（较慢，默认关闭）')
    parser.add_argument('--statistical_nb_neighbors', type=int, default=10,
                       help='统计滤波：邻居点数量（减少以提高速度）')
    parser.add_argument('--statistical_std_ratio', type=float, default=2.0,
                       help='统计滤波：标准差倍数')
    parser.add_argument('--use_radius', action='store_true',
                       help='使用半径离群点移除（较慢，默认关闭）')
    parser.add_argument('--radius_nb_points', type=int, default=10,
                       help='半径滤波：最小邻居点数（减少以提高速度）')
    parser.add_argument('--radius', type=float, default=0.05,
                       help='半径滤波：搜索半径（米）')
    parser.add_argument('--use_voxel', action='store_true', default=True,
                       help='使用体素下采样（快速，默认开启）')
    parser.add_argument('--voxel_size', type=float, default=0.02,
                       help='体素下采样：体素大小（米，增大以提高速度）')
    parser.add_argument('--filter_verbose', action='store_true',
                       help='显示滤波详细信息')
    
    # 架构参数
    parser.add_argument('--hidden_dims', nargs='+', type=int, default=[32, 64, 96], 
                       help="隐藏状态和上下文维度")
    parser.add_argument('--corr_implementation', 
                       choices=["reg", "alt", "reg_cuda", "alt_cuda"], 
                       default="reg", help="相关体实现方式")
    parser.add_argument('--shared_backbone', action='store_true', 
                       help="使用单一骨干网络")
    parser.add_argument('--corr_levels', type=int, default=2, 
                       help="相关金字塔层数")
    parser.add_argument('--corr_radius', nargs='+', type=int, default=[2, 2, 4], 
                        help="相关金字塔宽度")
    parser.add_argument('--n_downsample', type=int, default=2, 
                       help="视差场分辨率 (1/2^K)")
    parser.add_argument('--slow_fast_gru', action='store_true', 
                       help="更频繁地迭代低分辨率 GRU")
    parser.add_argument('--n_gru_layers', type=int, default=3, 
                       help="隐藏 GRU 层数")
    parser.add_argument('--max_disp', type=int, default=416, 
                       help="几何编码体的最大视差")
    parser.add_argument('--mixed_precision', action='store_true', 
                       help='使用混合精度')
    
    args = parser.parse_args()
    
    demo_realtime(args)
