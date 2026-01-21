"""
Depth Anything V2 深度估计器实现
"""

import os
import sys
import torch
import cv2
import numpy as np
from typing import Dict, Any

# 添加depth_anything_v2模块路径
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from depth_anything_v2.dpt import DepthAnythingV2
from .base_depth_estimator import BaseDepthEstimator


class DepthAnythingV2Estimator(BaseDepthEstimator):
    """Depth Anything V2 深度估计器"""
    
    def __init__(self):
        """初始化"""
        self.model = None
        self.input_size = None
        self.max_depth = None
        self.device = None
        self.encoder = None
        self.enable_preprocessing = False
        self.preprocessing_params = {}
        
    def initialize(self, config: Dict[str, Any]) -> None:
        """
        初始化Depth Anything V2模型
        
        Args:
            config: 配置字典
                {
                    'model_path': str,      # 模型权重路径
                    'encoder': str,         # 编码器类型 ('vits', 'vitb', 'vitl', 'vitg')
                    'input_size': int,      # 输入尺寸 (256, 384, 512, 640)
                    'max_depth': float,     # 最大深度值（米）
                    'device': str           # 计算设备 ('cuda', 'cpu', 'mps')
                }
        """
        self.input_size = config.get('input_size', 256)
        self.max_depth = config.get('max_depth', 70.0)
        self.device = config.get('device', 'cuda')
        self.encoder = config.get('encoder', 'vitb')
        
        # 模型配置
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }
        
        if self.encoder not in model_configs:
            raise ValueError(f"不支持的编码器类型: {self.encoder}. 支持的类型: {list(model_configs.keys())}")
        
        # 创建模型
        self.model = DepthAnythingV2(
            **model_configs[self.encoder],
            max_depth=self.max_depth
        )
        
        # 加载权重
        model_path = config.get('model_path')
        if not model_path or not os.path.exists(model_path):
            raise FileNotFoundError(f"模型文件不存在: {model_path}")
        
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model = self.model.to(self.device).eval()
        
        # 深度图预处理配置
        preprocessing_config = config.get('depth_preprocessing', {})
        self.enable_preprocessing = preprocessing_config.get('enabled', False)
        if self.enable_preprocessing:
            self.preprocessing_params = {
                'kernel_size': preprocessing_config.get('kernel_size', 5),
                'threshold': preprocessing_config.get('threshold', 0.07)
            }
        
        print(f"✓ Depth Anything V2 模型已加载: {self.encoder} @ {self.device}")
        print(f"  - 输入尺寸: {self.input_size}")
        print(f"  - 最大深度: {self.max_depth}m")
        print(f"  - 深度预处理: {'启用' if self.enable_preprocessing else '禁用'}")
        
    def estimate(self, image: np.ndarray) -> np.ndarray:
        """
        估计深度图
        
        Args:
            image: RGB图像 (H, W, 3), uint8, [0, 255]
            
        Returns:
            depth: 深度图 (H, W), float32, 单位：米
        """
        if self.model is None:
            raise RuntimeError("模型未初始化，请先调用 initialize()")
        
        with torch.no_grad():
            depth = self.model.infer_image(image, self.input_size)
        
        return depth
    
    def get_info(self) -> Dict[str, Any]:
        """获取模型信息"""
        return {
            'name': 'Depth Anything V2',
            'version': '2.0',
            'encoder': self.encoder,
            'input_size': self.input_size,
            'max_depth': self.max_depth,
            'device': self.device
        }
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        预处理图像（Depth Anything V2 内部已处理，这里保持原样）
        
        Args:
            image: 原始图像
            
        Returns:
            image: 原始图像（不做处理）
        """
        return image
    
    def postprocess(self, depth: np.ndarray) -> np.ndarray:
        """
        后处理深度图（包括可选的降噪处理）
        
        Args:
            depth: 原始深度图
            
        Returns:
            depth: 处理后的深度图
        """
        # 深度图预处理（降噪）
        if self.enable_preprocessing:
            depth = self._preprocess_depth_map(
                depth,
                kernel_size=self.preprocessing_params['kernel_size'],
                threshold=self.preprocessing_params['threshold']
            )
        
        # 确保深度值在合理范围内
        depth = np.clip(depth, 0, self.max_depth)
        
        # 移除无效值
        depth[~np.isfinite(depth)] = 0
        
        return depth
    
    def _preprocess_depth_map(self, depth_map: np.ndarray, kernel_size: int = 5, threshold: float = 0.07) -> np.ndarray:
        """
        深度图预处理，减少边缘噪声
        
        Args:
            depth_map: 原始深度图
            kernel_size: 滤波核大小
            threshold: 梯度阈值
            
        Returns:
            depth_filtered: 处理后的深度图
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
