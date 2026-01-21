"""
深度估计器抽象基类

定义统一的深度估计接口，所有深度估计模型都需要实现这个接口
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Any


class BaseDepthEstimator(ABC):
    """深度估计器抽象基类"""
    
    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> None:
        """
        初始化模型
        
        Args:
            config: 配置字典，包含模型参数
                {
                    'model_path': str,          # 模型文件路径
                    'input_size': int,          # 输入图像尺寸
                    'max_depth': float,         # 最大深度值（米）
                    'device': str,              # 计算设备 ('cuda', 'cpu', 'mps')
                    ...                         # 其他模型特定参数
                }
        """
        pass
    
    @abstractmethod
    def estimate(self, image: np.ndarray) -> np.ndarray:
        """
        估计深度图
        
        Args:
            image: RGB图像
                - shape: (H, W, 3)
                - dtype: uint8
                - range: [0, 255]
            
        Returns:
            depth: 深度图
                - shape: (H, W)
                - dtype: float32
                - unit: 米
                - range: [0, max_depth]
        """
        pass
    
    @abstractmethod
    def get_info(self) -> Dict[str, Any]:
        """
        获取模型信息
        
        Returns:
            info: 模型信息字典
                {
                    'name': str,            # 模型名称
                    'version': str,         # 模型版本
                    'input_size': int,      # 输入尺寸
                    'max_depth': float,     # 最大深度
                    'device': str           # 计算设备
                }
        """
        pass
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        预处理图像（可选重写）
        
        Args:
            image: 原始图像
            
        Returns:
            preprocessed_image: 预处理后的图像
        """
        return image
    
    def postprocess(self, depth: np.ndarray) -> np.ndarray:
        """
        后处理深度图（可选重写）
        
        Args:
            depth: 原始深度图
            
        Returns:
            postprocessed_depth: 后处理后的深度图
        """
        return depth
    
    def __str__(self) -> str:
        """返回模型信息字符串"""
        info = self.get_info()
        return f"{info['name']} v{info['version']} (device: {info['device']})"
