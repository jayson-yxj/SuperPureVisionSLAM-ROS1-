"""
点云生成器抽象基类

定义统一的点云生成和处理接口
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple, Optional, Dict, Any


class BasePointCloudGenerator(ABC):
    """点云生成器抽象基类"""
    
    @abstractmethod
    def generate(self, 
                 depth: np.ndarray,
                 rgb: np.ndarray,
                 camera_params: Dict[str, float],
                 pose: np.ndarray) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        生成点云
        
        Args:
            depth: 深度图 (H, W), float32, 单位：米
            rgb: RGB图像 (H, W, 3), uint8
            camera_params: 相机内参
                {
                    'fx': float,  # 焦距x
                    'fy': float,  # 焦距y
                    'cx': float,  # 主点x
                    'cy': float   # 主点y
                }
            pose: 位姿矩阵 (4, 4), float32
                世界坐标系到相机坐标系的变换
            
        Returns:
            points: 点云坐标 (N, 3), float32
            colors: 点云颜色 (N, 3), float32, range [0, 1]
        """
        pass
    
    @abstractmethod
    def filter(self, 
               points: np.ndarray,
               colors: Optional[np.ndarray],
               filter_params: Dict[str, Any]) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        点云过滤
        
        Args:
            points: 输入点云 (N, 3)
            colors: 输入颜色 (N, 3)
            filter_params: 过滤参数
                {
                    'depth_range': Tuple[float, float],      # 深度范围
                    'height_range': Tuple[float, float],     # 高度范围
                    'crop_params': Dict,                     # 裁剪参数
                    'statistical_outlier': Dict[str, Any]    # 统计离群点过滤
                }
            
        Returns:
            filtered_points: 过滤后的点云 (M, 3), M <= N
            filtered_colors: 过滤后的颜色 (M, 3)
        """
        pass
    
    @abstractmethod
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
        pass
    
    def get_statistics(self, points: np.ndarray) -> Dict[str, Any]:
        """
        获取点云统计信息（可选实现）
        
        Args:
            points: 点云坐标 (N, 3)
            
        Returns:
            stats: 统计信息字典
        """
        if len(points) == 0:
            return {
                'num_points': 0,
                'bounds': None
            }
        
        return {
            'num_points': len(points),
            'bounds': {
                'x_min': float(points[:, 0].min()),
                'x_max': float(points[:, 0].max()),
                'y_min': float(points[:, 1].min()),
                'y_max': float(points[:, 1].max()),
                'z_min': float(points[:, 2].min()),
                'z_max': float(points[:, 2].max())
            }
        }
