"""
地图构建器抽象基类

定义统一的地图构建和管理接口
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Tuple, Optional, Any


class BaseMapBuilder(ABC):
    """地图构建器抽象基类"""
    
    @abstractmethod
    def update(self, 
               points: np.ndarray, 
               colors: Optional[np.ndarray] = None) -> None:
        """
        更新地图
        
        Args:
            points: 新的点云 (N, 3)
            colors: 点云颜色 (N, 3)
        """
        pass
    
    @abstractmethod
    def get_occupancy_grid(self, 
                          resolution: float,
                          height_range: Tuple[float, float]) -> Dict[str, Any]:
        """
        获取2D占用栅格地图
        
        Args:
            resolution: 网格分辨率（米/格）
            height_range: 高度范围 (min, max) 或 (ratio_min, ratio_max)
            
        Returns:
            grid_map: 地图字典
                {
                    'data': np.ndarray (H, W), int8, [-1, 0, 100],
                    'resolution': float,
                    'origin': Tuple[float, float],
                    'width': int,
                    'height': int
                }
        """
        pass
    
    @abstractmethod
    def get_point_cloud(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        获取完整点云
        
        Returns:
            points: 点云坐标 (N, 3)
            colors: 点云颜色 (N, 3)
        """
        pass
    
    @abstractmethod
    def clear(self) -> None:
        """清空地图"""
        pass
    
    def save(self, filepath: str) -> None:
        """
        保存地图（可选实现）
        
        Args:
            filepath: 保存路径
        """
        raise NotImplementedError("save() 方法未实现")
    
    def load(self, filepath: str) -> None:
        """
        加载地图（可选实现）
        
        Args:
            filepath: 加载路径
        """
        raise NotImplementedError("load() 方法未实现")
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        获取地图统计信息（可选实现）
        
        Returns:
            stats: 统计信息字典
        """
        points, _ = self.get_point_cloud()
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
