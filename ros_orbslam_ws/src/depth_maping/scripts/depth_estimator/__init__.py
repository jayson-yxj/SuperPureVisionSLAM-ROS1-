"""
深度估计模块

提供统一的深度估计接口，支持多种深度估计模型
"""

from .base_depth_estimator import BaseDepthEstimator
from .depth_anything_v2_estimator import DepthAnythingV2Estimator

__all__ = ['BaseDepthEstimator', 'DepthAnythingV2Estimator']
