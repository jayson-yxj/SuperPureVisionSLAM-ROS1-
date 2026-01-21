# Depth Mapping 功能包重构说明

## 📋 重构概述

本次重构基于 [`optimization_plan.md`](docs/optimization_plan.md) 和 [`refactoring_plan.md`](docs/refactoring_plan.md) 进行，采用模块化架构设计，提高代码可维护性和可扩展性。

## 🏗️ 新架构

### 目录结构

```
depth_maping/
├── scripts/
│   ├── depth_maping_node.py          # ROS节点（重构后）
│   ├── depth_maping_node_backup.py   # 原始节点备份
│   ├── pipeline_manager.py           # 流程管理器
│   │
│   ├── depth_estimator/              # 深度估计模块
│   │   ├── __init__.py
│   │   ├── base_depth_estimator.py   # 抽象基类
│   │   └── depth_anything_v2_estimator.py
│   │
│   ├── point_cloud/                  # 点云处理模块
│   │   ├── __init__.py
│   │   ├── base_point_cloud_generator.py
│   │   └── open3d_generator.py
│   │
│   ├── map_builder/                  # 地图构建模块
│   │   ├── __init__.py
│   │   ├── base_map_builder.py
│   │   └── occupancy_grid_builder.py
│   │
│   └── depth_anything_v2/            # 深度估计模型（保持不变）
│
├── config/
│   └── default_config.yaml           # 配置文件
│
├── docs/
│   ├── optimization_plan.md          # 优化计划
│   ├── refactoring_plan.md           # 重构计划
│   └── REFACTORING_README.md         # 本文档
│
└── launch/
    └── slam_mapping.launch
```

## 🎯 核心改进

### 1. 模块化设计

**深度估计模块** ([`depth_estimator/`](scripts/depth_estimator/))
- 抽象基类定义统一接口
- 支持多种深度估计模型
- 易于添加新模型（MiDaS, ZoeDepth等）

**点云生成模块** ([`point_cloud/`](scripts/point_cloud/))
- 统一的点云生成和处理接口
- 支持过滤、下采样等操作
- 可扩展到PCL等其他库

**地图构建模块** ([`map_builder/`](scripts/map_builder/))
- 支持滑动窗口和累积模式
- 2D占用栅格地图生成
- 可扩展到Octomap等

### 2. 配置文件驱动

所有参数通过 [`config/default_config.yaml`](config/default_config.yaml) 配置：

```yaml
# 深度估计配置
depth_estimator:
  type: depth_anything_v2
  model_path: /path/to/model.pth
  encoder: vitb
  input_size: 256
  max_depth: 70.0
  device: cuda

# 点云生成配置
point_cloud:
  crop_params:
    img_crop_size: 128
    max_depth: 35.0
  filter:
    depth_range: [0.1, 50.0]
    height_range: [-10.0, 10.0]
  voxel_size: 1.0

# 地图构建配置
map:
  type: occupancy_grid
  sliding_window:
    enabled: true
    size: 3
  resolution: 0.8
  height_range: [0.3, 0.7]
  use_ratio: true
  occupied_thresh: 3
```

### 3. Pipeline Manager

[`pipeline_manager.py`](scripts/pipeline_manager.py) 协调所有模块：

```python
# 初始化
pipeline = PipelineManager(config_path='config/default_config.yaml')

# 处理单帧
result = pipeline.process_frame(image, pose, camera_params)

# 获取2D地图
map_dict = pipeline.get_2d_map()

# 获取点云
points, colors = pipeline.get_point_cloud()
```

## 🚀 使用方法

### 基本使用

```bash
# 使用默认配置
roslaunch depth_maping slam_mapping.launch

# 使用自定义配置
roslaunch depth_maping slam_mapping.launch config_path:=/path/to/config.yaml
```

### 切换深度估计模型

只需修改配置文件：

```yaml
depth_estimator:
  type: midas  # 从 depth_anything_v2 改为 midas
  model_type: DPT_Large
  device: cuda
```

### 调整性能参数

```yaml
# 提高速度
depth_estimator:
  input_size: 192  # 降低分辨率

point_cloud:
  voxel_size: 1.5  # 增加下采样

ros:
  publish_rate:
    point_cloud: 2  # 降低发布频率
    map: 5
```

## 📊 性能监控

启用性能分析：

```yaml
profiling:
  enabled: true
  log_interval: 5
```

运行时会输出：

```
⏱️  性能: 深度=300ms, 点云=50ms, 总计=400ms (2.5 FPS)
```

节点关闭时会打印完整摘要：

```
性能分析摘要
============================================================
depth_estimation:
  平均: 300.00ms
  标准差: 20.00ms
  最小: 280.00ms
  最大: 350.00ms
  样本数: 100

平均FPS: 2.50
============================================================
```

## 🔧 扩展指南

### 添加新的深度估计模型

1. 创建新文件 `scripts/depth_estimator/new_model_estimator.py`
2. 继承 `BaseDepthEstimator`
3. 实现必需方法：

```python
from .base_depth_estimator import BaseDepthEstimator

class NewModelEstimator(BaseDepthEstimator):
    def initialize(self, config):
        # 初始化模型
        pass
    
    def estimate(self, image):
        # 深度估计
        pass
    
    def get_info(self):
        return {'name': 'NewModel', 'version': '1.0'}
```

4. 在 `pipeline_manager.py` 中注册
5. 在配置文件中使用：

```yaml
depth_estimator:
  type: new_model
  ...
```

### 添加新的地图类型

类似地，继承 `BaseMapBuilder` 并实现接口。

## ⚠️ 注意事项

### 兼容性

- ✅ 保留所有原有功能
- ✅ ROS接口保持不变
- ✅ 支持重力对齐
- ✅ 支持滑动窗口
- ✅ 支持高度过滤

### 迁移

如果遇到问题，可以回退到原始版本：

```bash
cd scripts
cp depth_maping_node_backup.py depth_maping_node.py
```

## 📈 性能对比

| 指标 | 重构前 | 重构后 | 改进 |
|------|--------|--------|------|
| 代码行数 | 952 | ~600 (主节点) | ↓37% |
| 模块耦合度 | 高 | 低 | ✓ |
| 可维护性 | 中 | 高 | ✓ |
| 可扩展性 | 低 | 高 | ✓ |
| 运行性能 | 基准 | 相同 | = |

## 🐛 故障排除

### 导入错误

确保所有模块的 `__init__.py` 文件存在：

```bash
ls scripts/depth_estimator/__init__.py
ls scripts/point_cloud/__init__.py
ls scripts/map_builder/__init__.py
```

### 配置文件未找到

检查配置文件路径：

```bash
ls config/default_config.yaml
```

或在launch文件中指定：

```xml
<param name="config_path" value="$(find depth_maping)/config/default_config.yaml"/>
```

### 模型加载失败

检查模型路径是否正确：

```yaml
depth_estimator:
  model_path: /home/sunteng/Downloads/depth_anything_v2_metric_hypersim_vitb.pth
```

## 📚 相关文档

- [优化计划](docs/optimization_plan.md) - 性能优化方案
- [重构计划](docs/refactoring_plan.md) - 详细重构设计
- [高度过滤指南](docs/height_filter_guide.md) - 高度过滤功能说明
- [重力对齐指南](docs/gravity_alignment_guide.md) - 重力对齐功能说明

## 🤝 贡献

如有问题或建议，请：
1. 查阅相关文档
2. 检查代码注释
3. 提交Issue或Pull Request

---

**重构完成日期**: 2026-01-21  
**版本**: v2.0  
**状态**: ✅ 已完成
