# 视觉 SLAM 建图导航系统 ROS 集成

基于 ORB-SLAM3 的视觉 SLAM 系统，集成深度估计、3D 点云重建和机器人导航功能。支持单目和双目相机，提供完整的建图导航解决方案。
=======
单目基于 ORB-SLAM3 的视觉 SLAM 系统，集成深度估计和 3D 点云重建功能演示：
<img width="1906" height="1035" alt="image" src="https://github.com/user-attachments/assets/e612218b-7c36-474b-8a05-53a5cef22a38" />
<img width="605" height="318" alt="image" src="https://github.com/user-attachments/assets/6fad0955-22b5-4603-9e24-cdd833b0bf23" />
<img width="1272" height="635" alt="image" src="https://github.com/user-attachments/assets/bb2339f9-dc8a-4979-b853-a78c65a52e04" />

双目基于 ORB-SLAM3 的视觉 SLAM 系统，集成深度估计和 3D 点云重建功能演示：
<img width="1318" height="580" alt="image" src="https://github.com/user-attachments/assets/eb8f4949-9aff-4b76-a282-0f4192bd6114" />
<img width="1423" height="789" alt="image" src="https://github.com/user-attachments/assets/1d392279-53a6-43c0-83e8-eebd23467222" />

## 📋 目录

- [项目简介](#项目简介)
- [主要特性](#主要特性)
- [系统要求](#系统要求)
- [依赖项](#依赖项)
- [安装步骤](#安装步骤)
- [使用方法](#使用方法)
- [配置说明](#配置说明)
- [项目结构](#项目结构)
- [常见问题](#常见问题)
- [更新日志](#更新日志)
- [贡献指南](#贡献指南)
- [许可证](#许可证)

## 🎯 项目简介

本项目是一个完整的视觉 SLAM 解决方案，支持**单目**和**双目**两种模式，结合了：
- **ORB-SLAM3**：用于相机位姿估计和特征跟踪
- **Depth Anything V2**：基于深度学习的单目深度估计
- **Monster**：混合单目先验和双目立体匹配的深度估计
- **ROS Noetic**：机器人操作系统集成
- **Open3D**：3D 点云可视化和处理
- **Move Base**：机器人导航和路径规划
- **Gazebo**：机器人仿真环境

系统能够从单目或双目相机输入实时生成稠密的 3D 点云地图，并支持机器人自主导航。

## ✨ 主要特性

### 核心功能
- ✅ **单目视觉 SLAM**（支持鱼眼相机）
- ✅ **双目视觉 SLAM**（支持标准双目相机）
- ✅ **动态物体过滤**（🆕 v1.3.0）
  - YOLOv26-seg 实时分割
  - 像素级动态区域过滤
  - 提升动态场景下的SLAM精度
- ✅ **实时深度估计**
  - 单目模式：Depth Anything V2
  - 双目模式：Monster（混合单目先验 + 立体匹配）
- ✅ **3D 点云重建和可视化**
- ✅ **2D 占用栅格地图生成**
- ✅ **TF 坐标变换发布**
- ✅ **机器人导航集成**（Move Base）
- ✅ **Gazebo 仿真支持**

### 高级特性
- ✅ **FOV感知点云管理**（🆕 v1.2.0）
  - "关注之处必更新，过往之域永留存"
  - 视野内点云实时更新，不累加
  - 历史点云永久保留
  - 体素网格加速（性能提升100倍）
- ✅ **重力对齐**（自动估计重力方向）
- ✅ **并行处理模式**（提升30-50%性能）
- ✅ **滑动窗口点云管理**
- ✅ **自适应高度过滤**
- ✅ **体素下采样优化**
- ✅ **深度图降噪处理**
- ✅ **模块化架构设计**

### ROS 集成
- ✅ 标准 ROS 话题发布（图像、位姿、点云、地图）
- ✅ TF 树发布（map -> odom -> base_link -> camera）
- ✅ 里程计消息发布
- ✅ 支持离线和在线处理
- ✅ 完整的 Launch 文件配置

## 💻 系统要求

- **操作系统**：Ubuntu 20.04 LTS
- **ROS 版本**：ROS Noetic
- **Python**：3.8+
- **CUDA**：10.2+ (推荐用于深度估计加速)
- **内存**：至少 8GB RAM（推荐 16GB）
- **存储**：至少 10GB 可用空间
- **GPU**：NVIDIA GPU（推荐用于实时处理）

## 📦 依赖项

### C++ 依赖
```bash
# 基础工具
sudo apt-get update
sudo apt-get install -y build-essential cmake git

# OpenCV 4.2+
sudo apt-get install -y libopencv-dev

# Eigen3
sudo apt-get install -y libeigen3-dev

# Pangolin
sudo apt-get install -y libglew-dev libpython2.7-dev
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# ROS Noetic
sudo apt-get install -y ros-noetic-desktop-full
sudo apt-get install -y ros-noetic-cv-bridge ros-noetic-sensor-msgs
sudo apt-get install -y ros-noetic-navigation ros-noetic-move-base
```

### Python 依赖
```bash
pip install torch torchvision torchaudio
pip install open3d
pip install pypose
pip install opencv-python
pip install numpy scipy matplotlib
pip install pyyaml
pip install ultralytics onnx  # 用于动态物体分割
```

### 深度估计模型
下载 Depth Anything V2 模型权重：
```bash
# 下载模型文件（约 400MB）
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Base/resolve/main/depth_anything_v2_metric_hypersim_vitb.pth
# 将模型放置到指定位置
mv depth_anything_v2_metric_hypersim_vitb.pth ~/Downloads/
```

### ORB 词汇表
下载 ORB 词汇表文件（约 139MB）：
```bash
# 从 ORB-SLAM3 官方仓库下载
wget https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt
mkdir -p Vocabulary
mv ORBvoc.txt Vocabulary/
```

## 🚀 安装步骤

### 1. 克隆仓库
```bash
cd ~/Desktop
git clone https://github.com/jayson-yxj/orbslam_depthmaping_ros.git
cd orbslam_depthmaping_ros
```

### 2. 构建项目
使用提供的构建脚本：
```bash
chmod +x build.sh
./build.sh
```

构建脚本会自动完成以下步骤：
1. 构建 DBoW2 词袋库
2. 构建 g2o 图优化库
3. 构建 ORB-SLAM3 主库
4. 构建 ROS 工作空间

### 3. 配置环境
```bash
# 添加到 ~/.bashrc
echo "source ~/Desktop/orbslam_depthmaping_ros/ros_orbslam_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🎮 使用方法

### 模式选择

本项目支持两种工作模式：

#### 1. 单目模式（Depth Anything V2）
适用于单目相机，使用深度学习进行深度估计。

#### 2. 双目模式（Monster）🆕
适用于双目相机，结合单目先验和立体匹配，提供更高质量的深度估计。

---

### 快速启动

**使用启动脚本（推荐）**
```bash
cd ros_orbslam_ws
./launch.sh
```

### 高级功能

#### 启用并行处理模式
编辑 `ros_orbslam_ws/src/depth_maping/config/default_config.yaml`：
```yaml
parallel_processing:
  enabled: true
  queue_size: 10
```

#### 启用重力对齐
```yaml
gravity_alignment:
  enabled: true
  save_interval: 1.0
  check_interval: 0.5
```

#### 调整点云过滤参数
```yaml
point_cloud:
  filter:
    depth_range: [0.1, 100.0]
    height_filter_mode: relative
    height_ratio_range: [0.5, 1.0]
  voxel_size: 1.0
```

## ⚙️ 配置说明

### 单目模式主配置文件
`ros_orbslam_ws/src/depth_maping/config/default_config.yaml`

#### 深度估计配置
```yaml
depth_estimator:
  type: depth_anything_v2
  model_path: /home/sunteng/Downloads/depth_anything_v2_metric_hypersim_vitb.pth
  encoder: vitb  # 可选: vits, vitb, vitl, vitg
  input_size: 384
  max_depth: 70.0
  device: cuda
```

#### 点云生成配置
```yaml
point_cloud:
  crop_params:
    img_crop_size: 128
    max_depth: 40.0
  filter:
    depth_range: [0.1, 100.0]
    height_filter_mode: relative
    height_ratio_range: [0.5, 1.0]
  voxel_size: 1.0
```

#### 地图构建配置
```yaml
map:
  type: occupancy_grid
  sliding_window:
    enabled: true
    size: 5
  resolution: 0.8
  height_range: [0.5, 0.6]
  use_ratio: true
  occupied_thresh: 3
```

#### 位姿处理配置
```yaml
pose:
  translation_scale: 18  # 平移向量缩放倍率（与TF发布节点共用）
```

### 双目模式主配置文件
`ros_orbslam_ws/src/monster/config/monster_config.yaml`

### 相机参数配置
编辑 `MonoConfig/Fisheye.yaml`：
```yaml
Camera.fx: 138.54264656
Camera.fy: 138.60053687
Camera.cx: 331.89824222
Camera.cy: 239.70296783

# 鱼眼畸变参数
Camera.k1: -0.05094921
Camera.k2: -0.00983458
Camera.k3: 0.00521841
Camera.k4: -0.00128268
```

### 导航参数配置
编辑 `ros_orbslam_ws/src/robot_navigation/params/`：
- `costmap_common_params.yaml` - 代价地图通用参数
- `global_costmap_params.yaml` - 全局代价地图参数
- `dwa_local_planner_params.yaml` - DWA 局部规划器参数

## 📁 项目结构

```
orbslam_depthmaping_ros_2/
├── build.sh                    # 自动构建脚本
├── CMakeLists.txt             # ORB-SLAM3 主 CMake 配置
├── .gitignore                 # Git 忽略文件配置
├── README.md                  # 项目主文档
├── include/                   # ORB-SLAM3 头文件
├── src/                       # ORB-SLAM3 源代码
├── lib/                       # 编译后的库文件
├── Thirdparty/               # 第三方库
│   ├── DBoW2/               # 词袋库
│   └── g2o/                 # 图优化库
├── Vocabulary/              # ORB 词汇表（需下载）
├── MonoConfig/              # 相机配置文件
│   ├── Fisheye.yaml         # 鱼眼相机配置
│   ├── Gazebo_Stereo.yaml   # Gazebo 双目配置
│   ├── ZED_Mini.yaml        # ZED Mini 配置
│   └── USBCam.yaml          # USB 相机配置
└── ros_orbslam_ws/          # ROS 工作空间
    ├── src/
    │   ├── ORB_SLAM3_ROS/   # ORB-SLAM3 ROS 包装
    │   │   └── src/
    │   │       ├── ros_mono.cc              # 单目节点
    │   │       ├── ros_stereo.cc            # 双目节点
    │   │       └── ros_stereo_with_mask.cc  # 双目节点(带动态过滤) 🆕
    │   │
    │   ├── segmentation/    # 动态物体分割节点 🆕
    │   │   ├── config/
    │   │   ├── scripts/
    │   │   │   ├── dynamic_segmentor_node.py  # 分割节点
    │   │   │   ├── demo.py                    # 可视化demo
    │   │   │   └── export_onnx.py             # ONNX导出
    │   │   ├── launch/
    │   │   │   └── dynamic_segmentor.launch
    │   │   ├── checkpoints/
    │   │   │   └── yolo26n-seg.pt             # YOLOv26模型
    │   │   └── README.md                      # 分割系统文档
    │   │
    │   ├── depth_maping/    # 单目深度映射节点
    │   │   ├── config/
    │   │   │   └── default_config.yaml      # 主配置文件
    │   │   ├── scripts/
    │   │   │   ├── depth_maping_node.py     # 主节点
    │   │   │   ├── pipeline_manager.py      # 串行处理管理器
    │   │   │   ├── parallel_pipeline_manager.py  # 并行处理管理器
    │   │   │   ├── gravity_estimate.py      # 重力估计
    │   │   │   ├── depth_estimator/         # 深度估计模块
    │   │   │   ├── point_cloud/             # 点云生成模块
    │   │   │   ├── map_builder/             # 地图构建模块
    │   │   │   └── depth_anything_v2/       # 深度估计模型
    │   │   ├── launch/
    │   │   │   └── slam_mapping.launch
    │   │   └── rviz/
    │   │       └── slam_mapping.rviz
    │   │
    │   ├── monster/         # 双目深度估计节点 🆕
    │   │   ├── config/
    │   │   │   └── monster_config.yaml      # Monster 配置
    │   │   ├── scripts/
    │   │   │   ├── monster_stereo_node.py   # 主节点
    │   │   │   ├── test_fov_aware.py        # FOV 测试脚本
    │   │   │   ├── point_cloud/             # 点云生成模块
    │   │   │   ├── map_builder/             # 地图构建模块（含FOV感知）
    │   │   │   ├── gravity_estimator/       # 重力估计
    │   │   │   └── core/                    # Monster 核心算法
    │   │   ├── launch/
    │   │   │   └── monster_stereo.launch
    │   │   ├── rviz/
    │   │   │   └── monster_mapping.rviz
    │   │   ├── docs/
    │   │   │   └── FOV_AWARE_MODE_GUIDE.md  # FOV感知模式指南
    │   │   └── README.md                    # Monster 文档
    │   │
    │   ├── robot_navigation/    # 机器人导航节点
    │   │   ├── scripts/
    │   │   │   ├── tf_publisher.py          # TF 发布节点
    │   │   │   └── local_pointcloud_filter.py
    │   │   ├── launch/
    │   │   │   ├── navigation.launch
    │   │   │   ├── navigation_stereo.launch  # 双目导航
    │   │   │   └── move_base.launch
    │   │   └── params/                      # 导航参数
    │   │
    │   ├── aws-robomaker-small-house-world/  # Gazebo 仿真环境 🆕
    │   │   ├── worlds/
    │   │   │   └── small_house.world
    │   │   ├── models/                      # 3D 模型
    │   │   ├── launch/
    │   │   │   ├── stereo_robot.launch      # 双目机器人
    │   │   │   └── stereo_robot_with_slam.launch
    │   │   ├── examples/
    │   │   │   └── stereo_robot.urdf.xacro  # 机器人模型
    │   │   └── README_STEREO_ROBOT.md       # 仿真指南
    │   │
    │   ├── pub_video/           # 视频发布节点
    │   │   └── scripts/
    │   │       ├── pub_video_node.py        # 单目视频发布
    │   │       └── pub_video_node_stereo.py # 双目视频发布
    │   │
    │   └── aws-robomaker-hospital-world/  # Gazebo 医院环境 🆕
    │       ├── worlds/
    │       │   └── hospital.world
    │       ├── models/                    # 3D 模型
    │       ├── launch/
    │       │   ├── stereo_robot.launch
    │       │   └── stereo_robot_with_slam.launch  # 集成动态过滤
    │       └── examples/
    │           └── stereo_robot.urdf.xacro
    │
    ├── run.sh                   # 启动脚本
    └── launch.sh                # Launch 文件启动脚本
```

## 🔧 常见问题

### 1. 点云范围限制问题
**问题**：当机器人远离原点时，点云不再显示。

**原因**：深度过滤使用了到世界坐标系原点的距离，而不是到相机的距离。

**解决方案**：已在 v1.1.0 中修复，深度过滤现在基于相机坐标系。

### 2. TF 坐标变换错误
**问题**：机器人旋转时会绕着 map 原点旋转。

**原因**：ORB-SLAM3 输出的是 Twc（World->Camera），需要取逆。

**解决方案**：已在 v1.1.0 中修复，参见 `docs/TF_FIX_GUIDE.md`。

### 3. 模块导入错误
```
ModuleNotFoundError: No module named 'plan_path'
```
**解决方案**：已在代码中自动添加脚本目录到 Python 路径。

### 4. Open3D GLX 错误
```
[Open3D WARNING] GLFW Error: GLX: Failed to make context current
```
**解决方案**：
- 在配置文件中设置 `visualization.enabled: false`
- 或配置 X11 转发：`xhost +local:`

### 5. GitHub 大文件推送失败
```
error: File Vocabulary/ORBvoc.txt is 138.52 MB; this exceeds GitHub's file size limit
```
**解决方案**：词汇表和模型文件已添加到 `.gitignore`，需要单独下载。

### 6. 编译错误
```
fatal error: Eigen/Core: No such file or directory
```
**解决方案**：
```bash
sudo apt-get install libeigen3-dev
```

### 7. ROS 话题无数据
**检查步骤**：
```bash
# 查看活动话题
rostopic list

# 查看话题信息
rostopic info /orb_slam3/image_pose

# 查看话题数据
rostopic echo /orb_slam3/image_pose
```

### 8. 性能优化建议
- 启用并行处理模式（提升 30-50% 性能）
- 调整体素下采样大小（`voxel_size`）
- 使用 GPU 加速深度估计
- 减小输入图像分辨率
- 调整滑动窗口大小

## 📝 更新日志

### v1.3.0 (2026-02-10) 🆕
- ✨ **新增动态物体分割系统**
  - YOLOv26-seg 实时实例分割
  - 像素级动态区域过滤（人、车辆、动物等）
  - 支持分割掩码和边界框两种模式
  - ROS节点架构，易于集成和调试
  - 实时性能监控（FPS、推理时间、动态占比）
- ✨ **集成到ORB-SLAM3**
  - 新增 `ros_stereo_with_mask.cc` 节点
  - 订阅动态掩码并应用到图像
  - 提升动态场景下的SLAM精度和建图质量
- ✨ **完整的Launch文件集成**
  - 支持一键启动完整系统
  - 可配置掩码模式（segmentation/bbox）
  - 可配置置信度阈值和启用状态
- 📝 **新增详细文档**
  - 动态分割系统使用指南
  - 性能优化建议
  - 故障排除指南

### v1.2.0 (2026-02-04)
- ✨ **新增 Monster 双目深度估计系统**
  - 混合单目先验和双目立体匹配
  - 支持实时深度估计和点云生成
  - 集成重力对齐功能
- ✨ **新增 FOV感知点云管理模式**
  - "关注之处必更新，过往之域永留存"哲学
  - 视野内点云实时更新，不累加
  - 历史点云永久保留
  - 体素网格加速FOV判断（性能提升100倍）
  - 可配置FOV参数（水平/垂直视场角、最大距离）
  - 历史点云数量限制（默认10万点）
- ✨ **新增 Gazebo 仿真支持**
  - 双目视觉机器人模型
  - 小房子仿真环境
  - 完整的 SLAM + 导航集成
- 📝 **新增详细文档**
  - FOV感知模式使用指南
  - Monster 系统文档
  - 双目机器人仿真指南
- 🧪 **新增测试脚本**
  - FOV感知模式单元测试
  - 自动化验证流程

### v1.1.0 (2026-01-22)
- 🐛 **修复点云范围限制bug**：深度过滤现在基于相机坐标系而非世界坐标系
- 🐛 **修复TF坐标变换bug**：正确处理 Twc 到 Tcw 的逆变换
- 🐛 **修复机器人朝向问题**：调整 CV 到 ROS 坐标系转换矩阵
- ✨ **统一尺度因子配置**：TF 发布节点现在从配置文件读取尺度因子
- ✨ **改进 .gitignore**：自动忽略 build 目录和临时文件
- ✨ **实现模块化架构**
- ✨ **添加并行处理支持**
- ✨ **添加重力对齐功能**
- ✨ **集成机器人导航**
- 📝 **更新文档**：添加详细的配置说明和故障排除指南

### v1.0.0 (2025-12-24)
- ✨ 初始版本发布
- ✨ 集成 ORB-SLAM3 和 Depth Anything V2
- ✨ 添加自动构建脚本
- 🐛 修复模块导入问题
- 🐛 修复 Open3D 可视化错误
- 📝 完善文档

## 🤝 贡献指南

欢迎贡献！请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

### 代码规范
- Python 代码遵循 PEP 8 规范
- C++ 代码遵循 Google C++ Style Guide
- 提交信息使用中文或英文，格式清晰
- 添加必要的注释和文档

## 📄 许可证

本项目基于以下开源项目：
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) - GPLv3
- [Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) - Apache 2.0

## 👥 作者

- **jayson-yxj** - [GitHub](https://github.com/jayson-yxj)

## 🙏 致谢

- ORB-SLAM3 团队
- Depth Anything V2 团队
- Open3D 社区
- ROS 社区
- PyPose 团队

## 📧 联系方式

如有问题或建议，请通过以下方式联系：
- 📧 Email: 3630115255@qq.com
- 🐛 Issues: [GitHub Issues](https://github.com/jayson-yxj/orbslam_depthmaping_ros/issues)

## 📚 相关资源

### 论文
- [ORB-SLAM3 论文](https://arxiv.org/abs/2007.11898)
- [Depth Anything V2 论文](https://arxiv.org/abs/2406.09414)
- [MonSter 论文](https://arxiv.org/abs/xxxx.xxxxx)

### 文档
- [ROS Noetic 文档](http://wiki.ros.org/noetic)
- [Open3D 文档](http://www.open3d.org/docs/)
- [Gazebo 文档](http://gazebosim.org/tutorials)

### 项目内文档
- [动态物体分割系统文档](ros_orbslam_ws/src/segmentation/README.md) 🆕
- [Monster 系统文档](ros_orbslam_ws/src/monster/README.md)
- [FOV感知模式指南](ros_orbslam_ws/src/monster/docs/FOV_AWARE_MODE_GUIDE.md)
- [双目机器人仿真指南](ros_orbslam_ws/src/aws-robomaker-small-house-world/README_STEREO_ROBOT.md)
- [TF 修复指南](ros_orbslam_ws/src/robot_navigation/docs/TF_FIX_GUIDE.md)

---

⭐ 如果这个项目对你有帮助，请给个 Star！

## 🌟 项目亮点

### 动态物体过滤 🆕
针对动态场景下SLAM精度下降的问题，本项目集成了YOLOv26-seg实时分割系统：
- ✅ 像素级精确过滤动态物体（人、车辆、动物等）
- ✅ 避免动态特征点干扰SLAM跟踪
- ✅ 提升建图质量和定位精度
- ✅ 支持GPU加速，实时性能优异（30-150 FPS）
- ✅ 灵活配置，支持分割掩码和边界框两种模式

### FOV感知点云管理
本项目独创的"关注之处必更新，过往之域永留存"点云管理哲学，解决了传统SLAM系统中点云无限累积导致的性能问题。通过智能的视野判断和历史管理，实现了：
- ✅ 视野内点云实时更新，避免重复累积
- ✅ 历史点云永久保留，形成完整地图
- ✅ 性能优化，内存可控
- ✅ 适用于长时间运行的机器人系统

### 双模式支持
- **单目模式**：适用于资源受限场景，使用深度学习估计深度
- **双目模式**：提供更高质量的深度估计，结合单目先验和立体匹配

### 完整的仿真环境
提供 Gazebo 仿真环境，无需真实硬件即可测试完整系统。
