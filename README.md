# ORB-SLAM3 深度映射 ROS 集成

基于 ORB-SLAM3 的视觉 SLAM 系统，集成深度估计和 3D 点云重建功能。

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
- [贡献指南](#贡献指南)
- [许可证](#许可证)

## 🎯 项目简介

本项目是一个完整的视觉 SLAM 解决方案，结合了：
- **ORB-SLAM3**：用于相机位姿估计和特征跟踪
- **Depth Anything V2**：基于深度学习的单目深度估计
- **ROS Noetic**：机器人操作系统集成
- **Open3D**：3D 点云可视化和处理

系统能够从单目相机输入实时生成稠密的 3D 点云地图。

## ✨ 主要特性

- ✅ 单目视觉 SLAM（支持鱼眼相机）
- ✅ 实时深度估计（Depth Anything V2）
- ✅ 3D 点云重建和可视化
- ✅ ROS 话题发布（图像、位姿、点云）
- ✅ 自适应关键帧选择
- ✅ 点云降采样和优化
- ✅ 路径规划集成
- ✅ 支持离线和在线处理

## 💻 系统要求

- **操作系统**：Ubuntu 20.04 LTS
- **ROS 版本**：ROS Noetic
- **Python**：3.8+
- **CUDA**：10.2+ (推荐用于深度估计加速)
- **内存**：至少 8GB RAM
- **存储**：至少 10GB 可用空间

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
```

### Python 依赖
```bash
pip install torch torchvision torchaudio
pip install open3d
pip install pypose
pip install opencv-python
pip install numpy scipy matplotlib
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

### 启动系统

#### 方法 1：使用启动脚本（推荐）
```bash
cd ros_orbslam_ws
./run.sh
```

#### 方法 2：手动启动各个节点
```bash
# 终端 1：启动 ROS Master
roscore

# 终端 2：启动深度映射节点
rosrun depth_maping depth_maping_node.py

# 终端 3：启动 ORB-SLAM3 节点
rosrun orb_slam3_ros orb_mono ../Vocabulary/ORBvoc.txt ../MonoConfig/Fisheye.yaml

# 终端 4（可选）：启动视频发布节点
rosrun pub_video pub_video_node.py
```

### 禁用可视化（远程或无头环境）
```bash
rosrun depth_maping depth_maping_node.py _enable_visualization:=false
```

### 查看点云
```bash
# 使用 RViz
rviz -d ros_orbslam_ws/src/depth_maping/rviz/HT_vslam.rviz

# 或使用 ROS 话题
rostopic echo /o3d_pointCloud
```

## ⚙️ 配置说明

### 相机参数配置
编辑 `MonoConfig/Fisheye.yaml` 文件：
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

### 深度估计配置
编辑 `ros_orbslam_ws/src/depth_maping/scripts/depth_maping_node.py`：
```python
# 模型配置
self.encoder = 'vitb'  # 可选: 'vits', 'vitb', 'vitl', 'vitg'
self.max_depth = 70.0  # 最大深度范围（米）

# 关键帧选择参数
dis_range = 4.0        # 平移距离阈值（米）
yaw_range = 25.0       # 偏航角阈值（度）
pitch_range = 25.0     # 俯仰角阈值（度）
```

## 📁 项目结构

```
orbslam_depthmaping_ros/
├── build.sh                    # 自动构建脚本
├── CMakeLists.txt             # ORB-SLAM3 主 CMake 配置
├── include/                   # ORB-SLAM3 头文件
├── src/                       # ORB-SLAM3 源代码
├── lib/                       # 编译后的库文件
├── Thirdparty/               # 第三方库
│   ├── DBoW2/               # 词袋库
│   └── g2o/                 # 图优化库
├── Vocabulary/              # ORB 词汇表（需下载）
├── MonoConfig/              # 相机配置文件
│   └── Fisheye.yaml
└── ros_orbslam_ws/          # ROS 工作空间
    ├── src/
    │   ├── ORB_SLAM3_ROS/   # ORB-SLAM3 ROS 包装
    │   ├── depth_maping/    # 深度映射节点
    │   │   ├── scripts/
    │   │   │   ├── depth_maping_node.py      # 主节点
    │   │   │   ├── plan_path.py              # 路径规划
    │   │   │   └── depth_anything_v2/        # 深度估计模型
    │   │   └── msg/
    │   │       └── ImagePose.msg             # 自定义消息
    │   └── pub_video/       # 视频发布节点
    └── run.sh               # 启动脚本
```

## 🔧 常见问题

### 1. 模块导入错误
```
ModuleNotFoundError: No module named 'plan_path'
```
**解决方案**：已在代码中自动添加脚本目录到 Python 路径。

### 2. Open3D GLX 错误
```
[Open3D WARNING] GLFW Error: GLX: Failed to make context current
```
**解决方案**：
- 使用 `_enable_visualization:=false` 参数禁用可视化
- 或配置 X11 转发：`xhost +local:`

### 3. GitHub 大文件推送失败
```
error: File Vocabulary/ORBvoc.txt is 138.52 MB; this exceeds GitHub's file size limit
```
**解决方案**：词汇表和模型文件已添加到 `.gitignore`，需要单独下载。

### 4. 编译错误
```
fatal error: Eigen/Core: No such file or directory
```
**解决方案**：
```bash
sudo apt-get install libeigen3-dev
```

### 5. ROS 话题无数据
**检查步骤**：
```bash
# 查看活动话题
rostopic list

# 查看话题信息
rostopic info /orb_slam3/image_pose

# 查看话题数据
rostopic echo /orb_slam3/image_pose
```

## 🤝 贡献指南

欢迎贡献！请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 📝 更新日志

### v1.0.0 (2025-12-24)
- ✨ 初始版本发布
- ✨ 集成 ORB-SLAM3 和 Depth Anything V2
- ✨ 添加自动构建脚本
- 🐛 修复模块导入问题
- 🐛 修复 Open3D 可视化错误
- 📝 完善文档

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

## 📧 联系方式

如有问题或建议，请通过以下方式联系：
- 📧 Email: 3630115255@qq.com
- 🐛 Issues: [GitHub Issues](https://github.com/jayson-yxj/orbslam_depthmaping_ros/issues)

---

⭐ 如果这个项目对你有帮助，请给个 Star！