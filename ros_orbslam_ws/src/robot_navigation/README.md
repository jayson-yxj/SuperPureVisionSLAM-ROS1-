# ROS Navigation Stack 集成指南

本文档说明如何使用已集成的 ROS Navigation Stack 进行自主导航。

## ✅ 已完成功能

- ✅ TF 发布器（位姿转换）
- ✅ move_base 导航节点
- ✅ 全局/局部代价地图配置
- ✅ DWA 局部规划器
- ✅ 速度命令转发节点
- ✅ RViz 可视化配置
- ✅ 快速启动脚本

## 📋 前提条件

- ✅ 已有 OccupancyGrid 地图发布到 `/projected_map`
- ✅ ORB-SLAM3 提供位姿信息
- ✅ TF 树自动发布（map -> odom -> base_link）
- ⚠️ 需要机器人底盘控制接口（可选）

---

## 🚀 快速开始

### 1. 安装依赖

```bash
sudo apt-get install ros-noetic-navigation \
                     ros-noetic-move-base \
                     ros-noetic-amcl \
                     ros-noetic-map-server \
                     ros-noetic-dwa-local-planner
```

### 2. 启动导航系统

**方法1：使用快速启动脚本（推荐）**

```bash
# 终端1: 启动SLAM和建图
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./launch.sh

# 终端2: 启动导航（等待SLAM初始化完成）
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./start_navigation.sh
```

**方法2：手动启动**

```bash
# 终端1: 启动SLAM和建图
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./launch.sh

# 终端2: 启动导航
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
source devel/setup.bash
roslaunch robot_navigation navigation.launch
```

### 3. 发送导航目标

在 RViz 中：
1. 点击顶部工具栏的 **"2D Nav Goal"** 按钮
2. 在地图上点击目标位置
3. 拖动鼠标设置目标方向
4. 松开鼠标，机器人开始导航

### 4. 配置机器人参数（可选）

如需调整机器人参数，编辑 [`config/robot_params.yaml`](config/robot_params.yaml:1)：

```yaml
# 机器人物理参数
robot_radius: 0.2          # 机器人半径（米）
max_vel_x: 0.5            # 最大线速度（米/秒）
max_vel_theta: 1.0        # 最大角速度（弧度/秒）
```

修改后需要重新启动导航系统。

---

## 📁 文件结构

```
robot_navigation/
├── package.xml                    # ROS包配置
├── CMakeLists.txt                 # 编译配置
├── launch/
│   ├── navigation.launch          # 主启动文件
│   └── move_base.launch           # move_base配置
├── params/
│   ├── costmap_common_params.yaml # 代价地图通用参数
│   ├── local_costmap_params.yaml  # 局部代价地图
│   ├── global_costmap_params.yaml # 全局代价地图
│   ├── base_local_planner_params.yaml  # 局部规划器
│   └── dwa_local_planner_params.yaml   # DWA规划器
└── config/
    └── robot_params.yaml          # 机器人参数
```

---

## ⚙️ 关键配置说明

### 1. 地图话题映射

由于您的地图发布在 `/projected_map`，需要重映射：

```xml
<remap from="map" to="/projected_map"/>
```

### 2. 定位方式

**选项A: 使用 ORB-SLAM3 位姿（推荐）**
- 直接使用 ORB-SLAM3 的位姿
- 不需要 AMCL
- 需要发布 TF: `map -> odom -> base_link`

**选项B: 使用 AMCL**
- 在地图上进行粒子滤波定位
- 适合地图已知的情况
- 需要里程计信息

### 3. 代价地图配置

```yaml
# 全局代价地图 - 使用完整地图
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  static_map: false  # 使用动态地图
  rolling_window: false

# 局部代价地图 - 机器人周围小范围
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 4.0
  height: 4.0
  resolution: 0.05
```

---

## 🔧 核心组件说明

### 1. TF 发布器 ✅

**文件**: [`scripts/tf_publisher.py`](scripts/tf_publisher.py:1)

功能：
- 订阅 ORB-SLAM3 位姿 (`/orb_slam3/image_pose`)
- 发布 TF 树: `map -> odom -> base_link -> camera`
- 发布里程计消息 (`/odom`)

### 2. 速度命令转发 ✅

**文件**: [`scripts/cmd_vel_relay.py`](scripts/cmd_vel_relay.py:1)

功能：
- 订阅 move_base 的速度命令 (`/cmd_vel`)
- 速度限制和安全检查
- 转发到机器人底盘 (`/robot/cmd_vel`)

**配置**: 修改第40行的话题名以匹配实际机器人

### 3. move_base 导航 ✅

**文件**: [`launch/move_base.launch`](launch/move_base.launch:1)

功能：
- 全局路径规划（GlobalPlanner）
- 局部路径规划（DWA）
- 代价地图管理
- 恢复行为

---

## 📊 调试步骤

### 1. 验证地图

```bash
# 查看地图话题
rostopic echo /projected_map --noarr

# 在 RViz 中添加 Map 显示
# Topic: /projected_map
```

### 2. 检查 TF 树

```bash
# 查看 TF 树
rosrun tf view_frames

# 应该看到: map -> odom -> base_link
```

### 3. 测试导航

```bash
# 发送简单的速度命令测试
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

---

## 🎯 使用建议

1. **首次使用**: 建议先在仿真环境或空旷区域测试
2. **参数调优**: 根据实际机器人调整速度和代价地图参数
3. **地图质量**: 确保 SLAM 地图质量良好，避免在光照不足或纹理缺乏的环境使用
4. **实时性**: 如遇性能问题，可调整地图分辨率和更新频率
5. **安全性**: 首次连接实际机器人时，建议设置较低的速度限制

## 📖 详细文档

- **使用指南**: [`docs/USAGE_GUIDE.md`](docs/USAGE_GUIDE.md:1) - 详细的使用说明和调试方法
- **参数调优**: 查看各参数文件中的注释说明

---

## 📚 参考资源

- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
- [move_base Documentation](http://wiki.ros.org/move_base)
- [costmap_2d Documentation](http://wiki.ros.org/costmap_2d)

---

**注意**: 由于您的系统使用视觉SLAM，地图是实时更新的。建议：
1. 先在静态环境中测试
2. 确保地图质量稳定
3. 考虑添加地图保存/加载功能

如需帮助创建具体的配置文件和节点，请告诉我您的机器人具体参数。
