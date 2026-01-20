# 导航系统使用指南

本指南说明如何使用已集成的 ROS Navigation Stack 进行自主导航。

---

## 📋 系统架构

```
┌─────────────────┐
│  ORB-SLAM3      │ ──> 位姿 ──> TF Publisher ──> TF树 (map->odom->base_link)
└─────────────────┘                                    │
                                                       ↓
┌─────────────────┐                              ┌──────────┐
│ Depth Anything  │ ──> 深度图 ──┐               │ move_base│
└─────────────────┘              │               └──────────┘
                                 ↓                     │
┌─────────────────┐         ┌─────────┐              ↓
│ depth_maping    │ ──────> │ 点云    │ ──> 代价地图  │
│     _node       │         │ 2D地图  │              ↓
└─────────────────┘         └─────────┘         ┌─────────┐
                                                 │ /cmd_vel│
                                                 └─────────┘
                                                      │
                                                      ↓
                                              ┌──────────────┐
                                              │ 机器人底盘   │
                                              └──────────────┘
```

---

## 🚀 快速启动

### 1. 启动 SLAM 和建图系统

```bash
# 终端1: 启动 ORB-SLAM3 + 深度建图
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./launch.sh
```

等待系统初始化，确保：
- ✅ ORB-SLAM3 成功初始化
- ✅ 深度建图节点正常运行
- ✅ `/projected_map` 话题有数据发布

### 2. 启动导航系统

```bash
# 终端2: 启动导航
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
source devel/setup.bash
roslaunch robot_navigation navigation.launch
```

这将启动：
- ✅ TF 发布器（位姿转换）
- ✅ move_base（路径规划）
- ✅ RViz（可视化）

### 3. 在 RViz 中发送导航目标

1. 等待 RViz 窗口打开
2. 点击顶部工具栏的 **"2D Nav Goal"** 按钮
3. 在地图上点击目标位置
4. 拖动鼠标设置目标方向
5. 松开鼠标，机器人开始导航

---

## 🔧 配置说明

### 机器人参数配置

编辑 [`robot_params.yaml`](ros_orbslam_ws/src/robot_navigation/config/robot_params.yaml:1)：

```yaml
# 根据实际机器人调整
robot_radius: 0.2          # 机器人半径（米）
max_vel_x: 0.5            # 最大线速度（米/秒）
max_vel_theta: 1.0        # 最大角速度（弧度/秒）
```

### 速度命令转发

如果需要连接实际机器人底盘，修改 [`cmd_vel_relay.py`](ros_orbslam_ws/src/robot_navigation/scripts/cmd_vel_relay.py:1)：

```python
# 第 40 行：修改为实际机器人的速度命令话题
self.robot_cmd_pub = rospy.Publisher(
    '/robot/cmd_vel',  # 改为实际话题，如 /turtlebot3/cmd_vel
    Twist, 
    queue_size=10
)
```

然后启动时启用转发：

```bash
roslaunch robot_navigation navigation.launch enable_cmd_relay:=true
```

### TF 坐标系配置

如果相机安装位置不在机器人中心，修改 [`tf_publisher.py`](ros_orbslam_ws/src/robot_navigation/scripts/tf_publisher.py:1) 的静态 TF：

```python
# 第 52-54 行：调整相机相对于 base_link 的位置
static_tf.transform.translation.x = 0.0  # 前方偏移（米）
static_tf.transform.translation.y = 0.0  # 左右偏移（米）
static_tf.transform.translation.z = 0.2  # 高度偏移（米）
```

---

## 📊 调试与监控

### 1. 检查话题

```bash
# 查看地图话题
rostopic echo /projected_map --noarr

# 查看点云话题
rostopic echo /o3d_pointCloud --noarr

# 查看速度命令
rostopic echo /cmd_vel

# 查看导航状态
rostopic echo /move_base/status
```

### 2. 检查 TF 树

```bash
# 查看 TF 树结构
rosrun tf view_frames

# 查看特定 TF 变换
rosrun tf tf_echo map base_link
```

应该看到完整的 TF 树：
```
map -> odom -> base_link -> camera
```

### 3. 查看代价地图

在 RViz 中：
- **Global Costmap**: 全局代价地图（完整地图）
- **Local Costmap**: 局部代价地图（机器人周围）
- **Global Plan**: 全局路径（绿色）
- **Local Plan**: 局部路径（红色）

### 4. 常见问题排查

#### 问题1: 机器人不移动

**检查项**：
```bash
# 1. 检查是否有速度命令发布
rostopic hz /cmd_vel

# 2. 检查 move_base 状态
rostopic echo /move_base/status

# 3. 检查 TF 是否正常
rosrun tf tf_monitor
```

**可能原因**：
- TF 树不完整
- 代价地图中目标点被标记为障碍物
- 机器人底盘未连接

#### 问题2: 路径规划失败

**检查项**：
```bash
# 查看 move_base 日志
rosnode info /move_base
```

**可能原因**：
- 地图质量差（点云稀疏）
- 目标点在障碍物内
- 代价地图参数不合理

#### 问题3: 地图不更新

**检查项**：
```bash
# 检查地图发布频率
rostopic hz /projected_map

# 检查点云发布频率
rostopic hz /o3d_pointCloud
```

**可能原因**：
- [`depth_maping_node.py`](ros_orbslam_ws/src/depth_maping/scripts/depth_maping_node.py:1) 未运行
- ORB-SLAM3 跟踪丢失

---

## ⚙️ 参数调优

### 代价地图参数

编辑 [`costmap_common_params.yaml`](ros_orbslam_ws/src/robot_navigation/params/costmap_common_params.yaml:1)：

```yaml
# 增大障碍物检测范围
obstacle_range: 5.0        # 改为 8.0
raytrace_range: 6.0        # 改为 10.0

# 调整膨胀半径（安全距离）
inflation_radius: 0.5      # 根据机器人大小调整
```

### DWA 规划器参数

编辑 [`dwa_local_planner_params.yaml`](ros_orbslam_ws/src/robot_navigation/params/dwa_local_planner_params.yaml:1)：

```yaml
# 调整速度限制
max_vel_x: 0.5            # 降低以提高安全性
min_vel_x: 0.1            # 最小速度

# 调整轨迹评分权重
path_distance_bias: 32.0  # 增大以更紧密跟随全局路径
goal_distance_bias: 24.0  # 增大以更快接近目标
occdist_scale: 0.01       # 增大以更远离障碍物
```

### 地图分辨率

编辑 [`depth_maping_node.py`](ros_orbslam_ws/src/depth_maping/scripts/depth_maping_node.py:404)：

```python
# 第 404 行：调整地图分辨率
occ_msg = self.project_to_2d_occupancy(
    resolution=0.05,  # 降低以提高精度（0.02-0.1）
    ...
)
```

---

## 🎯 高级功能

### 1. 保存和加载地图

```bash
# 保存当前地图
rosrun map_server map_saver -f my_map map:=/projected_map

# 加载已保存的地图
rosrun map_server map_server my_map.yaml
```

### 2. 设置初始位姿

在 RViz 中：
1. 点击 **"2D Pose Estimate"**
2. 在地图上点击机器人当前位置
3. 拖动设置当前朝向

### 3. 多目标点导航

使用 Python 脚本发送多个目标点：

```python
#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('multi_goal_navigation')
    
    # 定义多个目标点
    goals = [
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 0.0),
        (0.0, 2.0, 0.0),
    ]
    
    for x, y, yaw in goals:
        print(f"导航到: ({x}, {y})")
        result = send_goal(x, y, yaw)
        print(f"结果: {result}")
```

---

## 📚 参考资料

- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
- [move_base Documentation](http://wiki.ros.org/move_base)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)
- [costmap_2d Documentation](http://wiki.ros.org/costmap_2d)

---

## 🔄 完整启动流程

```bash
# 1. 启动 SLAM 和建图
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./launch.sh

# 2. 等待系统初始化（约10秒）

# 3. 新终端：启动导航
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
source devel/setup.bash
roslaunch robot_navigation navigation.launch

# 4. 在 RViz 中设置导航目标

# 5. （可选）连接实际机器人底盘
roslaunch robot_navigation navigation.launch enable_cmd_relay:=true
```

---

## ⚠️ 注意事项

1. **地图质量**：导航效果依赖于地图质量，建议在光照良好、纹理丰富的环境中使用
2. **尺度问题**：单目 SLAM 存在尺度不确定性，建议使用 `translation_size` 参数校准
3. **实时性**：深度估计和点云处理较耗时，建议使用 GPU 加速
4. **安全性**：首次使用时建议在仿真环境或空旷区域测试
5. **TF 同步**：确保所有传感器数据的时间戳同步

---

如有问题，请查看日志：
```bash
# 查看所有节点日志
rosnode list
rosnode info <node_name>

# 查看特定话题
rostopic list
rostopic info <topic_name>
```
