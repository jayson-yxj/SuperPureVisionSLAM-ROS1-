# TF 外推错误修复指南

## 🔍 问题描述

错误信息：
```
[ERROR] Extrapolation Error: Lookup would require extrapolation into the future
[ERROR] Global Frame: odom Plan Frame size 8: map
[WARN] Could not transform the global plan to the frame of the controller
[ERROR] Could not get local plan
```

## 🎯 问题原因

1. **TF 时间戳不同步**：TF 发布的时间戳与 move_base 请求的时间戳不匹配
2. **TF 发布频率不足**：20Hz 可能不够，导航系统需要更高频率
3. **坐标系配置不一致**：全局规划在 `map`，局部控制器在 `odom`

## ✅ 已应用的修复

### 1. 统一时间戳

修改了 [`tf_publisher.py`](../scripts/tf_publisher.py:1)，所有 TF 使用相同的时间戳：

```python
# 修复前：每个 TF 独立使用 rospy.Time.now()
def publish_map_to_odom(self):
    t.header.stamp = rospy.Time.now()  # ❌ 时间戳不一致

# 修复后：统一使用同一时间戳
def timer_callback(self, event):
    current_time = rospy.Time.now()  # ✅ 统一时间戳
    self.publish_map_to_odom(current_time)
    self.publish_odom_to_base_link(current_time)
```

### 2. 提高 TF 发布频率

TF 发布器已设置为 20Hz，如需更高频率，修改 [`tf_publisher.py`](../scripts/tf_publisher.py:1) 第 36 行：

```python
# 当前：20Hz
self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

# 可选：50Hz（更流畅，但占用更多资源）
self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
```

## 🔧 进一步调试步骤

### 1. 运行 TF 诊断工具

```bash
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
source devel/setup.bash
./src/robot_navigation/scripts/diagnose_tf.sh
```

这将检查：
- ✅ TF 发布器是否运行
- ✅ TF 树是否完整
- ✅ 各个 TF 变换是否正常
- ✅ TF 发布频率
- ✅ 位姿话题是否有数据

### 2. 手动检查 TF

```bash
# 查看 TF 树
rosrun tf view_frames
evince frames.pdf

# 实时监控 TF
rosrun tf tf_monitor map base_link

# 查看特定变换
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
```

### 3. 检查时间戳

```bash
# 查看 TF 话题的时间戳
rostopic echo /tf | grep stamp

# 查看位姿话题的时间戳
rostopic echo /orb_slam3/image_pose | grep stamp
```

## 🛠️ 可能的额外修复

### 选项 1：增加 TF 缓存时间

编辑 [`move_base.launch`](../launch/move_base.launch:1)，添加：

```xml
<node pkg="move_base" type="move_base" name="move_base">
  <!-- 增加 TF 缓存时间 -->
  <param name="transform_tolerance" value="0.5"/>  <!-- 默认 0.2 -->
  ...
</node>
```

### 选项 2：调整局部代价地图坐标系

如果问题持续，可以将局部代价地图的坐标系从 `odom` 改为 `map`。

编辑 [`local_costmap_params.yaml`](../params/local_costmap_params.yaml:1)：

```yaml
local_costmap:
  global_frame: map  # 改为 map（原来是 odom）
  robot_base_frame: base_link
  ...
```

**注意**：这会增加计算负担，因为局部地图也需要全局坐标转换。

### 选项 3：使用 robot_localization 包

如果 TF 问题持续，可以使用 `robot_localization` 包来融合多个传感器数据：

```bash
sudo apt-get install ros-noetic-robot-localization
```

然后配置 EKF（扩展卡尔曼滤波器）来平滑位姿估计。

## 📊 验证修复

重启导航系统后，检查：

1. **无错误日志**：
   ```bash
   rosnode info /move_base | grep -i error
   ```

2. **TF 正常**：
   ```bash
   rosrun tf tf_monitor map base_link
   # 应该显示：Frame: base_link, published by: /tf_publisher, Average Delay: 0.001s
   ```

3. **导航正常**：
   - 在 RViz 中设置导航目标
   - 观察全局路径（绿色）和局部路径（红色）是否正常显示
   - 检查是否有速度命令发布：`rostopic echo /cmd_vel`

## 🔄 重启步骤

```bash
# 1. 停止所有节点
Ctrl+C (在所有终端)

# 2. 重新启动 SLAM
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./launch.sh

# 3. 等待 SLAM 初始化（约 10 秒）

# 4. 重新启动导航
cd ~/Desktop/HighTorque_vision/orbslam_depthmaping_ros_2/ros_orbslam_ws
./start_navigation.sh
```

## 📝 常见问题

### Q1: 为什么需要 map 和 odom 两个坐标系？

**A**: 
- `map`：全局固定坐标系，不会漂移
- `odom`：里程计坐标系，会随时间漂移

在视觉 SLAM 中，我们让它们重合（因为视觉定位不漂移）。但 ROS Navigation 标准要求这两个坐标系存在。

### Q2: 如何确认 TF 发布正常？

**A**: 运行以下命令，应该看到连续的输出：
```bash
rostopic hz /tf
# 应该显示：average rate: 20.000
```

### Q3: 如果 ORB-SLAM3 跟踪丢失怎么办？

**A**: TF 发布器会继续发布最后一个已知位姿，并显示警告。需要重新初始化 ORB-SLAM3。

## 📚 参考资料

- [ROS TF Troubleshooting](http://wiki.ros.org/tf/Debugging%20tools)
- [move_base Troubleshooting](http://wiki.ros.org/move_base/Troubleshooting)
- [Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)

---

如果问题仍未解决，请运行诊断工具并提供输出：
```bash
./src/robot_navigation/scripts/diagnose_tf.sh > tf_diagnosis.log 2>&1
```
