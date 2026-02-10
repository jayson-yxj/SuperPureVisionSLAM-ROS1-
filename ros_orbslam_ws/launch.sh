#!/bin/bash
# ROS Launch 快速启动脚本
# 使用 roslaunch 一键启动完整系统

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  ROS Launch 启动脚本"
echo "=========================================="
echo ""

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS 环境未设置${NC}"
    echo "正在加载 ROS 环境..."
    source /opt/ros/noetic/setup.bash
fi

# 进入工作空间目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source workspace
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo -e "${GREEN}✓ ROS workspace 已加载${NC}"
else
    echo -e "${RED}✗ 找不到 devel/setup.bash${NC}"
    echo "请先编译工作空间: catkin_make"
    exit 1
fi

echo ""
echo "=========================================="
echo "  启动选项"
echo "=========================================="
echo ""
echo "1) 单目模式 "
echo "   - 启动所有节点和重力估计"
echo "   - 启用 RViz"
echo "   - 禁用 Open3D 可视化（性能优化）"
echo ""
echo "2) 高性能模式"
echo "   - 启动所有节点"
echo "   - 禁用 RViz"
echo "   - 禁用 Open3D 可视化"
echo ""
echo "3) 双目模式"
echo "   - 启动所有节点"
echo "   - 启用 RViz"
echo ""
echo "4) 双目仿真环境模式（房间环境）"
echo "   - 启动所有节点"
echo "   - 启用 RViz"
echo "   - 启用 gazebo"
echo ""
echo "5) 双目仿真环境模式（书店环境）"
echo "   - 启动所有节点"
echo "   - 启用 RViz"
echo "   - 启用 gazebo"
echo ""
echo "6) 双目仿真环境模式（医院环境）"
echo "   - 启动所有节点"
echo "   - 启用 RViz"
echo "   - 启用 gazebo"
echo ""
echo "0) 退出"
echo ""
read -p "请选择模式 [1-6]: " choice

case $choice in
    1)
        echo -e "${BLUE}启动单目模式...${NC}"
        echo ""
        catkin_make
        source devel/setup.bash
        rosparam set use_sim_time false # 确保使用系统时间
        roslaunch depth_maping slam_mapping.launch \
            enable_rviz:=false \
            enable_visualization:=false \
            sliding_window_size:=3
        ;;
        
    2)
        echo -e "${BLUE}启动高性能模式...${NC}"
        echo ""
        roslaunch depth_maping slam_mapping.launch \
            enable_rviz:=false \
            enable_visualization:=false \
            enable_gravity_estimate:=true \
            sliding_window_size:=2
        ;;
        
    3)
        echo -e "${BLUE}启动双目真实相机模式...${NC}"
        echo ""
        catkin_make
        source devel/setup.bash
        rosparam set use_sim_time false # 确保使用系统时间
        ./src/monster/scripts/gravity_estimate_wrapper.sh &
        roslaunch monster monster_stereo.launch
        ;;
        
    4)
        echo -e "${BLUE}双目仿真环境模式（房间环境）${NC}"
        echo ""
        catkin_make
        source devel/setup.bash
        rosparam set use_sim_time true # 仿真环境使用仿真时间
        ./src/monster/scripts/gravity_estimate_wrapper.sh &
        roslaunch aws_robomaker_small_house_world stereo_robot_with_slam.launch
        pkill -9 python
        ;;

    5)
        echo -e "${BLUE}双目仿真环境模式（书店环境）${NC}"
        echo ""
        catkin_make
        source devel/setup.bash
        rosparam set use_sim_time true # 仿真环境使用仿真时间
        ./src/monster/scripts/gravity_estimate_wrapper.sh &
        roslaunch aws_robomaker_bookstore_world stereo_robot_with_slam.launch
        pkill -9 python
        ;;

    6)
        echo -e "${BLUE}双目仿真环境模式（医院环境）${NC}"
        echo ""
        catkin_make
        source devel/setup.bash
        rosparam set use_sim_time true # 仿真环境使用仿真时间
        ./src/monster/scripts/gravity_estimate_wrapper.sh &
        roslaunch aws_robomaker_hospital_world stereo_robot_with_slam.launch
        pkill -9 python
        ;;
        
    0)
        echo "退出"
        exit 0
        ;;
        
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "  系统已停止"
echo "=========================================="