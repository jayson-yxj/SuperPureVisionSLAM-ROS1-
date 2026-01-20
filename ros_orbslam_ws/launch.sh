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
echo "1) 标准模式 (推荐)"
echo "   - 启动所有节点和重力估计"
echo "   - 启用 RViz"
echo "   - 禁用 Open3D 可视化（性能优化）"
echo ""
echo "2) 高性能模式"
echo "   - 启动所有节点"
echo "   - 禁用 RViz"
echo "   - 禁用 Open3D 可视化"
echo ""
echo "3) 调试模式"
echo "   - 启动所有节点"
echo "   - 启用 RViz"
echo "   - 启用 Open3D 可视化"
echo ""
echo "4) 自定义参数"
echo ""
echo "0) 退出"
echo ""
read -p "请选择模式 [1-4]: " choice

case $choice in
    1)
        echo -e "${BLUE}启动标准模式...${NC}"
        echo ""
        catkin_make
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
        echo -e "${BLUE}启动调试模式...${NC}"
        echo -e "${YELLOW}⚠️  警告: Open3D 可视化会降低性能${NC}"
        echo ""
        roslaunch depth_maping slam_mapping.launch \
            enable_rviz:=true \
            enable_visualization:=true \
            enable_gravity_estimate:=true \
            sliding_window_size:=3
        ;;
        
    4)
        echo -e "${BLUE}自定义参数模式${NC}"
        echo ""
        
        # RViz
        read -p "启用 RViz? [y/n] (默认: y): " rviz_choice
        rviz_choice=${rviz_choice:-y}
        if [ "$rviz_choice" = "y" ]; then
            enable_rviz="true"
        else
            enable_rviz="false"
        fi
        
        # Open3D 可视化
        read -p "启用 Open3D 可视化? [y/n] (默认: n): " vis_choice
        vis_choice=${vis_choice:-n}
        if [ "$vis_choice" = "y" ]; then
            enable_vis="true"
        else
            enable_vis="false"
        fi
        
        # 重力估计
        read -p "启用重力估计? [y/n] (默认: y): " gravity_choice
        gravity_choice=${gravity_choice:-y}
        if [ "$gravity_choice" = "y" ]; then
            enable_gravity="true"
        else
            enable_gravity="false"
        fi
        
        # 滑动窗口大小
        read -p "滑动窗口大小 (默认: 3): " window_size
        window_size=${window_size:-3}
        
        echo ""
        echo "启动参数:"
        echo "  - RViz: $enable_rviz"
        echo "  - Open3D 可视化: $enable_vis"
        echo "  - 重力估计: $enable_gravity"
        echo "  - 滑动窗口大小: $window_size"
        echo ""
        
        roslaunch depth_maping slam_mapping.launch \
            enable_rviz:=$enable_rviz \
            enable_visualization:=$enable_vis \
            enable_gravity_estimate:=$enable_gravity \
            sliding_window_size:=$window_size
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