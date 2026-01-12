#!/bin/bash
# 完整系统启动脚本
# 启动 ORB-SLAM3 + Depth Anything V2 + Octomap Mapping

echo "=========================================="
echo "  启动完整 SLAM + 建图系统"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS 环境未设置${NC}"
    echo "请运行: source devel/setup.bash"
    exit 1
fi

# Source workspace
cd "$(dirname "$0")"
source devel/setup.bash
echo -e "${GREEN}✓ ROS workspace 已加载${NC}"

# 检查 roscore
if ! pgrep -x "roscore" > /dev/null; then
    echo -e "${YELLOW}启动 roscore...${NC}"
    roscore &
    sleep 3
fi
echo -e "${GREEN}✓ roscore 正在运行${NC}"

echo ""
echo "=========================================="
echo "  系统架构"
echo "=========================================="
echo ""
echo "  摄像头"
echo "     ↓"
echo "  ORB-SLAM3 (位姿估计)"
echo "     ↓"
echo "  Depth Anything V2 (深度估计)"
echo "     ↓"
echo "  Octomap Mapping (3D 建图)"
echo "     ↓"
echo "  2D 占据地图 (导航)"
echo ""

echo "=========================================="
echo "  启动选项"
echo "=========================================="
echo ""
echo "请选择启动模式："
echo "  1) 启动所有节点（推荐）"
echo "  2) 仅启动 Octomap Mapping"
echo "  3) 启动 Depth + Octomap"
echo "  4) 测试系统（检查话题）"
echo "  5) 启动 RViz 可视化"
echo "  0) 退出"
echo ""
read -p "请输入选项 [1-5]: " choice

case $choice in
    1)
        echo -e "${BLUE}启动完整系统...${NC}"
        echo ""
        
        # 启动 ORB-SLAM3
        echo -e "${YELLOW}[1/4] 启动 ORB-SLAM3...${NC}"
        gnome-terminal -- bash -c "source devel/setup.bash; rosrun orb_slam3_ros orb_mono ..//Vocabulary/ORBvoc.txt ../MonoConfig/Fisheye.yaml; exec bash" &
        sleep 3
        
        # 启动 Depth Mapping
        echo -e "${YELLOW}[2/4] 启动 Depth Mapping...${NC}"
        gnome-terminal -- bash -c "source devel/setup.bash; rosrun depth_maping depth_maping_node.py; exec bash" &
        sleep 2

        # 启动 gravity estimate
        echo -e "${YELLOW}[3/4] 启动 gravity estimate...${NC}"
        gnome-terminal -- bash -c "conda run -n plato python src/depth_maping/scripts/gravity_estimate.py; exec bash" &
        sleep 2
        
        # 启动 RViz
        echo -e "${YELLOW}[4/4] 启动 RViz...${NC}"
        gnome-terminal -- bash -c "source devel/setup.bash; rosrun rviz rviz; exec bash" &
        
        echo ""
        echo -e "${GREEN}✓ 所有节点已启动${NC}"
        ;;
        
    2)
        echo -e "${BLUE}仅启动 Octomap Mapping...${NC}"
        roslaunch octomap_mapping octomap_mapping.launch
        ;;
        
    3)
        echo -e "${BLUE}启动 Depth + Octomap...${NC}"
        echo ""
        
        echo -e "${YELLOW}[1/2] 启动 Depth Mapping...${NC}"
        gnome-terminal -- bash -c "source devel/setup.bash; rosrun depth_maping depth_maping_node.py; exec bash" &
        sleep 2
        
        echo -e "${YELLOW}[2/2] 启动 Octomap Mapping...${NC}"
        gnome-terminal -- bash -c "source devel/setup.bash; roslaunch octomap_mapping octomap_mapping.launch; exec bash" &
        
        echo ""
        echo -e "${GREEN}✓ Depth + Octomap 已启动${NC}"
        ;;
        
    4)
        echo -e "${BLUE}运行系统测试...${NC}"
        ./test_integration.sh
        ;;
        
    5)
        echo -e "${BLUE}启动 RViz...${NC}"
        rosrun rviz rviz
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
echo "  系统已启动"
echo "=========================================="
echo ""
echo "监控命令："
echo "  rostopic list          # 查看所有话题"
echo "  rostopic hz <topic>    # 查看话题频率"
echo "  rostopic echo <topic>  # 查看话题内容"
echo "  rosnode list           # 查看运行的节点"
echo ""
echo "关键话题："
echo "  /orb_slam3/image_pose      # ORB-SLAM3 位姿"
echo "  /depth_anything/depth_image # 深度图"
echo "  /octomap_2d                # 2D 占据地图"
echo "  /octomap_pointcloud        # 3D 点云"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo ""

# 等待用户中断
wait