#!/bin/bash
# TF 诊断脚本

echo "========================================="
echo "  TF 诊断工具"
echo "========================================="
echo ""

# 检查 TF 发布器是否运行
echo "1. 检查 TF 发布器节点..."
if rosnode list | grep -q "tf_publisher"; then
    echo "✓ TF 发布器正在运行"
else
    echo "✗ TF 发布器未运行！"
    echo "  请启动: roslaunch robot_navigation navigation.launch"
    exit 1
fi

echo ""
echo "2. 检查 TF 树..."
echo "   正在生成 TF 树图..."
rosrun tf view_frames
echo "✓ TF 树已保存到 frames.pdf"

echo ""
echo "3. 检查关键 TF 变换..."

# 检查 map -> odom
echo -n "   map -> odom: "
if timeout 2 rosrun tf tf_echo map odom > /dev/null 2>&1; then
    echo "✓"
else
    echo "✗ 缺失或超时"
fi

# 检查 odom -> base_link
echo -n "   odom -> base_link: "
if timeout 2 rosrun tf tf_echo odom base_link > /dev/null 2>&1; then
    echo "✓"
else
    echo "✗ 缺失或超时"
fi

# 检查 base_link -> camera
echo -n "   base_link -> camera: "
if timeout 2 rosrun tf tf_echo base_link camera > /dev/null 2>&1; then
    echo "✓"
else
    echo "✗ 缺失或超时"
fi

# 检查完整链路 map -> base_link
echo -n "   map -> base_link (完整链路): "
if timeout 2 rosrun tf tf_echo map base_link > /dev/null 2>&1; then
    echo "✓"
else
    echo "✗ 缺失或超时"
fi

echo ""
echo "4. 检查 TF 发布频率..."
echo "   map -> odom:"
timeout 3 rostopic hz /tf | grep "average rate" || echo "   无数据"

echo ""
echo "5. 检查位姿话题..."
echo -n "   /orb_slam3/image_pose: "
if rostopic list | grep -q "/orb_slam3/image_pose"; then
    RATE=$(timeout 3 rostopic hz /orb_slam3/image_pose 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$RATE" ]; then
        echo "✓ ($RATE Hz)"
    else
        echo "⚠ 话题存在但无数据"
    fi
else
    echo "✗ 话题不存在"
fi

echo ""
echo "6. 实时监控 TF (按 Ctrl+C 停止)..."
echo "   监控 map -> base_link 变换..."
rosrun tf tf_monitor map base_link
