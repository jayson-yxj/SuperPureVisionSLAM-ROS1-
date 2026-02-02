#!/bin/bash
# Monster 重力估计包装脚本
# 用于在 conda 环境中运行 gravity_estimator.py

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# GE_information 目录（在 scripts 目录下）
GE_DIR="$(dirname "$SCRIPT_DIR")/GE_information"

# 默认参数
INTERVAL="${INTERVAL:-0.1}"
TARGET_GRAVITY="${1:-0 0 1}"
POSE_JUMP_THRESHOLD="${POSE_JUMP_THRESHOLD:-1.0}"

echo "=========================================="
echo "Monster 重力估计启动脚本"
echo "=========================================="
echo "GE 目录: $GE_DIR"
echo "更新间隔: ${INTERVAL}s"
echo "目标重力: $TARGET_GRAVITY"
echo "位姿跳变阈值: ${POSE_JUMP_THRESHOLD}m"
echo "=========================================="

# 激活 conda 环境并运行 gravity_estimator.py
conda run -n plato python "$SCRIPT_DIR/gravity_estimator.py" \
    --ge_dir "$GE_DIR" \
    --interval "$INTERVAL" \
    --target_gravity $TARGET_GRAVITY \
    --pose_jump_threshold "$POSE_JUMP_THRESHOLD"