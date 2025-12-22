#!/bin/bash
# 动态调整标签大小的脚本
# 使用方法: ./set_label_scale.sh 1.2

if [ -z "$1" ]; then
    echo "用法: $0 <标签大小>"
    echo "例如: $0 0.5  # 缩小标签"
    echo "      $0 1.0  # 正常大小"
    echo "      $0 1.5  # 放大标签"
    echo ""
    echo "当前标签大小:"
    source /opt/ros/jazzy/setup.bash
    ros2 param get /optimized_visualizer label_scale
    exit 1
fi

LABEL_SCALE=$1

echo "设置标签大小为: $LABEL_SCALE"
source /opt/ros/jazzy/setup.bash
ros2 param set /optimized_visualizer label_scale $LABEL_SCALE

if [ $? -eq 0 ]; then
    echo "✓ 标签大小已更新！"
    echo "当前值:"
    ros2 param get /optimized_visualizer label_scale
else
    echo "✗ 更新失败，请检查可视化程序是否在运行"
fi
