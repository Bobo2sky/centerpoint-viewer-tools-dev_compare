#!/bin/bash

# 最简单的启动脚本 - 使用系统 Python
# 适用于已安装依赖的情况

echo "启动 ROS2 点云可视化..."

# 设置路径，排除 conda
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:$PATH"

# 加载 ROS2 环境
source /opt/ros/jazzy/setup.bash

# 运行程序
/usr/bin/python3 vis.py --config "${1:-config.yaml}"
