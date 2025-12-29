#!/bin/bash

# ROS2 依赖安装脚本
# 为系统 Python 安装所需依赖

set -e  # 遇到错误立即退出

echo "=========================================="
echo "ROS2 点云可视化工具 - 依赖安装"
echo "=========================================="
echo ""

# 检查是否为 root
if [ "$EUID" -eq 0 ]; then 
    echo "警告: 请不要使用 sudo 运行此脚本"
    echo "脚本会在需要时提示输入密码"
    exit 1
fi

# 1. 安装 Python 包
echo "步骤 1/2: 安装 Python 依赖包..."
echo "-------------------------------------------"

PYTHON_PACKAGES="pandas numpy opencv-python pyyaml"

echo "将要安装: $PYTHON_PACKAGES"
echo "使用清华源加速..."
pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple $PYTHON_PACKAGES

echo ""
echo "✓ Python 包安装完成"
echo ""

# 2. 安装 ROS2 包
echo "步骤 2/2: 安装 ROS2 相关包..."
echo "-------------------------------------------"

# 检测 ROS 版本
if [ -z "$ROS_DISTRO" ]; then
    # 尝试自动检测
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        ROS_DISTRO="jazzy"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        ROS_DISTRO="humble"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        ROS_DISTRO="foxy"
    else
        echo "错误: 未检测到 ROS2 安装"
        echo "请先安装 ROS2 或运行: source /opt/ros/<distro>/setup.bash"
        exit 1
    fi
    echo "检测到 ROS2 版本: $ROS_DISTRO"
else
    echo "使用当前 ROS2 版本: $ROS_DISTRO"
fi

ROS_PACKAGES="ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-cv-bridge"

echo "将要安装: $ROS_PACKAGES"
echo "需要 sudo 权限..."

sudo apt update
sudo apt install -y $ROS_PACKAGES

echo ""
echo "✓ ROS2 包安装完成"
echo ""

# 3. 验证安装
echo "验证安装..."
echo "-------------------------------------------"

SYSTEM_PYTHON="/usr/bin/python3"

echo "Python 版本: $($SYSTEM_PYTHON --version)"
echo ""

DEPS_OK=true

# 检查 Python 包
for pkg in pandas numpy cv2 yaml; do
    if $SYSTEM_PYTHON -c "import $pkg" 2>/dev/null; then
        echo "✓ $pkg"
    else
        echo "✗ $pkg (未安装)"
        DEPS_OK=false
    fi
done

# 检查 ROS2 包
source /opt/ros/$ROS_DISTRO/setup.bash
for pkg in rclpy sensor_msgs_py; do
    if $SYSTEM_PYTHON -c "import ${pkg//-/_}" 2>/dev/null; then
        echo "✓ $pkg"
    else
        echo "✗ $pkg (未安装)"
        DEPS_OK=false
    fi
done

echo ""
echo "=========================================="

if [ "$DEPS_OK" = true ]; then
    echo "✓ 所有依赖已成功安装!"
    echo ""
    echo "现在可以运行:"
    echo "  ./run_ros2.sh config.yaml"
    echo ""
    echo "或者手动运行:"
    echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
    echo "  /usr/bin/python3 vis.py --config config.yaml"
else
    echo "✗ 部分依赖安装失败，请检查错误信息"
    exit 1
fi

echo "=========================================="

pip install matplotlib
