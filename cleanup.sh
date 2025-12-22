#!/bin/bash
# 清理所有可视化相关的进程

echo "正在查找并清理可视化进程..."

# 查找并终止 vis.py 进程
VIS_PIDS=$(pgrep -f "vis.py")
if [ -n "$VIS_PIDS" ]; then
    echo "终止 vis.py 进程: $VIS_PIDS"
    kill -9 $VIS_PIDS 2>/dev/null
else
    echo "未找到 vis.py 进程"
fi

# 查找并终止 rviz2 进程
RVIZ_PIDS=$(pgrep -f "rviz2")
if [ -n "$RVIZ_PIDS" ]; then
    echo "终止 rviz2 进程: $RVIZ_PIDS"
    kill -9 $RVIZ_PIDS 2>/dev/null
else
    echo "未找到 rviz2 进程"
fi

# 清理 PID 文件
if [ -f ".viz_pids" ]; then
    echo "清理 PID 文件"
    rm -f .viz_pids
fi

echo "清理完成！"
