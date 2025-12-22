#!/bin/bash
# 启动点云可视化系统
# 在后台运行 vis.py 和 rviz2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 日志文件
VIS_LOG="$SCRIPT_DIR/vis_output.log"
RVIZ_LOG="$SCRIPT_DIR/rviz_output.log"
PID_FILE="$SCRIPT_DIR/.viz_pids"

# 清理函数
cleanup() {
    echo ""
    echo "正在停止可视化系统..."
    
    if [ -f "$PID_FILE" ]; then
        while read pid name; do
            if kill -0 "$pid" 2>/dev/null; then
                echo "  停止 $name (PID: $pid)"
                kill "$pid" 2>/dev/null
            fi
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    
    # 清理可能的子进程
    pkill -P $$ 2>/dev/null
    
    echo "可视化系统已停止"
    exit 0
}

# 注册清理函数
trap cleanup SIGINT SIGTERM EXIT

echo "========================================"
echo "  启动点云可视化系统"
echo "========================================"
echo ""

# 首先清理所有残留的进程
echo "检查并清理残留进程..."
VIS_PIDS=$(pgrep -f "vis.py" 2>/dev/null)
if [ -n "$VIS_PIDS" ]; then
    echo "  清理残留的 vis.py 进程: $VIS_PIDS"
    kill -9 $VIS_PIDS 2>/dev/null
    sleep 1
fi

RVIZ_PIDS=$(pgrep -f "rviz2" 2>/dev/null)
if [ -n "$RVIZ_PIDS" ]; then
    echo "  清理残留的 rviz2 进程: $RVIZ_PIDS"
    kill -9 $RVIZ_PIDS 2>/dev/null
    sleep 1
fi

# 清理旧的日志和 PID 文件
rm -f "$VIS_LOG" "$RVIZ_LOG" "$PID_FILE"

echo "准备就绪！"
echo ""

# 启动点云数据发布器（后台运行）
echo "[1/2] 启动点云数据发布器..."
./run.sh config.yaml > "$VIS_LOG" 2>&1 &
VIS_PID=$!
echo "$VIS_PID vis.py" >> "$PID_FILE"
echo "      PID: $VIS_PID"
echo "      日志: $VIS_LOG"

# 等待 ROS2 节点初始化
echo ""
echo "等待 ROS2 节点初始化..."
sleep 3

# 启动 RViz2（后台运行）
echo ""
echo "[2/2] 启动 RViz2 可视化界面..."
source /opt/ros/jazzy/setup.bash
rviz2 -d centerpoint_vis_ros2.rviz > "$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!
echo "$RVIZ_PID rviz2" >> "$PID_FILE"
echo "      PID: $RVIZ_PID"
echo "      日志: $RVIZ_LOG"

echo ""
echo "========================================"
echo "✓ 可视化系统已启动！"
echo "========================================"
echo ""
echo "进程信息:"
echo "  - 点云发布器: PID $VIS_PID"
echo "  - RViz2:     PID $RVIZ_PID"
echo ""
echo "日志文件:"
echo "  - 点云发布器: tail -f $VIS_LOG"
echo "  - RViz2:     tail -f $RVIZ_LOG"
echo ""
echo "控制说明:"
echo "  - 在 OpenCV 窗口中使用键盘控制:"
echo "    w: 下一帧    s: 上一帧"
echo "    Space: 快进10帧"
echo "    a: 切换自动播放"
echo "    q: 退出"
echo ""
echo "  - 按 Ctrl+C 停止所有进程"
echo "========================================"
echo ""
echo "等待进程运行... (按 Ctrl+C 停止)"
echo ""

# 监控进程状态
while true; do
    # 检查 vis.py 是否还在运行
    if ! kill -0 "$VIS_PID" 2>/dev/null; then
        echo ""
        echo "⚠ 点云发布器已退出"
        break
    fi
    
    # 检查 rviz2 是否还在运行
    if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
        echo ""
        echo "⚠ RViz2 已退出"
        break
    fi
    
    sleep 2
done

# 清理并退出
cleanup
