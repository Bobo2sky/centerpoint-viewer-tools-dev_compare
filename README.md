# centerpoint-viewer-tools-dev_compare
# Point Cloud Viewer Tools (ROS2 版本)

这是一个点云可视化工具的 ROS2 版本,支持在 RViz2 中实时可视化点云、分割标签和 3D 边界框。

```bash 1.启动可视化系统（已启动）：
./start_visualization.sh
```

```bash 2.在另一个终端中调整标签大小：
# 查看当前标签大小
./set_label_scale.sh

# 缩小标签（50%）
./set_label_scale.sh 0.5

# 正常大小
./set_label_scale.sh 0.8

# 放大标签（150%）
./set_label_scale.sh 1.5

# 超大标签
./set_label_scale.sh 2.0
```


## 主要特性

- ✅ 支持多种点云格式(.bin, .pcd, .npy, .txt)
- ✅ 支持 3D 边界框可视化
- ✅ 支持分割标签可视化
- ✅ 支持多数据源同时显示(如 GT 和预测结果对比)
- ✅ 交互式控制界面
- ✅ 帧缓存优化,提升性能
- ✅ ROS2 兼容

## 环境要求

### 系统要求
- Ubuntu 20.04+ (推荐 22.04)
- ROS2 jazzy/Humble/Foxy/Galactic
- Python 3.8+

### 依赖安装

```bash
# 安装 ROS2 (如果未安装)
# 参考: https://docs.ros.org/en/humble/Installation.html

# 安装 Python 依赖
pip install opencv-python numpy pandas pyyaml

# 安装 ROS2 Python 包
sudo apt install ros-${ROS_DISTRO}-sensor-msgs-py ros-${ROS_DISTRO}-cv-bridge

# 或使用 pip
pip install sensor-msgs-py
```

## 快速开始

### 1. 配置文件

创建或修改 `config.yaml`:

```yaml
data_sources:
  - name: "GT"  # 数据源名称
    data_dir: "/path/to/your/data"  # 数据目录
    ext: ".bin"  # 点云文件扩展名
    point_dim: 4  # 点云维度
    has_score: false  # 标签是否包含 score
    seg_label_suffix: ".label"  # 分割标签后缀
    seg_label_dtype: "int32"  # 分割标签数据类型
    
    # ROS2 话题配置
    pc_topic: "gt_point_cloud"
    seg_topic: "gt_seg_point_cloud"
    box_topic: "gt_bounding_boxes"
    
    # 单色框配置(可选)
    mono_box_topic: "gt_mono_boxes"
    box_color: [0, 1, 0]  # RGB, 范围 0-1

  - name: "Prediction"  # 第二个数据源(可选)
    data_dir: "/path/to/prediction/data"
    ext: ".bin"
    point_dim: 4
    has_score: true
    pc_topic: "pred_point_cloud"
    box_topic: "pred_bounding_boxes"
    mono_box_topic: "pred_mono_boxes"
    box_color: [1, 0, 0]  # 红色

visualization:
  auto_play: false  # 是否自动播放
  delay: 100  # 自动播放延迟(ms)
  vis_seg: false  # 是否显示分割
  vis_label: true  # 是否显示边界框
  save_dir: "output"  # 保存目录
  verbose: true  # 详细输出
```

### 2. 数据目录结构

支持两种目录结构:

#### 方式 1: 扁平结构
```
data_dir/
  ├── frame_0001.bin
  ├── frame_0001.txt  (标签)
  ├── frame_0001.label  (分割标签,可选)
  ├── frame_0002.bin
  └── ...
```

#### 方式 2: 分层结构
```
data_dir/
  ├── bin/
  │   ├── frame_0001.bin
  │   └── frame_0002.bin
  ├── label/
  │   ├── frame_0001.txt
  │   └── frame_0002.txt
  └── seg_label/  (可选)
      ├── frame_0001.label
      └── frame_0002.label
```

### 3. 标签格式

标签文件(`.txt`)格式,每行一个物体:
```
type height width length x y z rz [score]
```

- `type`: 物体类型字符串(如 "car", "pedestrian")
- `height, width, length`: 物体尺寸
- `x, y, z`: 物体中心位置
- `rz`: 绕 Z 轴旋转角度
- `score`: 置信度(可选,需设置 `has_score: true`)

### 4. 运行程序

```bash
# 启动可视化工具
source /opt/ros/humble/setup.bash
python3 vis.py --config config.yaml

# 在另一个终端启动 RViz2
source /opt/ros/humble/setup.bash
rviz2
```

### 5. RViz2 配置

在 RViz2 中添加以下显示:

1. **点云**: Add → PointCloud2
   - Topic: `/gt_point_cloud` 或 `/pred_point_cloud`
   - Fixed Frame: `map`

2. **边界框**: Add → MarkerArray
   - Topic: `/gt_bounding_boxes` 或 `/pred_bounding_boxes`

3. **分割点云**: Add → PointCloud2
   - Topic: `/gt_seg_point_cloud`
   - Color Transformer: `RGB8`

## 控制键

在 OpenCV 窗口中:

- `w`: 下一帧
- `s`: 上一帧
- `Space`: 快进 10 帧
- `a`: 切换自动播放
- `t`: 切换分割显示
- `l`: 切换边界框显示
- `v`: 切换详细输出
- `c`: 保存当前帧
- `q`: 退出

## 高级功能

### 性能优化

代码包含多项性能优化:
- 帧缓存机制,避免重复读取
- 可配置的调试输出
- 高效的点云加载

### 多数据源对比

支持同时显示多个数据源,例如:
- GT(真实标注) vs Prediction(模型预测)
- 不同模型的预测结果对比
- 使用不同颜色区分(通过 `box_color` 和 `mono_box_topic`)

### 自定义物体类型和颜色

在 `publish_utils.py` 中修改:
- `id2type`: 类型 ID 到名称的映射
- `type2color`: 类型到颜色的映射

## 故障排除

### 1. 找不到 ROS2 包

```bash
# 确保已 source ROS2 环境
source /opt/ros/humble/setup.bash

# 检查包是否安装
ros2 pkg list | grep sensor_msgs
```

### 2. 点云不显示

- 检查 Fixed Frame 是否设置为 `map`
- 检查话题名称是否正确
- 使用 `ros2 topic list` 查看可用话题
- 使用 `ros2 topic echo /gt_point_cloud` 检查数据

### 3. 边界框不显示

- 确保标签文件存在且格式正确
- 按 `l` 键切换边界框显示
- 检查 MarkerArray 话题

### 4. sensor_msgs_py 导入错误

```bash
# 方式 1: 使用 apt
sudo apt install ros-${ROS_DISTRO}-sensor-msgs-py

# 方式 2: 使用 pip
pip install sensor-msgs-py

# 如果仍有问题,尝试降级
pip install sensor-msgs-py==1.3.0
```

## 从 ROS1 迁移

如果你有 ROS1 版本的代码,请参考 `ROS2_MIGRATION.md` 了解详细的迁移步骤和修改说明。

主要变化:
- `rospy` → `rclpy`
- 节点创建方式变更
- Publisher 创建方式变更
- 时间戳获取方式变更
- Duration 类型变更

## 许可证

[根据您的项目添加许可证信息]

## 贡献

欢迎提交 Issue 和 Pull Request!

## 参考

- [ROS2 文档](https://docs.ros.org/en/humble/)
- [RViz2 用户指南](https://github.com/ros2/rviz)
