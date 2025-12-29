# 目标检测评估程序

## 功能说明

本程序用于评估 3D 目标检测结果，计算以下指标：

- **mAP (mean Average Precision)**: 平均精度均值
- **NDS (nuScenes Detection Score)**: nuScenes 检测分数
- **mATE (mean Average Translation Error)**: 平均平移误差

## 使用方法

### 1. 安装依赖

```bash
./install_eval_deps.sh
```

或手动安装：

```bash
pip install shapely numpy
```

### 2. 运行评估

```bash
python eval.py
```

程序会自动：
- 从 `sweeper_gt/label/` 读取真值标签
- 从 `sweeper_predict/label/` 读取预测标签
- 计算评估指标
- 在终端输出结果
- 将结果保存到 `output/evaluation_results.json`

## 标签格式

标签文件格式（每行一个目标）：

```
class_name h w l x y z yaw [score]
```

参数说明：
- `class_name`: 类别名称（如 Car, Pedestrian, Cyclist, Bus, barrier）
- `h`: 高度（米）
- `w`: 宽度（米）
- `l`: 长度（米）
- `x, y, z`: 3D 中心点坐标（米）
- `yaw`: 偏航角（弧度）
- `score`: 置信度分数（可选，预测结果需要）

## 评估指标说明

### mAP (mean Average Precision)
- 所有类别的平均精度的均值
- 使用 11-point interpolation 方法计算
- IoU 阈值默认为 0.5

### NDS (nuScenes Detection Score)
- nuScenes 数据集的综合评估指标
- 综合考虑 mAP 和定位误差（mATE）
- 计算公式: `NDS = 0.5 * mAP + 0.5 * (1 - mATE)`

### mATE (mean Average Translation Error)
- 成功匹配目标的平均中心点距离误差
- 单位：米
- 越小越好

## 输出结果

### 终端输出示例

```
================================================================================
评估结果
================================================================================

总体指标:
  mAP (mean Average Precision): 0.4196
  NDS (nuScenes Detection Score): 0.6154
  mATE (mean Average Translation Error): 0.1887 米

数据集统计:
  帧数: 935
  类别数: 5
  IoU阈值: 0.5

各类别详细指标:
类别                    AP  Precision   Recall     TP     FP     FN
--------------------------------------------------------------------------------
Bus               0.3616     0.4332   0.6552    133    174     70
Car               0.6679     0.8946   0.7143   7807    920   3122
Cyclist           0.3050     0.5432   0.4644   2503   2105   2887
Pedestrian        0.5246     0.7521   0.6756   1933    637    928
barrier           0.2391     0.4510   0.4779    368    448    402
================================================================================
```

### JSON 输出

结果保存在 `output/evaluation_results.json`，包含：
- 总体指标（mAP, NDS, mATE）
- 各类别详细指标
- TP, FP, FN 统计
- 各距离阈值下的误差分布

## 参数调整

如需修改评估参数，可编辑 `eval.py` 中的 `main()` 函数：

```python
# IoU 阈值
iou_threshold = 0.5

# 距离阈值（用于计算 mATE）
distance_thresholds = [0.5, 1.0, 2.0, 4.0]
```

## 评估流程

1. **加载数据**: 读取 GT 和预测的所有标签文件
2. **匹配检测框**: 基于 IoU 和中心距离匹配 GT 和预测框
3. **计算 AP**: 对每个类别计算 Average Precision
4. **计算 mAP**: 所有类别 AP 的均值
5. **计算误差**: 统计成功匹配目标的定位误差
6. **计算 NDS**: 综合 mAP 和 mATE

## 注意事项

- 确保 GT 和预测文件的文件名一致
- 预测结果应包含置信度分数（最后一列）
- 类别名称必须完全匹配
- 支持的类别: Car, Pedestrian, Cyclist, Bus, barrier

## 依赖库

- numpy: 数值计算
- shapely: 几何计算（IoU）

## 作者

Created for CenterPoint viewer tools evaluation
