# evaluation_results.json 文件结构说明

## 概述

`evaluation_results.json` 文件包含了评估程序的所有输出信息，既有原始数值数据，也有格式化的文本输出。

## JSON 文件结构

```json
{
  // ========== 原始数值数据 ==========
  "mAP": 0.4196,                    // 平均精度均值
  "NDS": 0.6755,                    // nuScenes检测分数
  "mATE": 0.1887,                   // 平均平移误差（米）
  "mASE": 0.9389,                   // 平均尺度误差（1-IOU）
  "mAOE": 0.2918,                   // 平均方向误差（弧度）
  "mAVE": 0.0,                      // 平均速度误差（m/s）
  "mAAE": 0.0,                      // 平均属性误差（1-acc）
  
  // ========== 各类别详细指标 ==========
  "class_metrics": {
    "Bus": {
      "ap": 0.3616,
      "tp": 133,
      "fp": 174,
      "fn": 70,
      "precision": 0.4332,
      "recall": 0.6552
    },
    "Car": { ... },
    ...
  },
  
  // ========== 数据集统计 ==========
  "num_frames": 935,
  "num_classes": 5,
  "iou_threshold": 0.5,
  
  // ========== 格式化输出 ==========
  "formatted_output": {
    // 完整的文本输出（与终端显示完全一致）
    "full_text": "...",
    
    // 结构化的输出信息
    "sections": {
      "overall_metrics": {
        "mAP": "0.4196",
        "mAP_description": "mean Average Precision",
        "NDS": "0.6755",
        ...
      },
      "notes": {
        "mAAE_explanation": "...",
        "implementation_note": "..."
      },
      "dataset_statistics": { ... },
      "per_class_details": [ ... ]
    }
  }
}
```

## 使用方式

### 1. 直接读取完整的格式化文本

```python
import json

with open('output/evaluation_results.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

# 打印完整的评估结果（与终端输出完全一致）
print(data['formatted_output']['full_text'])
```

### 2. 访问结构化的数据

```python
import json

with open('output/evaluation_results.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

# 获取总体指标
sections = data['formatted_output']['sections']
metrics = sections['overall_metrics']

print(f"mAP: {metrics['mAP']}")
print(f"NDS: {metrics['NDS']}")

# 获取各类别详细信息
for item in sections['per_class_details']:
    print(f"{item['class_name']}: AP={item['AP']}, Precision={item['Precision']}")
```

### 3. 访问原始数值（用于进一步计算）

```python
import json

with open('output/evaluation_results.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

# 获取高精度的原始数值
mAP = data['mAP']  # 0.4196473245231668
NDS = data['NDS']  # 0.6755449810715917

# 获取各类别原始数据
for class_name, metrics in data['class_metrics'].items():
    print(f"{class_name}: AP={metrics['ap']}, TP={metrics['tp']}")
```

### 4. 使用提供的脚本

```bash
# 运行 display_results.py 查看评估结果
python display_results.py

# 或指定JSON文件路径
python display_results.py path/to/evaluation_results.json
```

## 字段说明

### 总体指标

| 字段 | 含义 | 单位 | 取值范围 | 越大越好 |
|------|------|------|----------|----------|
| mAP | 平均精度均值 | - | [0, 1] | ✓ |
| NDS | nuScenes检测分数 | - | [0, 1] | ✓ |
| mATE | 平均平移误差 | 米 | [0, ∞) | ✗ |
| mASE | 平均尺度误差(1-IOU) | - | [0, 1] | ✓ |
| mAOE | 平均方向误差 | 弧度 | [0, π] | ✗ |
| mAVE | 平均速度误差 | m/s | [0, ∞) | ✗ |
| mAAE | 平均属性误差(1-acc) | - | [0, 1] | ✗ |

### 各类别指标

| 字段 | 含义 |
|------|------|
| ap | Average Precision（平均精度） |
| tp | True Positive（真阳性） |
| fp | False Positive（假阳性） |
| fn | False Negative（假阴性） |
| precision | 精确率 = TP / (TP + FP) |
| recall | 召回率 = TP / (TP + FN) |

## 注意事项

1. **tp_errors 已删除**: 为减小文件大小，详细的误差列表（tp_errors, scale_errors, orientation_errors）已从JSON中删除
2. **双重格式**: 同时提供原始数值和格式化文本，方便不同场景使用
3. **编码**: 文件使用 UTF-8 编码，支持中文
4. **精度**: 原始数值保留完整精度，格式化文本保留4位小数

## 文件大小

- 优化前（包含所有误差列表）: ~3MB+
- 优化后（删除误差列表）: ~4.6KB
- 减少了 99%+ 的文件大小

## 示例输出

```
================================================================================
评估结果
================================================================================

总体指标:
  mAP (mean Average Precision):        0.4196
  NDS (nuScenes Detection Score):      0.6755
  mATE (mean Translation Error):       0.1887 米
  mASE (mean Scale Error, 1-IOU):      0.9389
  mAOE (mean Orientation Error):       0.2918 弧度 (16.72°)
  mAVE (mean Velocity Error):          0.0000 m/s
  mAAE (mean Attribute Error):         0.0000 (1-acc)

指标说明:
  * mAAE = 0.0000 表示类别分类准确率为 100.00%
  * 当前实现中，检测框只与同类别GT匹配，因此类别准确率为100%

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
