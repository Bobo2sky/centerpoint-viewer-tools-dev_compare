#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
重新实现的评估脚本（满足用户要求）：
1) 计算 sweeper_gt/label 与 sweeper_predict/label 中每个目标的全局 mAP 与 NDS（并显示 mATE, mASE, mAOE, mAVE, mAAE）
2) 在此基础上，按平面 (x,y) 欧氏距离分段评估：区间 [0,20), [20,40), [40,60), [60,80)

说明:
- 需要安装 shapely: pip install shapely
- 运行: python3 eval.py
- 输出: output/evaluation_results.json 与 output/terminal_output.txt

实现细节:
- 对每个类别汇总预测（按 score 降序），使用 greedy matching (per-frame) 以 IoU>=0.5 视为匹配。
- 对匹配的对计算 translation error (中心点平面距离)、scale error (相对尺寸误差平均)、orientation error (yaw 差，归一化到 [0,pi])。
- mAVE 和 mAAE 目前设为 0（数据中没有速度与属性信息）。
- mASE 的返回为 mean scale error（0..+inf），在 NDS 计算中使用 1 - min(1, mASE) 来近似 1-IOU 的量级。

"""

import json
import math
import os
from pathlib import Path
from typing import Dict, List, Tuple
from collections import defaultdict
import numpy as np
import sys

try:
    from shapely.geometry import Polygon
except Exception:
    Polygon = None


def load_label_file(file_path: str) -> List[Dict]:
    """加载单帧标签文件，格式：class_name h w l x y z yaw [score]
    如果预测文件没有 score 则使用 1.0
    """
    boxes = []
    if not os.path.exists(file_path):
        return boxes
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 7:
                continue
            box = {
                'class_name': parts[0],
                'h': float(parts[1]),
                'w': float(parts[2]),
                'l': float(parts[3]),
                'x': float(parts[4]),
                'y': float(parts[5]),
                'z': float(parts[6]),
                'yaw': float(parts[7]) if len(parts) > 7 else 0.0,
                'score': float(parts[8]) if len(parts) > 8 else 1.0
            }
            boxes.append(box)
    return boxes


def bev_corners(box: Dict) -> List[Tuple[float, float]]:
    """返回 BEV 的四个角点 (x,y)，按顺时针或逆时针顺序"""
    x, y, yaw = box['x'], box['y'], box.get('yaw', 0.0)
    l, w = box['l'], box['w']
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    corners = []
    for dx, dy in [(-l/2, -w/2), (l/2, -w/2), (l/2, w/2), (-l/2, w/2)]:
        cx = x + dx * cos_y - dy * sin_y
        cy = y + dx * sin_y + dy * cos_y
        corners.append((cx, cy))
    return corners


def iou_bev(box1: Dict, box2: Dict) -> float:
    """计算 BEV IoU，若 shapely 不可用则返回 0.0"""
    if Polygon is None:
        # shapely 未安装，无法计算精确的多边形 IoU
        return 0.0
    try:
        p1 = Polygon(bev_corners(box1))
        p2 = Polygon(bev_corners(box2))
        if not p1.is_valid or not p2.is_valid:
            return 0.0
        inter = p1.intersection(p2).area
        union = p1.area + p2.area - inter
        if union <= 0:
            return 0.0
        return float(inter / union)
    except Exception:
        return 0.0


def center_dist_3d(box1: Dict, box2: Dict) -> float:
    dx = box1['x'] - box2['x']
    dy = box1['y'] - box2['y']
    dz = box1['z'] - box2['z']
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def center_dist_2d(box: Dict) -> float:
    return math.sqrt(box['x']*box['x'] + box['y']*box['y'])


def angle_diff(a: float, b: float) -> float:
    d = abs(a - b) % (2*math.pi)
    if d > math.pi:
        d = 2*math.pi - d
    return d


def compute_ap(precisions: np.ndarray, recalls: np.ndarray) -> float:
    # 11-point interpolation
    ap = 0.0
    for t in np.linspace(0, 1, 11):
        if np.sum(recalls >= t) == 0:
            p = 0.0
        else:
            p = np.max(precisions[recalls >= t])
        ap += p / 11.0
    return float(ap)


def evaluate_class(gt_all: List[List[Dict]], pred_all: List[List[Dict]], class_name: str,
                   iou_thresh: float = 0.5) -> Dict:
    """对单个类别计算 AP 与误差指标（使用 per-frame greedy matching）"""
    # 收集所有预测，带 frame_idx
    preds = []
    for fid, preds_frame in enumerate(pred_all):
        for p in preds_frame:
            if p['class_name'] == class_name:
                preds.append({'frame': fid, 'box': p, 'score': p.get('score', 1.0)})
    preds.sort(key=lambda x: x['score'], reverse=True)

    # 统计 GT 数量
    num_gt = 0
    gt_per_frame = []
    for gt_frame in gt_all:
        gts = [g for g in gt_frame if g['class_name'] == class_name]
        gt_per_frame.append(gts)
        num_gt += len(gts)

    if num_gt == 0:
        # 没有该类别的 GT
        return {
            'ap': 0.0,
            'tp': 0,
            'fp': len(preds),
            'fn': 0,
            'precision': 0.0,
            'recall': 0.0,
            'mATE': None,
            'mASE': None,
            'mAOE': None,
            'mAVE': 0.0,
            'mAAE': 0.0
        }

    tp_flags = np.zeros(len(preds), dtype=int)
    fp_flags = np.zeros(len(preds), dtype=int)

    matched = {i: set() for i in range(len(gt_all))}  # per frame matched gt indices

    translation_errors = []
    scale_errors = []
    orientation_errors = []

    for i, p in enumerate(preds):
        fid = p['frame']
        pbox = p['box']
        best_iou = 0.0
        best_gt_idx = -1
        for gi, gt in enumerate(gt_per_frame[fid]):
            if gi in matched[fid]:
                continue
            iou = iou_bev(pbox, gt)
            if iou > best_iou:
                best_iou = iou
                best_gt_idx = gi
        if best_iou >= iou_thresh and best_gt_idx >= 0:
            # true positive
            tp_flags[i] = 1
            matched[fid].add(best_gt_idx)
            gt_box = gt_per_frame[fid][best_gt_idx]
            # errors
            translation_errors.append(center_dist_3d(pbox, gt_box))
            # scale error: mean relative error of l,w,h
            se = (abs(pbox['l'] - gt_box['l']) / (gt_box['l'] + 1e-6) +
                  abs(pbox['w'] - gt_box['w']) / (gt_box['w'] + 1e-6) +
                  abs(pbox['h'] - gt_box['h']) / (gt_box['h'] + 1e-6)) / 3.0
            scale_errors.append(se)
            orientation_errors.append(angle_diff(pbox.get('yaw', 0.0), gt_box.get('yaw', 0.0)))
        else:
            fp_flags[i] = 1

    tp_cum = np.cumsum(tp_flags)
    fp_cum = np.cumsum(fp_flags)
    precisions = tp_cum / (tp_cum + fp_cum + 1e-10)
    recalls = tp_cum / (num_gt + 1e-10)

    ap = compute_ap(precisions, recalls) if len(precisions) > 0 else 0.0
    total_tp = int(np.sum(tp_flags))
    total_fp = int(np.sum(fp_flags))
    total_fn = num_gt - total_tp
    precision_final = total_tp / (total_tp + total_fp + 1e-10)
    recall_final = total_tp / (num_gt + 1e-10)

    mATE = float(np.mean(translation_errors)) if len(translation_errors) > 0 else None
    mASE = float(np.mean(scale_errors)) if len(scale_errors) > 0 else None
    mAOE = float(np.mean(orientation_errors)) if len(orientation_errors) > 0 else None

    return {
        'ap': ap,
        'tp': total_tp,
        'fp': total_fp,
        'fn': total_fn,
        'precision': precision_final,
        'recall': recall_final,
        'mATE': mATE,
        'mASE': mASE,
        'mAOE': mAOE,
        'mAVE': 0.0,
        'mAAE': 0.0
    }


def calculate_nds_from_class_metrics(class_metrics: Dict[str, Dict]) -> Tuple[float, Dict]:
    """基于各类别指标计算 NDS 和汇总的 mAP, mATE, mASE(1-...), mAOE, mAVE, mAAE"""
    # mAP: 所有类的 ap 平均（包括为0的类）
    aps = [m.get('ap', 0.0) for m in class_metrics.values()]
    mAP = float(np.mean(aps)) if len(aps) > 0 else 0.0

    # mATE: 使用所有类别匹配对的 translation errors 平均；如果没有则设为1.0（较大）
    all_translate = []
    all_scale = []
    all_orient = []
    total_tp = 0
    for m in class_metrics.values():
        # 这里我们没有保留每类的所有 TP 误差序列（上面函数没有返回详细序列），
        # 因此使用 mATE/mASE/mAOE 的均值（如果存在），并对 TP 数进行加权平均
        tp = m['tp']
        total_tp += tp
        if m['mATE'] is not None:
            all_translate.extend([m['mATE']] * tp)
        if m['mASE'] is not None:
            all_scale.extend([m['mASE']] * tp)
        if m['mAOE'] is not None:
            all_orient.extend([m['mAOE']] * tp)

    mATE = float(np.mean(all_translate)) if len(all_translate) > 0 else 1.0
    mASE = float(np.mean(all_scale)) if len(all_scale) > 0 else 1.0
    # 转换成 1 - min(1, mASE) 以接近 mASE(1-IOU) 的尺度
    mASE_iou = 1.0 - min(1.0, mASE)
    mAOE = float(np.mean(all_orient)) if len(all_orient) > 0 else math.pi
    mAOE = min(mAOE, math.pi)
    mAVE = 0.0
    # mAAE: attribute accuracy，目前默认 0
    mAAE = 0.0

    # NDS 近似计算
    nds = (5.0 * mAP + (1.0 - min(1.0, mATE)) + mASE_iou + (1.0 - min(1.0, mAOE / math.pi)) + (1.0 - mAVE) + (1.0 - mAAE)) / 10.0

    return nds, {
        'mAP': mAP,
        'mATE': mATE,
        'mASE': mASE_iou,
        'mAOE': mAOE,
        'mAVE': mAVE,
        'mAAE': mAAE
    }


def filter_by_distance_bins(gt_all: List[List[Dict]], pred_all: List[List[Dict]],
                            bins: List[Tuple[float, float]]) -> Dict[str, Tuple[List[List[Dict]], List[List[Dict]]]]:
    """按平面距离对数据分段，返回每个 bin 的 (gt_list_all_frames, pred_list_all_frames)"""
    results = {}
    for (dmin, dmax) in bins:
        gt_list = []
        pred_list = []
        for gframe, pframe in zip(gt_all, pred_all):
            g_filtered = [g for g in gframe if dmin <= center_dist_2d(g) < dmax]
            p_filtered = [p for p in pframe if dmin <= center_dist_2d(p) < dmax]
            gt_list.append(g_filtered)
            pred_list.append(p_filtered)
        results[f"{dmin}_{dmax}"] = (gt_list, pred_list)
    return results


def evaluate_all(gt_dir: str, pred_dir: str, iou_thresh: float = 0.5) -> Dict:
    # 将输出统一到仓库根目录的 `output/`，保持与用户要求一致
    base = Path(__file__).parent
    out_dir = base / 'output'
    out_dir.mkdir(exist_ok=True)

    gt_files = sorted(list(Path(gt_dir).glob('*.txt')))
    pred_files = sorted(list(Path(pred_dir).glob('*.txt')))
    gt_map = {p.stem: p for p in gt_files}
    pred_map = {p.stem: p for p in pred_files}
    common = sorted(list(set(gt_map.keys()) & set(pred_map.keys())))

    gt_all = []
    pred_all = []
    classes = set()

    for name in common:
        g = load_label_file(str(gt_map[name]))
        p = load_label_file(str(pred_map[name]))
        gt_all.append(g)
        pred_all.append(p)
        for b in g:
            classes.add(b['class_name'])
        for b in p:
            classes.add(b['class_name'])

    classes = sorted(classes)

    # 全局评估
    class_metrics = {}
    for c in classes:
        m = evaluate_class(gt_all, pred_all, c, iou_thresh)
        class_metrics[c] = m

    nds, nds_components = calculate_nds_from_class_metrics(class_metrics)
    overall = {
        'class_metrics': class_metrics,
        'nds': nds,
        'nds_components': nds_components
    }

    # 按距离分段评估
    bins = [(0, 20), (20, 40), (40, 60), (60, 80)]
    binned = filter_by_distance_bins(gt_all, pred_all, bins)
    by_distance = {}
    for k, (g_list, p_list) in binned.items():
        cm = {}
        for c in classes:
            cm[c] = evaluate_class(g_list, p_list, c, iou_thresh)
        nds_k, nds_comp_k = calculate_nds_from_class_metrics(cm)
        by_distance[k] = {'class_metrics': cm, 'nds': nds_k, 'nds_components': nds_comp_k}

    results = {
        'overall': overall,
        'by_distance': by_distance,
        'num_frames': len(common),
        'classes': classes,
        'bins': bins
    }

    # 向后兼容: 构建 by_x_range / x_ranges 字段（格式兼容旧版）
    by_x_range = {}
    for k, v in by_distance.items():
        # k 形如 '0_20'
        parts = k.split('_')
        try:
            x_min = float(parts[0])
            x_max = float(parts[1])
        except Exception:
            x_min, x_max = None, None

        # 计算 total_gt 和 total_pred（sum over classes）
        total_gt = 0
        total_pred = 0
        for cm in v['class_metrics'].values():
            total_gt += (cm.get('tp', 0) + cm.get('fn', 0))
            total_pred += (cm.get('tp', 0) + cm.get('fp', 0))

        # 使用 nds_components 中的 mAP/mATE 等作为汇总指标
        comps = v.get('nds_components', {})
        mAP_bin = float(comps.get('mAP', 0.0))
        mATE_bin = float(comps.get('mATE', 1.0))
        mASE_bin = float(comps.get('mASE', 0.0))
        mAOE_bin = float(comps.get('mAOE', math.pi))

        by_x_range[k] = {
            'x_range': [x_min, x_max],
            'mAP': mAP_bin,
            'NDS': float(v.get('nds', 0.0)),
            'mATE': mATE_bin,
            'mASE': mASE_bin,
            'mAOE': mAOE_bin,
            'mAVE': float(comps.get('mAVE', 0.0)),
            'mAAE': float(comps.get('mAAE', 0.0)),
            'class_metrics': v.get('class_metrics', {}),
            'total_gt': int(total_gt),
            'total_pred': int(total_pred)
        }

    # 将兼容字段加入结果
    results['by_x_range'] = by_x_range
    results['x_ranges'] = bins

    # 保存 JSON
    out_file = out_dir / 'evaluation_results.json'
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    # 生成按距离的可视化图（mAP per bin）
    try:
        import matplotlib.pyplot as plt
        # 按 bins 顺序获取 mAP 与 NDS
        labels = []
        mAP_scores = []
        nds_scores = []
        for (dmin, dmax) in bins:
            key = f"{int(dmin)}_{int(dmax)}"
            labels.append(f"[{int(dmin)},{int(dmax)})")
            entry = results['by_distance'].get(key, {})
            comps = entry.get('nds_components', {})
            mAP_scores.append(float(comps.get('mAP', 0.0)))
            nds_scores.append(float(entry.get('nds', 0.0)))

        x = np.arange(len(labels))
        width = 0.35
        fig, ax1 = plt.subplots(figsize=(8, 4))
        bars = ax1.bar(x - width/2, mAP_scores, width, label='mAP', color='#4ecdc4')
        ax1.set_xlabel('Distance (m)')
        ax1.set_ylabel('mAP')
        ax1.set_ylim(0, 1.0)
        ax1.set_xticks(x)
        ax1.set_xticklabels(labels)

        ax2 = ax1.twinx()
        ax2.plot(x + width/2, nds_scores, color='#ff6b6b', marker='o', label='NDS')
        ax2.set_ylabel('NDS')
        ax2.set_ylim(0, 1.0)

        ax1.set_title('mAP and NDS by Distance Bins')
        fig.legend(loc='upper right')
        plt.tight_layout()
        img_file = out_dir / 'evaluation_by_distance.png'
        plt.savefig(img_file, dpi=200)
        plt.close(fig)
    except Exception as e:
        print('Warning: failed to generate plot:', e)

    # 保存 terminal 输出（简洁版）
    lines = []
    lines.append('='*80)
    lines.append('评估结果（全局）')
    lines.append('='*80)
    lines.append(f"帧数: {len(common)}")
    lines.append(f"类别: {classes}")
    # 全局 NDS 及组成指标
    try:
        overall_nds = overall.get('nds', None)
        overall_comps = overall.get('nds_components', {})
        lines.append('')
        if overall_nds is not None:
            lines.append(f"总体 NDS: {overall_nds:.4f}")
        lines.append(f"mAP: {overall_comps.get('mAP', 0.0):.4f}  mATE: {overall_comps.get('mATE', 0.0):.4f}  mASE: {overall_comps.get('mASE', 0.0):.4f}  mAOE: {overall_comps.get('mAOE', 0.0):.4f}  mAVE: {overall_comps.get('mAVE', 0.0):.4f}  mAAE: {overall_comps.get('mAAE', 0.0):.4f}")
    except Exception:
        pass
    lines.append('')
    lines.append(f"{'类别':<15} {'AP':>8}  {'Precision':>10} {'Recall':>8}  {'TP':>6} {'FP':>6} {'FN':>6} {'mATE':>8} {'mASE':>8} {'mAOE(deg)':>10}")
    lines.append('-'*80)
    for c in classes:
        m = class_metrics[c]
        mAOE_deg = (m['mAOE'] * 180.0 / math.pi) if m['mAOE'] is not None else None
        lines.append(f"{c:<15} {m['ap']:>8.4f}  {m['precision']:>10.4f} {m['recall']:>8.4f}  {m['tp']:>6} {m['fp']:>6} {m['fn']:>6} {str(m['mATE'])[:8]:>8} {str(m['mASE'])[:8]:>8} {str(mAOE_deg)[:10]:>10}")
    lines.append('')
    lines.append('='*80)
    lines.append('按距离分段评估 (平面距离)')
    lines.append('='*80)
    for k, v in by_distance.items():
        # 输出区间标题及 NDS 与组成指标（格式化为多行，便于阅读）
        comps = v.get('nds_components', {})
        lines.append(f"区间 {k} 米:")
        lines.append(f"NDS: {v.get('nds', 0.0):.4f}")
        lines.append(f"mAP: {comps.get('mAP', 0.0):.4f}  mATE: {comps.get('mATE', 0.0):.4f}  mASE: {comps.get('mASE', 0.0):.4f}  mAOE: {comps.get('mAOE', 0.0):.4f}  mAVE: {comps.get('mAVE', 0.0):.4f}  mAAE: {comps.get('mAAE', 0.0):.4f}")
        lines.append(f"  {'类别':<15} {'AP':>8}  {'Precision':>10} {'Recall':>8}  {'TP':>6} {'FP':>6} {'FN':>6} {'mATE':>8} {'mASE':>8} {'mAOE(deg)':>10}")
        for c in classes:
            m = v['class_metrics'][c]
            mAOE_deg = (m['mAOE'] * 180.0 / math.pi) if m['mAOE'] is not None else None
            lines.append(f"  {c:<15} {m['ap']:>8.4f}  {m['precision']:>10.4f} {m['recall']:>8.4f}  {m['tp']:>6} {m['fp']:>6} {m['fn']:>6} {str(m['mATE'])[:8]:>8} {str(m['mASE'])[:8]:>8} {str(mAOE_deg)[:10]:>10}")
        lines.append('-'*60)

    term_file = out_dir / 'terminal_output.txt'
    with open(term_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))

    print('\n'.join(lines))
    print('\n结果已保存到:', out_file, term_file)
    return results


if __name__ == '__main__':
    # 默认路径
    base = Path(__file__).parent
    gt_dir = base / 'sweeper_gt' / 'label'
    pred_dir = base / 'sweeper_predict' / 'label'
    # 运行评估
    evaluate_all(str(gt_dir), str(pred_dir), iou_thresh=0.5)
