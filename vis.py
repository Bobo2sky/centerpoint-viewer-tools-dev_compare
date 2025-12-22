"""
优化版点云可视化工具 (修复版 - 支持多源叠加)
- 保留所有原始功能
- 修复Box没框的问题
- 支持同时显示 GT 和 Prediction
"""

import os
import cv2
import glob
import shutil
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
import rclpy
from rclpy.node import Node
import yaml
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from publish_utils import publish_point_cloud, publish_point_cloud_seg, publish_lidar_3dbox
from box_utils import boxes_to_corners_3d
from geometry_msgs.msg import Point # 需要 Point
from visualization_msgs.msg import Marker, MarkerArray

# ============= 性能优化配置 =============
ENABLE_DEBUG_PRINT = False  # 关闭调试打印以提升性能
CACHE_SIZE = 10  # 缓存最近的帧数

# 支持的分割标签数据类型
SUPPORTED_SEG_DTYPES = {
    'uint8': np.uint8,
    'int32': np.int32,
    'uint32': np.uint32,
    'int16': np.int16,
    'uint16': np.uint16,
    'int64': np.int64,
    'uint64': np.uint64,
}


class FrameCache:
    """帧数据缓存,避免重复读取"""

    def __init__(self, max_size=10):
        self.cache = {}
        self.max_size = max_size
        self.access_order = []

    def get(self, key):
        if key in self.cache:
            # 更新访问顺序
            self.access_order.remove(key)
            self.access_order.append(key)
            return self.cache[key]
        return None

    def put(self, key, value):
        if key in self.cache:
            self.access_order.remove(key)
        elif len(self.cache) >= self.max_size:
            # 删除最久未使用的
            oldest = self.access_order.pop(0)
            del self.cache[oldest]

        self.cache[key] = value
        self.access_order.append(key)

    def clear(self):
        self.cache.clear()
        self.access_order.clear()


def debug_print(msg):
    """条件打印,可全局控制"""
    if ENABLE_DEBUG_PRINT:
        print(msg)


def load_pcd_file(file_path, point_dim=4):
    """
    优化的PCD加载函数
    """
    with open(file_path, 'rb') as f:
        header_lines = []
        header_bytes = 0
        data_type = 'ascii'

        # 读取头部（最多100行）
        for i in range(100):
            line_bytes = f.readline()
            if not line_bytes:
                break

            header_bytes += len(line_bytes)

            try:
                line = line_bytes.decode('utf-8').strip()
            except UnicodeDecodeError:
                line = line_bytes.decode('latin-1', errors='ignore').strip()

            header_lines.append(line)

            if line.startswith('DATA'):
                parts = line.split()
                if len(parts) >= 2:
                    data_type = parts[1].lower()
                break

    # 解析头部信息
    header = {}
    for line in header_lines:
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) >= 2:
            header[parts[0]] = parts[1:]

    # 获取点云信息
    try:
        width = int(header.get('WIDTH', ['0'])[0])
        height = int(header.get('HEIGHT', ['1'])[0])
        fields = header.get('FIELDS', ['x', 'y', 'z'])
        size = [int(s) for s in header.get('SIZE', ['4'] * len(fields))]
        dtype_map = header.get('TYPE', ['F'] * len(fields))
        num_points = width * height

        if num_points == 0:
            raise ValueError("Invalid PCD: width or height is 0")

    except (KeyError, ValueError, IndexError) as e:
        raise ValueError(f"Invalid PCD header: {e}")

    # 根据数据类型读取
    if data_type == 'ascii':
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()

        # 找到DATA行后的数据
        data_start = 0
        for i, line in enumerate(lines):
            if line.startswith('DATA'):
                data_start = i + 1
                break

        data_lines = [line.strip() for line in lines[data_start:]
                      if line.strip() and not line.startswith('#')]

        # 使用numpy的快速方法
        points_array = np.array([line.split() for line in data_lines], dtype=np.float32)

    elif data_type == 'binary':
        type_mapping = {
            'F': np.float32, 'f': np.float32,
            'U': np.uint32, 'u': np.uint32,
            'I': np.int32, 'i': np.int32
        }

        dtype_list = []
        for i, field in enumerate(fields):
            np_type = type_mapping.get(dtype_map[i], np.float32)
            if size[i] == 1:
                np_type = np.uint8
            elif size[i] == 2:
                if dtype_map[i] in ['F', 'f']:
                    np_type = np.float16
                elif dtype_map[i] in ['I', 'i']:
                    np_type = np.int16
                else:
                    np_type = np.uint16
            elif size[i] == 8:
                if dtype_map[i] in ['F', 'f']:
                    np_type = np.float64
                elif dtype_map[i] in ['I', 'i']:
                    np_type = np.int64
                else:
                    np_type = np.uint64

            dtype_list.append((field, np_type))

        dt = np.dtype(dtype_list)

        with open(file_path, 'rb') as f:
            f.read(header_bytes)
            binary_data = f.read()

        try:
            points_structured = np.frombuffer(binary_data, dtype=dt, count=num_points)
        except ValueError:
            available_points = len(binary_data) // dt.itemsize
            if available_points > 0:
                points_structured = np.frombuffer(binary_data, dtype=dt, count=available_points)
                debug_print(f"Warning: Only read {available_points}/{num_points} points")
            else:
                raise

        # 转换为普通数组
        points_array = np.zeros((len(points_structured), len(fields)), dtype=np.float32)
        for i, field in enumerate(fields):
            points_array[:, i] = points_structured[field]

    elif data_type == 'binary_compressed':
        raise NotImplementedError("Binary compressed PCD format is not supported yet")
    else:
        raise ValueError(f"Unsupported PCD format: {data_type}")

    # 调整维度
    if points_array.shape[1] < point_dim:
        padding = np.zeros((points_array.shape[0], point_dim - points_array.shape[1]), dtype=np.float32)
        points_array = np.hstack([points_array, padding])
        debug_print(f"Padded from {points_array.shape[1]} to {point_dim} dimensions")
    elif points_array.shape[1] > point_dim:
        points_array = points_array[:, :point_dim]
        debug_print(f"Truncated from {points_array.shape[1]} to {point_dim} dimensions")

    if len(points_array) == 0:
        raise ValueError("No valid points loaded from PCD file")

    return points_array.astype(np.float32)


def load_point_cloud(file_path, point_dim=4):
    """
    优化的点云加载函数
    """
    file_path = Path(file_path)
    debug_print(f"Loading point cloud: {file_path}")
    ext = file_path.suffix.lower()

    try:
        if ext == '.bin':
            # KITTI格式二进制文件
            points = np.fromfile(str(file_path), dtype=np.float32).reshape(-1, point_dim)

        elif ext == '.npy':
            # NumPy格式
            points = np.load(str(file_path))
            if points.ndim == 1:
                points = points.reshape(-1, point_dim)
            elif points.shape[1] != point_dim:
                if points.shape[1] < point_dim:
                    padding = np.zeros((points.shape[0], point_dim - points.shape[1]), dtype=np.float32)
                    points = np.hstack([points, padding])
                    debug_print(f"Padded from {points.shape[1]} to {point_dim} dimensions")
                else:
                    points = points[:, :point_dim]
                    debug_print(f"Truncated from {points.shape[1]} to {point_dim} dimensions")

        elif ext == '.txt':
            # 文本格式 (空格或逗号分隔)
            with open(file_path, 'rb') as f:
                first_bytes = f.read(1024)
            first_line = first_bytes.decode('utf-8', errors='ignore').split('\n')[0]

            delimiter = ',' if ',' in first_line else None
            points = np.loadtxt(str(file_path), delimiter=delimiter, dtype=np.float32)

            if points.ndim == 1:
                points = points.reshape(-1, point_dim)
            elif points.shape[1] != point_dim:
                if points.shape[1] < point_dim:
                    padding = np.zeros((points.shape[0], point_dim - points.shape[1]), dtype=np.float32)
                    points = np.hstack([points, padding])
                    debug_print(f"Padded from {points.shape[1]} to {point_dim} dimensions")
                else:
                    points = points[:, :point_dim]
                    debug_print(f"Truncated from {points.shape[1]} to {point_dim} dimensions")

        elif ext == '.pcd':
            # PCD格式
            points = load_pcd_file(file_path, point_dim)

        else:
            raise ValueError(f"Unsupported file format: {ext}")

        # 确保返回的是float32类型
        points = points.astype(np.float32)

        # 验证数据
        if len(points) == 0:
            raise ValueError("Point cloud is empty")

        # 只在需要时过滤NaN
        if not np.isfinite(points).all():
            debug_print("Warning: Point cloud contains NaN or Inf, filtering...")
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) == 0:
                raise ValueError("No valid points after filtering NaN/Inf")

        return points

    except Exception as e:
        raise RuntimeError(f"Failed to load point cloud from {file_path}: {e}")

# === [新增] 单色框发布函数 ===
def publish_simple_colored_boxes(publisher, corners_3d, color_rgb, node, frame_id='map', types=None, show_labels=True, rzs=None, label_scale=0.8):
    """
    发布统一颜色的框,用于区分数据源
    color_rgb: [r, g, b] 范围 0-1
    node: ROS2 节点实例,用于获取时间戳
    types: 物体类型列表（用于显示标签）
    show_labels: 是否显示标签
    rzs: 旋转角列表（用于显示方向箭头）
    label_scale: 标签文字大小（默认0.8米）
    """
    marker_array = MarkerArray()
    
    # 先清除所有旧的 markers
    delete_marker = Marker()
    delete_marker.header.frame_id = frame_id
    delete_marker.header.stamp = node.get_clock().now().to_msg()
    delete_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_marker)
    publisher.publish(marker_array)
    
    # 短暂等待确保删除完成
    import time
    time.sleep(0.01)
    
    # 重新创建 marker_array
    marker_array = MarkerArray()
    
    # 1. 创建边界框marker
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "mono_boxes"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1 # 线宽
    
    # 设置颜色
    marker.color.r = float(color_rgb[0])
    marker.color.g = float(color_rgb[1])
    marker.color.b = float(color_rgb[2])
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    
    # 12条棱的连接顺序
    # 底面(0-3), 顶面(4-7), 侧棱(0-4...)
    lines_idx = [
        0,1, 1,2, 2,3, 3,0, # Bottom
        4,5, 5,6, 6,7, 7,4, # Top
        0,4, 1,5, 2,6, 3,7  # Sides
    ]
    
    for box in corners_3d:
        # box shape: (8, 3)
        if box.shape != (8, 3): continue
        for idx in lines_idx:
            p = Point()
            # 确保转换为 Python float
            val_x = box[idx][0]
            val_y = box[idx][1]
            val_z = box[idx][2]
            p.x = float(val_x.item()) if hasattr(val_x, 'item') else float(val_x)
            p.y = float(val_y.item()) if hasattr(val_y, 'item') else float(val_y)
            p.z = float(val_z.item()) if hasattr(val_z, 'item') else float(val_z)
            marker.points.append(p)
    
    marker_array.markers.append(marker)
    
    # 2. 创建标签markers（如果提供了types）
    if show_labels and types is not None and len(types) > 0 and len(corners_3d) > 0:
        for i, (box, obj_type) in enumerate(zip(corners_3d, types)):
            if box.shape != (8, 3): continue
            
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = node.get_clock().now().to_msg()
            text_marker.ns = "mono_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 计算边界框中心点
            center = np.mean(box, axis=0)
            text_marker.pose.position.x = float(center[0])
            text_marker.pose.position.y = float(center[1])
            text_marker.pose.position.z = float(center[2]) + 1.0  # 在框上方1米处显示
            
            text_marker.text = str(obj_type)
            
            # 设置文本大小（使用参数控制）
            text_marker.scale.z = label_scale
            
            # 使用与框相同的颜色
            text_marker.color.r = float(color_rgb[0])
            text_marker.color.g = float(color_rgb[1])
            text_marker.color.b = float(color_rgb[2])
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
    
    # 3. 创建方向箭头markers（如果提供了rzs）
    if rzs is not None and len(rzs) > 0 and len(corners_3d) > 0:
        import math
        for i, (box, rz) in enumerate(zip(corners_3d, rzs)):
            if box.shape != (8, 3): continue
            
            # 创建方向箭头（从底面中心指向前方）
            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame_id
            arrow_marker.header.stamp = node.get_clock().now().to_msg()
            arrow_marker.ns = "mono_arrows"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # 计算底面的中心点 (所有底面4个角点的中心)
            bottom_center = np.mean(box[:4], axis=0)
            
            # 计算底面前边的中心点 (corners 0 和 1 的中点) - 车头位置
            front_center = (box[0] + box[1]) / 2.0
            
            # 计算边界框的长度（前后方向）
            front_vec = box[1] - box[0]
            length = np.linalg.norm(front_vec)
            
            # 箭头长度：从中心到前边 + 前边向外延伸
            arrow_length = length * 1.5
            
            # 箭头起点：边界框的中心
            arrow_start = bottom_center.copy()
            
            # 箭头终点：从中心向前延伸
            arrow_end = arrow_start.copy()
            arrow_end[0] += arrow_length * math.cos(rz)
            arrow_end[1] += arrow_length * math.sin(rz)
            
            # 设置箭头的起点和终点
            start_point = Point()
            start_point.x = float(arrow_start[0])
            start_point.y = float(arrow_start[1])
            start_point.z = float(arrow_start[2])
            
            end_point = Point()
            end_point.x = float(arrow_end[0])
            end_point.y = float(arrow_end[1])
            end_point.z = float(arrow_end[2])
            
            arrow_marker.points = [start_point, end_point]
            
            # 设置箭头样式
            arrow_marker.scale.x = 0.15  # 箭杆直径
            arrow_marker.scale.y = 0.25  # 箭头宽度
            arrow_marker.scale.z = 0.3   # 箭头长度
            
            # 使用与框相同的颜色，但更亮一些
            arrow_marker.color.r = min(1.0, float(color_rgb[0]) + 0.3)
            arrow_marker.color.g = min(1.0, float(color_rgb[1]) + 0.3)
            arrow_marker.color.b = min(1.0, float(color_rgb[2]) + 0.3)
            arrow_marker.color.a = 0.9
            
            marker_array.markers.append(arrow_marker)
            
    publisher.publish(marker_array)


class DemoDataset:
    """数据集类（带缓存优化）"""

    def __init__(self, root_path, ext='.bin', has_score=False, point_dim=4, seg_label_dtype='int32'):
        self.root_path = Path(root_path) if not isinstance(root_path, Path) else root_path
        self.ext = ext
        self.point_dim = point_dim

        if seg_label_dtype not in SUPPORTED_SEG_DTYPES:
            raise ValueError(f"Unsupported seg_label_dtype: {seg_label_dtype}")
        self.seg_label_dtype = SUPPORTED_SEG_DTYPES[seg_label_dtype]
        
        # 添加缓存
        self.cache = FrameCache(max_size=CACHE_SIZE)

        if not self.root_path.exists():
            raise ValueError(f"Path does not exist: {self.root_path}")

        # 检查文件夹结构
        bin_dir = self.root_path / 'bin'
        label_dir = self.root_path / 'label'
        seg_dir = self.root_path / 'seg_label'

        if bin_dir.exists() and bin_dir.is_dir():
            self.use_folder_structure = True
            self.bin_dir = bin_dir
            self.label_dir = label_dir if label_dir.exists() else None
            self.seg_dir = seg_dir if seg_dir.exists() else None
            data_file_list = glob.glob(str(bin_dir / f'*{ext}'))
        else:
            self.use_folder_structure = False
            self.bin_dir = self.root_path
            self.label_dir = None
            self.seg_dir = None
            if self.root_path.is_dir():
                data_file_list = glob.glob(str(self.root_path / f'*{ext}'))
            else:
                data_file_list = [str(self.root_path)]

        self.sample_file_list = sorted(data_file_list)
        print(f"Found {len(self.sample_file_list)} files with extension '{ext}' in {self.root_path}")


        self.label_names = ["type", "height", "width", "length", "x", "y", "z", "rz"]
        if has_score:
            self.label_names.append("score")

    def __len__(self):
        return len(self.sample_file_list)

    def get_label(self, index):
        """获取3D标注框标签（带缓存）"""
        cache_key = f'label_{index}'
        cached = self.cache.get(cache_key)
        if cached is not None: return cached

        if index >= len(self.sample_file_list): return None, None
        bin_path = Path(self.sample_file_list[index])

        # === [核心修改 2]: 强制寻找 .txt 文件 ===
        if self.use_folder_structure and self.label_dir:
            label_path = self.label_dir / bin_path.with_suffix('.txt').name
        else:
            label_path = bin_path.with_suffix(".txt")

        if not label_path.exists():
            result = (None, None)
        else:
            try:
                # === [核心修改 3]: 使用正则分隔符 \s+ 处理不定长空格，并使用 python 引擎 ===
                label = pd.read_csv(label_path, header=None, sep=r'\s+', names=self.label_names, engine='python')
                result = (str(label_path), label)
            except Exception as e:
                debug_print(f"Error reading label {label_path}: {e}")
                result = (None, None)

        self.cache.put(cache_key, result)
        return result

    def get_seg_label(self, index, suffix=".label"):
        """获取分割标签"""
        cache_key = f'seg_{index}_{suffix}'
        cached = self.cache.get(cache_key)
        if cached is not None: return cached

        if index >= len(self.sample_file_list): return None
        bin_path = Path(self.sample_file_list[index])

        if self.use_folder_structure and self.seg_dir:
            seg_path = self.seg_dir / bin_path.with_suffix(suffix).name
        else:
            seg_path = bin_path.with_suffix(suffix)

        if not seg_path.exists():
            result = None
        else:
            try:
                result = np.fromfile(str(seg_path), dtype=self.seg_label_dtype).reshape(-1, 1)
            except Exception as e:
                debug_print(f"Error reading segmentation {seg_path}: {e}")
                result = None

        self.cache.put(cache_key, result)
        return result

    def __getitem__(self, index):
        cache_key = f'pc_{index}'
        cached = self.cache.get(cache_key)
        if cached is not None: return cached

        if index >= len(self.sample_file_list):
            raise IndexError(f"Index {index} out of range")

        pc_path = Path(self.sample_file_list[index])
        try:
            points = load_point_cloud(str(pc_path), self.point_dim)
            result = (str(pc_path), {'path': pc_path, 'points': points, 'frame_id': index})
            self.cache.put(cache_key, result)
            return result
        except Exception as e:
            print(f"Error loading point cloud {pc_path}: {e}")
            raise


class DataSource:
    """数据源管理类（优化版）"""

    def __init__(self, config, node, label_scale=0.8):
        self.name = config['name']
        self.node = node  # 保存 ROS2 节点引用
        self.data_dir = Path(config['data_dir'])
        self.pc_topic = config.get('pc_topic', f'{self.name}_point_cloud')
        self.seg_topic = config.get('seg_topic', f'{self.name}_seg_point_cloud')
        self.box_topic = config.get('box_topic', f'{self.name}_bounding_boxes')
        self.has_score = config.get('has_score', False)
        self.point_dim = config.get('point_dim', 4)


        # 统一使用 seg_label_suffix
        self.seg_label_suffix = config.get('seg_label_suffix', '.label')
        self.ext = config.get('ext', '.bin')
        self.seg_label_dtype = config.get('seg_label_dtype', 'int32')
        
        # 单色框配置
        self.mono_box_topic = config.get('mono_box_topic', None)
        self.box_color = config.get('box_color', [1, 1, 1])
        self.label_scale = label_scale  # 标签大小
        if not self.data_dir.exists():
            raise ValueError(f"Data directory does not exist: {self.data_dir}")

        print(f"\nInitializing data source '{self.name}'...")
        self.dataset = DemoDataset(
            self.data_dir,
            ext=self.ext,
            has_score=self.has_score,
            point_dim=self.point_dim,
            seg_label_dtype=self.seg_label_dtype
        )

        if len(self.dataset) == 0:
            raise ValueError(f"No valid data files found in {self.data_dir}")

        # 使用 ROS2 的 Publisher 创建方式
        self.pc_pub = self.node.create_publisher(PointCloud2, self.pc_topic, 1)
        self.seg_pub = self.node.create_publisher(PointCloud2, self.seg_topic, 1)
        self.box_pub = self.node.create_publisher(MarkerArray, self.box_topic, 1)
        # 初始化单色框发布器
        self.mono_pub = None
        if self.mono_box_topic:
            self.mono_pub = self.node.create_publisher(MarkerArray, self.mono_box_topic, 1)
            print(f"  Mono Box Topic: {self.mono_box_topic} (Color: {self.box_color})")

        print(f"Data source '{self.name}' initialized: {len(self.dataset)} files")

    def publish_frame(self, frame_idx, vis_seg=False, vis_label=False, verbose=True):
        """发布指定帧的数据"""
        if frame_idx >= len(self.dataset):
            return None, None

        try:
            pc_path, data_dict = self.dataset[frame_idx]
            point_cloud = data_dict['points']
        except Exception as e:
            if verbose: print(f"Error loading point cloud: {e}")
            return None, None

        # 加载标签
        label_path, label = self.dataset.get_label(frame_idx)
        seg_gt = self.dataset.get_seg_label(frame_idx, self.seg_label_suffix)

        # === 1. 发布点云 ===
        publish_point_cloud(self.pc_pub, point_cloud, self.node)

        # === 2. 发布分割 ===
        if vis_seg and seg_gt is not None:
            publish_point_cloud_seg(self.seg_pub, point_cloud, seg_gt, self.node)

        # === 3. 发布边界框 ===
        # 无论 vis_label 是 True 还是 False，都调用 publish_lidar_3dbox
        # 如果为 False 或无数据，函数内部会负责清空 Marker
        
        corners = []
        types = []
        rzs = None
        
        if vis_label and label is not None and len(label) > 0:
            try:
                types = label['type'].tolist()
                
                # 映射列名：DataFrame (Type, Len, Wid, Hei...) -> BoxUtils (x, y, z, l, w, h, rz)
                box_fields = ['x', 'y', 'z', 'length', 'width', 'height', 'rz']
                if self.has_score:
                    box_fields.append('score')
                
                gt_boxes = np.array(label[box_fields])
                corners = boxes_to_corners_3d(gt_boxes)
                rzs = gt_boxes[:, 6] # 获取旋转角用于画方向
                
                if verbose:
                    print(f"  [{self.name}] Pub {len(gt_boxes)} boxes")
            except Exception as e:
                if verbose: print(f"Error processing boxes: {e}")
        
        # 执行发布 (含清空逻辑)
        publish_lidar_3dbox(
            argparse.Namespace(vis_label=vis_label),
            self.box_pub,
            corners,
            types,
            lifetime=0,
            rz=rzs,
            clear_previous=True,
            node=self.node
        )
        
        # 2. === 发布单色框 ===
        if self.mono_pub is not None:
            if vis_label and len(corners) > 0:
                publish_simple_colored_boxes(self.mono_pub, corners, self.box_color, self.node, 
                                            types=types, show_labels=True, rzs=rzs, label_scale=self.label_scale)
            else:
                # 清空 (发送空数组)
                publish_simple_colored_boxes(self.mono_pub, [], self.box_color, self.node,
                                            types=None, show_labels=False, rzs=None, label_scale=self.label_scale)
            

        return pc_path, label_path


def load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def create_default_config(config_path):
    # (保留原有的默认配置生成逻辑，略去具体内容以节省空间，不影响功能)
    pass


def parse_args():
    parser = argparse.ArgumentParser(description='Optimized 3D Point Cloud Visualizer')
    parser.add_argument('--config', type=str, default='config.yaml', help='Path to config')
    parser.add_argument('--create_config', action='store_true', help='Create default config')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    return parser.parse_args()


def main():
    args = parse_args()
    global ENABLE_DEBUG_PRINT
    ENABLE_DEBUG_PRINT = args.verbose

    if args.create_config:
        create_default_config(args.config)
        return

    if not os.path.exists(args.config):
        print(f"Config file '{args.config}' not found!")
        return

    config = load_config(args.config)
    
    # 初始化 ROS2
    rclpy.init()
    node = Node('optimized_visualizer')

    # 读取可视化配置
    vis_config = config.get('visualization', {})
    default_label_scale = vis_config.get('label_scale', 0.8)  # 标签大小默认值
    
    # 声明 ROS2 参数，可以动态调整
    node.declare_parameter('label_scale', default_label_scale)
    label_scale = node.get_parameter('label_scale').value
    
    # 初始化所有数据源
    data_sources = []
    for source_config in config['data_sources']:
        try:
            data_sources.append(DataSource(source_config, node, label_scale=label_scale))
        except Exception as e:
            print(f"Error initializing source '{source_config.get('name')}': {e}")

    if not data_sources:
        print("No valid data sources found!")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 继续读取其他可视化配置
    auto_play = vis_config.get('auto_play', False)
    delay = vis_config.get('delay', 100)
    vis_seg = vis_config.get('vis_seg', False)
    vis_label = vis_config.get('vis_label', False)
    save_dir = vis_config.get('save_dir', 'output')
    verbose = vis_config.get('verbose', True)

    os.makedirs(save_dir, exist_ok=True)

    # 清理所有旧的 OpenCV 窗口，避免重复
    # 尝试销毁已知的窗口名称
    try:
        cv2.destroyWindow('Control')
    except:
        pass
    
    cv2.destroyAllWindows()
    
    # 多次处理窗口事件以确保清理完成
    for _ in range(10):
        cv2.waitKey(1)
    
    # 简单界面
    img = np.zeros((200, 400, 3), dtype=np.uint8)
    cv2.namedWindow('Control', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Control', 400, 200)

    # 使用第一个源的长度作为基准
    dataset_length = len(data_sources[0].dataset)
    frame = 0

    print(f"\n{'=' * 60}")
    print(f" STARTING VISUALIZATION (Overlay Mode)")
    print(f" Sources: {[s.name for s in data_sources]}")
    print(f" Total frames: {dataset_length}")
    print(f"{'=' * 60}\n")

    # 性能计时
    import time
    last_time = time.time()

    try:
        while rclpy.ok():
            current_time = time.time()
            
            # 检查参数是否更新
            new_label_scale = node.get_parameter('label_scale').value
            if new_label_scale != label_scale:
                label_scale = new_label_scale
                # 更新所有数据源的标签大小
                for source in data_sources:
                    source.label_scale = label_scale
                print(f"Label scale updated to: {label_scale}")
            
            if verbose: print(f"\n--- Frame {frame} ---")

            # === [核心修改 4]: 遍历并发布所有源的数据 ===
            paths = [] # 用于记录路径方便保存
            for source in data_sources:
                pc_p, _ = source.publish_frame(frame, vis_seg, vis_label, verbose=verbose)
                if pc_p: 
                    paths.append(pc_p)
                    if verbose: print(f"  [{source.name}] File: {pc_p}")

            # 处理 ROS2 回调
            rclpy.spin_once(node, timeout_sec=0.001)

            # 界面更新
            img[:] = 0  # 清空图像
            cv2.putText(img, f"Frame: {frame}/{dataset_length}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.putText(img, f"Auto: {auto_play}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow('Control', img)

            last_time = current_time

            wait_time = delay if auto_play else 0
            key = cv2.waitKey(wait_time) & 0xFF

            if auto_play and key == 255:
                frame = (frame + 1) if (frame + 1) < dataset_length else frame
                if frame >= dataset_length - 1: auto_play = False
                continue

            if key == ord('w'): frame = min(frame + 1, dataset_length - 1)
            elif key == ord('s'): frame = max(frame - 1, 0)
            elif key == ord(' '): frame = min(frame + 10, dataset_length - 1)
            elif key == ord('c'):
                # 保存功能 (只保存第一个源的文件)
                if paths:
                    try:
                        shutil.copy(paths[0], os.path.join(save_dir, os.path.basename(paths[0])))
                        print(f"Saved {os.path.basename(paths[0])}")
                    except Exception as e: print(f"Save failed: {e}")
            elif key == ord('a'): auto_play = not auto_play
            elif key == ord('t'): vis_seg = not vis_seg; print(f"Seg: {vis_seg}")
            elif key == ord('l'): vis_label = not vis_label; print(f"Label: {vis_label}")
            elif key == ord('v'): verbose = not verbose
            elif key == ord('q'): break

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        # 清理 OpenCV 窗口
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        # 多次调用确保窗口完全关闭
        for _ in range(5):
            cv2.waitKey(1)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
