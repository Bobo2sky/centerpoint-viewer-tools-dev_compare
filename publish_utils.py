#!/usr/bin/python
# -*- coding:utf8 -*-

from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2 as pcl2
from cv_bridge import CvBridge
import numpy as np
import math
import struct
from builtin_interfaces.msg import Duration

id2type = {
    1: "car",
    2: "truck",
    3: "van",
    4: "bus",
    5: "Pedestrian",
    6: "cyclist",
    7: "Tricyclist",
    8: "Dust",
    9: "Grass",
    10: "Obstacle",
    11: "PVPanel",
    12: "Mound",
    13: "Trailer",
    14: "cone",
    15: "construction_truck",
    16: "bicycle",
    17: "tricycle",
    18: "Vehicle",
    19: "pedestrian",
    20: "motorcyclist_and_bicyclist",
    21: "traffic_cones",
    22: "other",
    23: "spalling",
    24: "Sign",
    25: "Cyclist"
}

type2color = {
    'car': (30, 144, 255),  # Dodger Blue
    'Car': (30, 144, 255),  # Dodger Blue
    'truck': (255, 0, 0),  # Red
    'van': (0, 255, 0),  # Lime
    'bus': (255, 255, 0),  # Yellow
    'Pedestrian': (0, 255, 255),  # Cyan
    'cyclist': (255, 165, 0),  # Orange
    'Tricyclist': (128, 0, 128),  # Purple
    'Dust': (139, 69, 19),  # Saddle Brown
    'Grass': (34, 139, 34),  # Forest Green
    'Obstacle': (128, 128, 128),  # Gray
    'PVPanel': (0, 128, 128),  # Teal
    'Mound': (210, 180, 140),  # Tan
    'Trailer': (70, 130, 180),  # Steel Blue
    'cone': (255, 20, 147),  # Deep Pink
    'construction_truck': (139, 0, 0),  # Dark Red
    'bicycle': (0, 191, 255),  # Deep Sky Blue
    'tricycle': (160, 32, 240),  # Purple
    'Vehicle': (255, 140, 0),  # Dark Orange
    'pedestrian': (220, 20, 60),  # Crimson
    'motorcyclist_and_bicyclist': (0, 100, 0),  # Dark Green
    'traffic_cones': (255, 69, 0),  # Orange Red
    'other': (128, 128, 0),  # Olive
    'spalling': (112, 128, 144),  # Slate Gray
    'Sign': (0, 0, 255),  # Blue
    'Cyclist': (255, 105, 180),  # Hot Pink
    'Stone': (255, 105, 180)  # Hot Pink
}

# FRAME_ID = 'panoramic_view'
FRAME_ID = 'map'
LIFETIME = 0.1
LINES = [[0, 1], [1, 2], [2, 3], [3, 0]]  # lower face
LINES += [[4, 5], [5, 6], [6, 7], [7, 4]]  # upper face
LINES += [[4, 0], [5, 1], [6, 2], [7, 3]]  # connect lower face and upper face

def to_python_float(value):
    """将 numpy float 转换为 Python float"""
    import numpy as np
    if isinstance(value, (np.floating, np.integer)):
        return float(value)
    return float(value)

# 定义标签映射字典
dict_ground = {'Ground': 1}
dict_wall = {'LowWall2': 2, 'LowWall3': 2, 'LowWall': 2, 'ArtificicalWall': 3, 'Mountain': 4, 'Grass': 6,
             'Construction': 21, 'PVPanel': 28}
dict_obj = {'Tree': 5, 'VerticalObstacle': 7, 'StaticObstacle': 8, 'Car': 12, 'Flag_Car': 12, 'Truck': 13,
            'Engineering': 14,
            'Bus': 15, 'Pickup': 16, 'Pedestrian': 17, 'Rider': 18, 'Tricycle': 19, 'Bicycle': 20, 'Trailer': 22,
            'Animal': 24, 'Unknown': 25, 'Stone': 26, 'Mound': 27}
dict_dust = {'Dust': 9, 'Noise': 9, 'Rainwater': 9, 'Wire': 23}

# 合并所有字典
label_dict = {**dict_ground, **dict_wall, **dict_obj, **dict_dust}

color_map = {
    # ===== 基础类别 =====
    0: (128, 128, 128),  # Unlabeled/Background - 灰色

    # ===== 动态物体 (暖色调) =====
    1: (30, 144, 255),  # Car - 道奇蓝
    2: (220, 20, 60),  # Person/Pedestrian - 深红色
    12: (0, 191, 255),  # Bicycle - 深天蓝
    13: (255, 69, 0),  # Motorcycle - 橙红色

    # ===== 道路相关 (紫色/蓝色调) =====
    17: (219, 112, 147),  # Curb - 浅紫红
    18: (147, 112, 219),  # Road - 中紫色
    19: (75, 0, 130),  # LaneMarker - 靛青
    20: (138, 43, 226),  # OtherGround - 蓝紫色
    21: (255, 215, 0),  # Walkable - 金色
    22: (169, 169, 169),  # Sidewalk - 深灰色

    # ===== 静态物体/结构 (棕色/深色调) =====
    23: (210, 105, 30),  # 棕色 (可能是建筑物/墙体)
    24: (255, 20, 147),  # 深粉红 (可能是标志牌)
    26: (184, 134, 11),  # 深金黄 (可能是石头)
    27: (160, 82, 45),  # 赭石色 (可能是土堆)
    28: (0, 128, 128),  # 青色 (可能是光伏板)

    # ===== 自然物体 (绿色调) =====
    5: (144, 238, 144),  # Terrain - 浅绿
    6: (34, 139, 34),  # Vegetation - 森林绿
    7: (0, 255, 127),  # Tree - 春绿
    16: (85, 107, 47),  # TreeTrunk - 暗橄榄绿

    # ===== 交通设施 (亮色) =====
    8: (255, 0, 0),  # Sign - 红色
    9: (255, 255, 0),  # TrafficLight - 黄色
    10: (112, 128, 144),  # Pole - 石板灰
    11: (255, 165, 0),  # ConstructionCone - 橙色
    14: (255, 192, 203),  # Traffic cone - 粉红

    # ===== 建筑物 (深色调) =====
    3: (139, 0, 139),  # Building - 深洋红
    4: (75, 75, 130),  # Wall - 中板岩蓝
    15: (47, 79, 79),  # Fence - 深石板灰

    # ===== 特殊类别 =====
    30: (255, 105, 180),  # 热粉红 (未知类别1)
    31: (64, 224, 208),  # 绿松石 (未知类别2)
    112: (255, 140, 0),  # 深橙色 (特殊标记)

    # ===== 其他 =====
    72: (240, 230, 140),  # 卡其色
}


# 将 RGB 颜色转换为 PCL 格式的浮点数
def rgb_to_pcl_color(r, g, b):
    """将 RGB 颜色转换为 PCL 格式的浮点数"""
    # 确保值在 0-255 范围内
    r = int(np.clip(r, 0, 255))
    g = int(np.clip(g, 0, 255))
    b = int(np.clip(b, 0, 255))

    # 打包成 32 位整数，然后转换为浮点数
    rgb_int = (r << 16) | (g << 8) | b
    return struct.unpack('f', struct.pack('I', rgb_int))[0]


# 可视化点云数据并发布
def publish_point_cloud_seg(pcl_pub, point_cloud, labels, node=None):
    header = Header()
    if node is not None:
        header.stamp = node.get_clock().now().to_msg()
    header.frame_id = "map"

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)
    ]

    points = []
    labels1, counts = np.unique(labels, return_counts=True)
    print("点云标签统计:",labels.shape)
    print("点云数量统计:",point_cloud.shape)

    for i in range(point_cloud.shape[0]):
        x, y, z = point_cloud[i, :3]
        label = labels[i]
        r, g, b = color_map.get(int(label), (255, 255, 255))  # 默认颜色为白色
        rgb = rgb_to_pcl_color(r, g, b)
        points.append([x, y, z, rgb])

    point_cloud_msg = pcl2.create_cloud(header, fields, points)
    pcl_pub.publish(point_cloud_msg)


def publish_point_cloud(pcl_pub, point_clond, node=None):
    header = Header()
    if node is not None:
        header.stamp = node.get_clock().now().to_msg()
    header.frame_id = FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_clond[:, :3]))


def publish_lidar_3dbox(args, box3d_pub, corners_3d_boxes_lidar, types=None, action=None, lifetime=None, rz=None,
                        clear_previous=True, node=None):
    """
    发布3D边界框
    如果 corners_3d_boxes_lidar 为空列表或None，则清除所有边界框
    clear_previous: 是否先清除之前的所有marker
    node: ROS2 节点实例,用于获取时间戳
    """
    marker_array = MarkerArray()

    # 先清除之前的所有marker（如果需要）
    if clear_previous:
        delete_marker = Marker()
        delete_marker.header.frame_id = FRAME_ID
        if node is not None:
            delete_marker.header.stamp = node.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        box3d_pub.publish(marker_array)
        import time
        time.sleep(0.02)  # 给ROS一点时间处理删除操作
        marker_array = MarkerArray()  # 重新创建空的marker_array

    # 如果没有边界框，直接返回（已经清除了）
    if corners_3d_boxes_lidar is None or len(corners_3d_boxes_lidar) == 0:
        return
    print(  "Publishing {} 3D boxes".format(len(corners_3d_boxes_lidar)))
    for i, corners_3d_box in enumerate(corners_3d_boxes_lidar):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        if node is not None:
            marker.header.stamp = node.get_clock().now().to_msg()

        marker.ns = "boxes"  # 设置 namespace
        marker.id = i
        if action is None:
            marker.action = Marker.ADD
        else:
            marker.action = action
        lifetime_val = LIFETIME if lifetime is None else lifetime
        # ROS2 使用 Duration 消息类型
        marker.lifetime = Duration(sec=int(lifetime_val), nanosec=int((lifetime_val % 1) * 1e9))
        marker.type = Marker.LINE_LIST

        # 根据不同类型，颜色不一样
        r, g, b = type2color.get(types[i], (255, 255, 255))  # 添加默认颜色
        marker.color.r = r / 255.
        marker.color.g = g / 255.
        marker.color.b = b / 255.
        marker.color.a = 1.0
        marker.scale.x = 0.3

        marker.points = []
        for l in LINES:
            p1 = corners_3d_box[l[0]]
            point1 = Point()
            # 确保使用 Python 原生 float 类型
            x_val = to_python_float(p1[0])
            y_val = to_python_float(p1[1])
            z_val = to_python_float(p1[2])
            # 验证类型
            assert isinstance(x_val, float), f"x is {type(x_val)}"
            assert isinstance(y_val, float), f"y is {type(y_val)}"
            assert isinstance(z_val, float), f"z is {type(z_val)}"
            point1.x = x_val
            point1.y = y_val
            point1.z = z_val
            marker.points.append(point1)
            p2 = corners_3d_box[l[1]]
            point2 = Point()
            point2.x = to_python_float(p2[0])
            point2.y = to_python_float(p2[1])
            point2.z = to_python_float(p2[2])
            marker.points.append(point2)
        if rz is not None:
            if math.fabs(rz[i]) <= (math.pi / 2.):
                p1 = corners_3d_box[0]
                point1 = Point()
                point1.x = to_python_float(p1[0])
                point1.y = to_python_float(p1[1])
                point1.z = to_python_float(p1[2])
                marker.points.append(point1)
                p2 = corners_3d_box[7]
                point2 = Point()
                point2.x = to_python_float(p2[0])
                point2.y = to_python_float(p2[1])
                point2.z = to_python_float(p2[2])
                marker.points.append(point2)

                p1 = corners_3d_box[3]
                point1 = Point()
                point1.x = to_python_float(p1[0])
                point1.y = to_python_float(p1[1])
                point1.z = to_python_float(p1[2])
                marker.points.append(point1)
                p2 = corners_3d_box[4]
                point2 = Point()
                point2.x = to_python_float(p2[0])
                point2.y = to_python_float(p2[1])
                point2.z = to_python_float(p2[2])
                marker.points.append(point2)
            else:
                p1 = corners_3d_box[1]
                point1 = Point()
                point1.x = to_python_float(p1[0])
                point1.y = to_python_float(p1[1])
                point1.z = to_python_float(p1[2])
                marker.points.append(point1)
                p2 = corners_3d_box[6]
                point2 = Point()
                point2.x = to_python_float(p2[0])
                point2.y = to_python_float(p2[1])
                point2.z = to_python_float(p2[2])
                marker.points.append(point2)

                p1 = corners_3d_box[2]
                point1 = Point()
                point1.x = to_python_float(p1[0])
                point1.y = to_python_float(p1[1])
                point1.z = to_python_float(p1[2])
                marker.points.append(point1)
                p2 = corners_3d_box[5]
                point2 = Point()
                point2.x = to_python_float(p2[0])
                point2.y = to_python_float(p2[1])
                point2.z = to_python_float(p2[2])
                marker.points.append(point2)

        marker_array.markers.append(marker)

        if args.vis_label:
            text_marker = Marker()
            text_marker.header.frame_id = FRAME_ID
            if node is not None:
                text_marker.header.stamp = node.get_clock().now().to_msg()

            text_marker.ns = "labels"  # 设置 namespace 用于标签
            text_marker.id = i + 1000  # i和上面定义一致，保证发布正常显显示
            if action is None:
                text_marker.action = Marker.ADD
            else:
                text_marker.action = action
            text_marker.lifetime = Duration(sec=int(lifetime_val), nanosec=int((lifetime_val % 1) * 1e9))
            text_marker.type = Marker.TEXT_VIEW_FACING  # TEXT表示文字，VIEW_FACING表示一直朝向你观看方向

            # p4 = corners_3d_box[4]#upper front left corner定义设置的marker位置,这里表示上左角
            p4 = np.mean(corners_3d_box, axis=0)  # axis=0表示取的是垂直方向的轴的平均，是的显示在侦测盒中心上方
            text_marker.pose.position.x = float(p4[0])
            text_marker.pose.position.y = float(p4[1])
            text_marker.pose.position.z = float(p4[2]) + 1.0  # 让track_id显示在侦测盒上方
            text_marker.text = str(types[i])

            # 指定marker大小
            text_marker.scale.x = 1.5
            text_marker.scale.y = 1.5
            text_marker.scale.z = 1.5

            b, g, r = type2color.get(types[i], (255, 255, 255))  # 添加默认颜色，track_id文字显示颜色根据物体种类显示
            text_marker.color.r = r / 255.0
            text_marker.color.g = g / 255.0
            text_marker.color.b = b / 255.0
            text_marker.color.a = 1.0
            marker_array.markers.append(text_marker)

    box3d_pub.publish(marker_array)