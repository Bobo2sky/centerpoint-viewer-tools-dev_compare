# -*- coding: utf-8 -*-
import numpy as np
import pdb


def check_numpy_to_torch(x):
    # 纯 NumPy 版本，不需要 torch
    if isinstance(x, np.ndarray):
        return x.astype(np.float32), True
    return x, False


def rotate_points_along_z(points, angle):
    """
    Args:
        points: (B, N, 3 + C)
        angle: (B), angle along z-axis, angle increases x ==> y
    Returns:

    """
    points, is_numpy = check_numpy_to_torch(points)
    angle, _ = check_numpy_to_torch(angle)

    cosa = np.cos(angle)
    sina = np.sin(angle)
    zeros = np.zeros(points.shape[0], dtype=np.float32)
    ones = np.ones(points.shape[0], dtype=np.float32)
    rot_matrix = np.stack((
        cosa, sina, zeros,
        -sina, cosa, zeros,
        zeros, zeros, ones
    ), axis=1).reshape(-1, 3, 3).astype(np.float32)
    points_rot = np.matmul(points[:, :, 0:3], rot_matrix)
    points_rot = np.concatenate((points_rot, points[:, :, 3:]), axis=-1)
    return points_rot if is_numpy else points_rot


def box_to_corners_3d(h, w, l, x, y, z, yaw):
    '''
        7 -------- 4
       /|         /|
      6 -------- 5 .
      | |        | |
      . 3 -------- 0
      |/         |/
      2 -------- 1
    Return:3Xn in cam coordinate
    '''
    # 建立旋转矩阵R
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    # 计算8个顶点坐标
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
    # 使用旋转矩阵变换坐标
    corners_3d_cam = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    # 最后在加上中心点
    corners_3d_cam += np.vstack([x, y, z])
    return corners_3d_cam


def boxes_to_corners_3d(boxes3d):
    """
        7 -------- 4
       /|         /|
      6 -------- 5 .
      | |        | |
      . 3 -------- 0
      |/         |/
      2 -------- 1
    Args:
        boxes3d:  (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center

    Returns:
    """
    boxes3d, is_numpy = check_numpy_to_torch(boxes3d)
    
    template = np.array((
        [1, -1, -1],
        [-1, -1, -1],
        [-1, 1, -1],
        [1, 1, -1],
        [1, -1, 1],
        [-1, -1, 1],
        [-1, 1, 1],
        [1, 1, 1],
    ), dtype=np.float32) / 2

    # shape(N,8,3) * shape(1,8,3)
    corners3d = np.repeat(boxes3d[:, None, 3:6], 8, axis=1) * template[None, :, :]
    corners3d = rotate_points_along_z(corners3d.reshape(-1, 8, 3), boxes3d[:, 6]).reshape(-1, 8, 3)
    corners3d += boxes3d[:, None, 0:3]

    return corners3d if is_numpy else corners3d
