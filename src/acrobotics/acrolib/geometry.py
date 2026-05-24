"""Rotation and homogeneous transformation helpers."""

import numpy as np

from .quaternion import Quaternion


def rot_x(angle):
    return np.array(
        [[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]]
    )


def rot_y(angle):
    return np.array(
        [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]]
    )


def rot_z(angle):
    return np.array(
        [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    )


def translation(x, y, z):
    tf = np.eye(4)
    tf[:3, 3] = [x, y, z]
    return tf


def pose_x(angle, x, y, z):
    tf = translation(x, y, z)
    tf[:3, :3] = rot_x(angle)
    return tf


def pose_y(angle, x, y, z):
    tf = translation(x, y, z)
    tf[:3, :3] = rot_y(angle)
    return tf


def pose_z(angle, x, y, z):
    tf = translation(x, y, z)
    tf[:3, :3] = rot_z(angle)
    return tf


def quat_distance(qa: Quaternion, qb: Quaternion):
    return np.arccos(np.clip(np.abs(qa.elements @ qb.elements), -1.0, 1.0))


def tf_inverse(tf):
    inverse = np.eye(4)
    inverse[:3, :3] = tf[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ tf[:3, 3]
    return inverse


def rpy_to_rot_mat(rxyz):
    return rot_x(rxyz[0]) @ rot_y(rxyz[1]) @ rot_z(rxyz[2])


def rotation_matrix_to_rpy(rotation):
    r11, r12, r13 = rotation[0]
    _, _, r23 = rotation[1]
    _, _, r33 = rotation[2]
    return [
        np.arctan2(-r23, r33),
        np.arctan2(r13, np.sqrt(r11**2 + r12**2)),
        np.arctan2(-r12, r11),
    ]
