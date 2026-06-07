"""Pure geometry helpers (no ROS message dependency for the math core).

These are kept import-light so they can be unit-tested on any host without a
ROS2 installation. ROS message construction lives in thin wrappers that import
geometry_msgs lazily.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert XYZ-euler (roll, pitch, yaw) to quaternion (x, y, z, w).

    Matches scipy ``Rotation.from_euler("xyz", ...)`` ordering used by the
    PANDA_ENV reference, implemented here without a scipy dependency so the
    pure-logic layer stays testable everywhere.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    # Intrinsic XYZ: q = qz * qy * qx  (applied right-to-left), giving:
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return 3x3 rotation matrix for intrinsic XYZ euler angles."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_list_to_matrix(pose: Sequence[float]) -> np.ndarray:
    """``[x, y, z, roll, pitch, yaw]`` → 4x4 homogeneous transform."""
    if len(pose) != 6:
        raise ValueError(f"pose must have 6 elements [x,y,z,r,p,y], got {len(pose)}")
    tf = np.eye(4)
    tf[:3, :3] = rpy_to_matrix(pose[3], pose[4], pose[5])
    tf[:3, 3] = pose[:3]
    return tf


def matrix_to_pose_list(tf: np.ndarray) -> List[float]:
    """4x4 homogeneous transform → ``[x, y, z, roll, pitch, yaw]`` (XYZ euler)."""
    x, y, z = tf[:3, 3]
    r = tf[:3, :3]
    # Recover intrinsic XYZ euler angles.
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(r[2, 0]) < 1.0 - 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Compose two 4x4 transforms (a then b in a's frame): a @ b."""
    return a @ b


def invert(tf: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transform."""
    r = tf[:3, :3]
    t = tf[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = r.T
    inv[:3, 3] = -r.T @ t
    return inv


def translation_along(tf: np.ndarray, axis: str, distance: float) -> np.ndarray:
    """Translate a pose along one of its own local axes by ``distance``."""
    axis_idx = {"x": 0, "y": 1, "z": 2}[axis]
    offset = np.eye(4)
    offset[axis_idx, 3] = distance
    return tf @ offset
