"""Unit tests for geometry_utils — runnable without ROS."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ur3_moveit_module import geometry_utils as gu  # noqa: E402


def test_rpy_to_quaternion_identity():
    x, y, z, w = gu.rpy_to_quaternion(0.0, 0.0, 0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert abs(w - 1.0) < 1e-12


def test_rpy_to_quaternion_roll_pi():
    # roll = pi about x => quaternion (1, 0, 0, 0)
    x, y, z, w = gu.rpy_to_quaternion(math.pi, 0.0, 0.0)
    assert abs(x - 1.0) < 1e-9
    assert abs(y) < 1e-9 and abs(z) < 1e-9 and abs(w) < 1e-9


def test_matrix_roundtrip():
    pose = [0.4, 0.1, 0.3, 0.2, -0.5, 1.1]
    tf = gu.pose_list_to_matrix(pose)
    back = gu.matrix_to_pose_list(tf)
    for a, b in zip(pose, back):
        assert abs(a - b) < 1e-9


def test_invert_is_inverse():
    tf = gu.pose_list_to_matrix([0.4, 0.1, 0.3, 0.2, -0.5, 1.1])
    ident = tf @ gu.invert(tf)
    assert np.allclose(ident, np.eye(4), atol=1e-9)


def test_translation_along_local_z():
    tf = gu.pose_list_to_matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    moved = gu.translation_along(tf, "z", 0.5)
    assert abs(moved[2, 3] - 0.5) < 1e-12


def test_rotation_matrix_orthonormal():
    r = gu.rpy_to_matrix(0.3, -0.7, 1.2)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(r) - 1.0) < 1e-9
