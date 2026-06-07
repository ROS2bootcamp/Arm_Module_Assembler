"""Unit tests for grasp_planner — pure geometry, no ROS."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ur3_moveit_module import geometry_utils as gu  # noqa: E402
from ur3_moveit_module import grasp_planner as gp  # noqa: E402

GRASP_TF = [0.0, 0.0, 0.13, math.pi, 0.0, 0.0]


def test_grasp_hand_pose_above_object():
    # Cylinder height 0.12 sitting at z=0 base -> center z = 0.06.
    # TCP coincides with center; hand is 0.13 behind TCP along its +Z.
    # With roll=pi the hand +Z points down (-Z world), so hand sits ABOVE.
    plan = gp.plan_grasp(
        object_pose_world=[0.4, 0.1, 0.0, 0.0, 0.0, 0.0],
        grasp_frame_transform=GRASP_TF,
        shape="cylinder",
        dimensions=[0.12, 0.025],
        approach_distance=0.15,
    )
    gx, gy, gz = plan.grasp_hand_pose[:3]
    assert abs(gx - 0.4) < 1e-6
    assert abs(gy - 0.1) < 1e-6
    # hand center = object center (0.06) + 0.13 above = 0.19
    assert abs(gz - 0.19) < 1e-6


def test_pregrasp_is_higher_than_grasp():
    plan = gp.plan_grasp(
        object_pose_world=[0.4, 0.1, 0.0, 0.0, 0.0, 0.0],
        grasp_frame_transform=GRASP_TF,
        shape="cylinder",
        dimensions=[0.12, 0.025],
        approach_distance=0.15,
    )
    # Approach moves hand toward object (downward), so pre-grasp is higher.
    assert plan.pregrasp_hand_pose[2] > plan.grasp_hand_pose[2]
    assert abs((plan.pregrasp_hand_pose[2] - plan.grasp_hand_pose[2]) - 0.15) < 1e-6


def test_grasp_orientation_points_down():
    # hand frame should be rolled ~pi so its local +Z faces -Z world.
    plan = gp.plan_grasp(
        object_pose_world=[0.4, 0.1, 0.0, 0.0, 0.0, 0.0],
        grasp_frame_transform=GRASP_TF,
    )
    tf = gu.pose_list_to_matrix(plan.grasp_hand_pose)
    local_z_in_world = tf[:3, 2]
    assert local_z_in_world[2] < -0.99  # points straight down


def test_place_pose_uses_surface_offset():
    plan = gp.plan_place(
        target_pose_world=[0.6, -0.2, 0.0, 0.0, 0.0, 0.0],
        grasp_frame_transform=GRASP_TF,
        shape="cylinder",
        dimensions=[0.12, 0.025],
        surface_offset=0.001,
        lower_distance=0.1,
    )
    # place hand center = (0.06 + 0.001) + 0.13 = 0.191
    assert abs(plan.grasp_hand_pose[2] - 0.191) < 1e-6
    # pre-place raised by lower_distance
    assert abs((plan.pregrasp_hand_pose[2] - plan.grasp_hand_pose[2]) - 0.1) < 1e-6


def test_box_height_handling():
    plan = gp.plan_grasp(
        object_pose_world=[0.4, 0.0, 0.0, 0.0, 0.0, 0.0],
        grasp_frame_transform=GRASP_TF,
        shape="box",
        dimensions=[0.05, 0.05, 0.2],  # height = 0.2 -> center 0.1
    )
    assert abs(plan.grasp_hand_pose[2] - (0.1 + 0.13)) < 1e-6
