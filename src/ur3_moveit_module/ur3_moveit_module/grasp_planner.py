"""Compute grasp / approach / place poses from object pose + grasp transform.

This mirrors the geometry the PANDA_ENV MTC reference encodes via
``ComputeIK.setIKFrame(tf, hand_frame)`` + ``GenerateGraspPose``, but produces
explicit target poses for the *hand_frame* so the arm can be driven
incrementally (without a monolithic MTC task).

All math is pure numpy so this module is unit-testable without ROS.

Frame convention (matches the reference):
  * ``grasp_frame_transform`` = [x, y, z, roll, pitch, yaw] describes where the
    TCP (grasp frame) sits relative to ``hand_frame``. Default [0,0,0.13,pi,0,0]
    => TCP is 0.13 m forward of hand_frame along +Z, rolled pi so the gripper
    points down for a top-down grasp.
  * The arm is planned so that ``hand_frame`` reaches ``T_world_hand`` where
    ``T_world_hand @ grasp_transform == T_world_tcp`` (TCP coincides with the
    object grasp point).
  * Approach is a straight line along ``hand_frame`` +Z toward the object, so
    the pre-grasp pose is the grasp pose translated back along local -Z.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from . import geometry_utils as gu


@dataclass
class GraspPlan:
    """Target poses (world frame, [x,y,z,r,p,y]) for an incremental pick."""

    pregrasp_hand_pose: List[float]   # arm moves here first (free-space plan)
    grasp_hand_pose: List[float]      # cartesian approach target
    approach_distance: float          # straight-line approach length (m)


def _object_center_pose(object_pose_world: Sequence[float],
                        shape: str,
                        dimensions: Sequence[float]) -> np.ndarray:
    """Return the 4x4 world transform of the object *center*.

    ``object_pose_world`` z is the object base (floor contact); the reference
    corrects cylinders/boxes to their center by adding height/2.
    """
    pose = list(object_pose_world)
    height = _object_height(shape, dimensions)
    pose = pose[:2] + [pose[2] + height / 2.0] + list(pose[3:6])
    return gu.pose_list_to_matrix(pose)


def _object_height(shape: str, dimensions: Sequence[float]) -> float:
    shape = (shape or "cylinder").lower()
    if shape == "cylinder":            # [height, radius]
        return float(dimensions[0])
    if shape == "box":                 # [x, y, z]
        return float(dimensions[2])
    if shape == "sphere":              # [radius]
        return 2.0 * float(dimensions[0])
    # Unknown: assume first dim is height.
    return float(dimensions[0])


def plan_grasp(object_pose_world: Sequence[float],
               grasp_frame_transform: Sequence[float],
               shape: str = "cylinder",
               dimensions: Sequence[float] = (0.12, 0.025),
               approach_distance: float = 0.15) -> GraspPlan:
    """Compute pre-grasp and grasp hand-frame poses for a top-down grasp."""
    t_world_tcp = _object_center_pose(object_pose_world, shape, dimensions)
    grasp_tf = gu.pose_list_to_matrix(list(grasp_frame_transform))

    # hand_frame target: T_world_hand @ grasp_tf == T_world_tcp
    t_world_hand = gu.compose(t_world_tcp, gu.invert(grasp_tf))

    # pre-grasp: back off along hand-frame local -Z (opposite the approach axis)
    t_world_pregrasp = gu.translation_along(t_world_hand, "z", -abs(approach_distance))

    return GraspPlan(
        pregrasp_hand_pose=gu.matrix_to_pose_list(t_world_pregrasp),
        grasp_hand_pose=gu.matrix_to_pose_list(t_world_hand),
        approach_distance=abs(approach_distance),
    )


def plan_place(target_pose_world: Sequence[float],
               grasp_frame_transform: Sequence[float],
               shape: str = "cylinder",
               dimensions: Sequence[float] = (0.12, 0.025),
               surface_offset: float = 0.001,
               lower_distance: float = 0.1) -> GraspPlan:
    """Compute pre-place (above target) and place hand-frame poses.

    ``target_pose_world`` z is the placement surface; the object center sits at
    z + height/2 + surface_offset (matching GeneratePlacePose in the reference).
    Re-uses GraspPlan: pregrasp_hand_pose = pose above target, grasp_hand_pose =
    final lowered pose.
    """
    pose = list(target_pose_world)
    height = _object_height(shape, dimensions)
    center = pose[:2] + [pose[2] + height / 2.0 + surface_offset] + list(pose[3:6])
    t_world_tcp = gu.pose_list_to_matrix(center)
    grasp_tf = gu.pose_list_to_matrix(list(grasp_frame_transform))
    t_world_hand = gu.compose(t_world_tcp, gu.invert(grasp_tf))

    # pre-place: raised by lower_distance along WORLD +Z (lower then drops down)
    t_world_preplace = t_world_hand.copy()
    t_world_preplace[2, 3] += abs(lower_distance)

    return GraspPlan(
        pregrasp_hand_pose=gu.matrix_to_pose_list(t_world_preplace),
        grasp_hand_pose=gu.matrix_to_pose_list(t_world_hand),
        approach_distance=abs(lower_distance),
    )
