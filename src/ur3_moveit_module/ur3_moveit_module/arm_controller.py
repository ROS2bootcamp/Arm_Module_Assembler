"""MoveItPy wrapper for the UR3 arm group.

Provides incremental motion primitives used by the command handlers:
  * plan_to_pose      — free-space plan so a link reaches a world pose
  * plan_to_named     — plan to an SRDF named state (e.g. "home")
  * get_link_pose     — current world pose of a link (from planning scene)
  * relative_move     — straight-ish move by an offset (world or local axis)

ROS / MoveIt are imported lazily inside __init__ so the module can be imported
on hosts without a ROS2 install (the node only constructs this in non-mock
mode). The geometry math reuses geometry_utils.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np

from . import geometry_utils as gu


class ArmControlError(RuntimeError):
    pass


class ArmController:
    def __init__(self,
                 moveit_py,
                 group_name: str,
                 world_frame: str = "world",
                 planning_time: float = 5.0,
                 velocity_scaling: float = 0.3,
                 acceleration_scaling: float = 0.3,
                 logger=None):
        self._moveit = moveit_py
        self._group_name = group_name
        self._world_frame = world_frame
        self._planning_time = planning_time
        self._vel = velocity_scaling
        self._acc = acceleration_scaling
        self._log = logger or (lambda _m: None)

        self._component = self._moveit.get_planning_component(group_name)
        self._robot_model = self._moveit.get_robot_model()

    # ── pose helpers ────────────────────────────────────────────────
    def _make_pose_stamped(self, pose_world: Sequence[float]):
        from geometry_msgs.msg import PoseStamped
        x, y, z, roll, pitch, yaw = pose_world
        qx, qy, qz, qw = gu.rpy_to_quaternion(roll, pitch, yaw)
        ps = PoseStamped()
        ps.header.frame_id = self._world_frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        return ps

    def get_link_pose(self, link: str) -> List[float]:
        """Current world pose [x,y,z,r,p,y] of ``link`` from the planning scene."""
        psm = self._moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            robot_state = scene.current_state
            tf = np.array(robot_state.get_global_link_transform(link))
        return gu.matrix_to_pose_list(tf)

    # ── planning / execution ────────────────────────────────────────
    def _plan_and_execute(self, what: str) -> None:
        plan_result = self._component.plan()
        if not plan_result:
            raise ArmControlError(f"{what}: planning failed (no solution)")
        trajectory = plan_result.trajectory
        exec_result = self._moveit.execute(trajectory, controllers=[])
        # moveit_py.execute returns a result whose .status / bool varies by
        # version; treat falsy / non-SUCCESS as failure.
        if exec_result is not None and hasattr(exec_result, "status"):
            status = str(exec_result.status)
            if "SUCCESS" not in status.upper():
                raise ArmControlError(f"{what}: execution status {status}")
        self._log(f"{what}: ok")

    def plan_to_pose(self, pose_world: Sequence[float], link: str) -> None:
        self._component.set_start_state_to_current_state()
        ps = self._make_pose_stamped(pose_world)
        self._component.set_goal_state(pose_stamped_msg=ps, pose_link=link)
        self._plan_and_execute(f"plan_to_pose({link})")

    def plan_to_named(self, named_state: str) -> None:
        self._component.set_start_state_to_current_state()
        self._component.set_goal_state(configuration_name=named_state)
        self._plan_and_execute(f"plan_to_named({named_state})")

    def relative_move(self, link: str,
                      dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                      local_axis: Optional[str] = None,
                      local_distance: float = 0.0) -> None:
        """Move ``link`` by a world offset (dx,dy,dz) OR along a local axis.

        If ``local_axis`` is given, translate along the link's own axis by
        ``local_distance`` (used for approach/retreat along the gripper Z).
        Otherwise apply the world-frame offset (used for lift / lower).
        """
        current = self.get_link_pose(link)
        tf = gu.pose_list_to_matrix(current)
        if local_axis is not None:
            tf = gu.translation_along(tf, local_axis, local_distance)
        else:
            tf[0, 3] += dx
            tf[1, 3] += dy
            tf[2, 3] += dz
        target = gu.matrix_to_pose_list(tf)
        self.plan_to_pose(target, link)
