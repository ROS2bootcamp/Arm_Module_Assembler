"""Gripper (Robotiq 2F-85) control via the MoveItPy 'gripper' planning group.

The SRDF defines named states ``open`` (finger_joint=0.0) and ``close``
(finger_joint=-0.7). We drive the gripper group to these named states and
execute, which routes through the gripper ros2_control controller.
"""

from __future__ import annotations


class GripperControlError(RuntimeError):
    pass


class GripperController:
    def __init__(self, moveit_py, group_name: str,
                 open_pose: str = "open", close_pose: str = "close",
                 logger=None):
        self._moveit = moveit_py
        self._group_name = group_name
        self._open_pose = open_pose
        self._close_pose = close_pose
        self._log = logger or (lambda _m: None)
        self._component = self._moveit.get_planning_component(group_name)

    def _go(self, named_state: str) -> None:
        self._component.set_start_state_to_current_state()
        self._component.set_goal_state(configuration_name=named_state)
        plan_result = self._component.plan()
        if not plan_result:
            raise GripperControlError(f"gripper '{named_state}': planning failed")
        exec_result = self._moveit.execute(plan_result.trajectory, controllers=[])
        if exec_result is not None and hasattr(exec_result, "status"):
            if "SUCCESS" not in str(exec_result.status).upper():
                raise GripperControlError(
                    f"gripper '{named_state}': execution {exec_result.status}")
        self._log(f"gripper -> {named_state}")

    def open(self, named_state: str = None) -> None:
        self._go(named_state or self._open_pose)

    def close(self, named_state: str = None) -> None:
        self._go(named_state or self._close_pose)
