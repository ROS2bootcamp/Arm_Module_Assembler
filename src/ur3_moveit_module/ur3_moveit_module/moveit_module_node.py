"""MoveIt Module node — the ROS2 entry point.

Serves the ``/moveit/execute`` service (``llm_agent_msgs/srv/MoveItExecute``)
called by the LLM Agent, executes the requested motion on the UR3 + Robotiq
2F-85 via MoveIt, and returns the result in the service response. Each request
is handled synchronously (plan+execute complete before responding) and serialised
with a lock so two motions are never planned at once. Status is also echoed on
/moveit_status (std_msgs/String JSON) for monitoring.

Transport = service (DECISIONS D15): the contract is owned by LLM_Agent
(MoveItExecute.srv); this module is the server. Request{cmd, params_json} is
flattened to a command dict and dispatched by CommandRouter.

Modes:
  mock=false  -> real MoveItPy motion (requires move_group + Gazebo running)
  mock=true   -> no motion, validates + echoes status (service handshake testing)
"""

from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from llm_agent_msgs.srv import MoveItExecute

from .command_router import CommandRouter
from . import handlers as H


# Parameters declared with their UR3 defaults (overridable via config yaml).
_PARAM_DEFAULTS = {
    "service_name": "/moveit/execute",
    "status_topic": "/moveit_status",
    "log_topic": "/moveit_module/log",
    "mock": False,
    "arm_group_name": "ur_manipulator",
    "hand_group_name": "gripper",
    "eef_name": "robotiq_2f_85",
    "hand_frame": "robotiq_85_tcp",
    "hand_open_pose": "open",
    "hand_close_pose": "close",
    "arm_home_pose": "home",
    "world_frame": "world",
    "surface_link": "table_surface",
    "grasp_frame_transform": [0.0, 0.0, 0.0, 3.1416, 0.0, 0.0],
    "approach_min_dist": 0.08,
    "approach_max_dist": 0.15,
    "lift_min_dist": 0.05,
    "lift_max_dist": 0.15,
    "max_solutions": 8,
    "planning_time_sec": 5.0,
    "cartesian_step_size": 0.01,
    "cartesian_jump_thresh": 0.0,
    "velocity_scaling": 0.3,
    "acceleration_scaling": 0.3,
}


class MoveItModuleNode(Node):
    def __init__(self):
        super().__init__("moveit_module")

        for name, default in _PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self._cfg = {k: self.get_parameter(k).value for k in _PARAM_DEFAULTS}
        self._cb_group = ReentrantCallbackGroup()

        # monitoring publishers
        self._status_pub = self.create_publisher(
            String, self._cfg["status_topic"], 10)
        self._log_pub = self.create_publisher(
            String, self._cfg["log_topic"], 10)

        # one motion at a time
        self._lock = threading.Lock()
        self._router = self._build_router()

        # service server: /moveit/execute (MoveItExecute) — LLM_Agent is client
        self._srv = self.create_service(
            MoveItExecute, self._cfg["service_name"], self._on_execute,
            callback_group=self._cb_group)

        mode = "MOCK" if self._cfg["mock"] else "REAL"
        self.get_logger().info(
            f"MoveIt Module up [{mode}] service={self._cfg['service_name']} "
            f"status={self._cfg['status_topic']} group={self._cfg['arm_group_name']}")

    # ── router construction ─────────────────────────────────────────
    def _build_router(self) -> CommandRouter:
        log = self._publish_log
        if self._cfg["mock"]:
            return CommandRouter(H.build_mock_handlers(logger=log), logger=log)

        try:
            from moveit.planning import MoveItPy
            from .arm_controller import ArmController
            from .gripper_controller import GripperController
            from .scene_manager import SceneManager

            moveit = MoveItPy(node_name="moveit_module_py")
            arm = ArmController(
                moveit, self._cfg["arm_group_name"], self._cfg["world_frame"],
                planning_time=self._cfg["planning_time_sec"],
                velocity_scaling=self._cfg["velocity_scaling"],
                acceleration_scaling=self._cfg["acceleration_scaling"],
                logger=log)
            gripper = GripperController(
                moveit, self._cfg["hand_group_name"],
                self._cfg["hand_open_pose"], self._cfg["hand_close_pose"],
                logger=log)
            scene = SceneManager(self, self._cfg["world_frame"], logger=log)

            handlers = H.Handlers(arm, gripper, scene, self._cfg, logger=log)
            self.get_logger().info("MoveItPy initialised; real motion enabled.")
            return CommandRouter(handlers.as_dict(), logger=log)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"MoveItPy init failed ({exc}); falling back to MOCK handlers.")
            return CommandRouter(H.build_mock_handlers(logger=log), logger=log)

    # ── service callback ────────────────────────────────────────────
    def _on_execute(self, request, response):
        """Handle one MoveItExecute call (plan+execute) and return the result.

        Serialised by ``self._lock`` so two motions never run at once. Returns
        after the motion completes (synchronous), matching the agent's
        call_and_wait semantics.
        """
        with self._lock:
            status = self._router.handle_request(request.cmd, request.params_json)
        self._publish_status(status)
        response.success = bool(status["success"])
        response.error_code = int(status["error_code"])
        response.error_message = str(status["error_message"])
        return response

    # ── publishing ──────────────────────────────────────────────────
    def _publish_status(self, status: dict) -> None:
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        self.get_logger().info(
            f"status cmd={status['cmd_echo']} success={status['success']} "
            f"{status['error_message']}")

    def _publish_log(self, text: str) -> None:
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItModuleNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
