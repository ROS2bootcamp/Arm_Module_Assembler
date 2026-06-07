"""Unit tests for CommandRouter with mock handlers — no ROS."""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ur3_moveit_module.command_router import (  # noqa: E402
    CommandRouter, HandlerResult, ERR_BAD_JSON, ERR_UNKNOWN_CMD,
    ERR_MISSING_FIELD, ERR_OK,
)


def _router(record=None):
    def make(name):
        def h(cmd):
            if record is not None:
                record.append((name, cmd))
            return HandlerResult.ok()
        return h
    handlers = {c: make(c) for c in ("scan", "pick", "lift", "place", "release", "home")}
    return CommandRouter(handlers), handlers


def test_successful_dispatch_and_phase():
    record = []
    router, _ = _router(record)
    status = router.handle(json.dumps({"cmd": "pick", "object": {"name": "x"}}))
    assert status["success"] is True
    assert status["cmd_echo"] == "pick"
    assert status["phase"] == "P2"
    assert status["error_code"] == ERR_OK
    assert "timestamp" in status
    assert record == [("pick", {"cmd": "pick", "object": {"name": "x"}})]


def test_phase_map_all_commands():
    router, _ = _router()
    expected = {"scan": "P1", "pick": "P2", "lift": "P3",
                "place": "P4", "release": "P4", "home": "P4"}
    for cmd, phase in expected.items():
        assert router.handle(json.dumps({"cmd": cmd}))["phase"] == phase


def test_invalid_json():
    router, _ = _router()
    status = router.handle("{not valid json")
    assert status["success"] is False
    assert status["error_code"] == ERR_BAD_JSON


def test_missing_cmd():
    router, _ = _router()
    status = router.handle(json.dumps({"foo": "bar"}))
    assert status["success"] is False
    assert status["error_code"] == ERR_MISSING_FIELD


def test_unknown_cmd():
    router, _ = _router()
    status = router.handle(json.dumps({"cmd": "dance"}))
    assert status["success"] is False
    assert status["error_code"] == ERR_UNKNOWN_CMD
    assert status["cmd_echo"] == "dance"


def test_handler_exception_is_isolated():
    def boom(_cmd):
        raise RuntimeError("planner exploded")
    router = CommandRouter({"pick": boom})
    status = router.handle(json.dumps({"cmd": "pick"}))
    assert status["success"] is False
    assert "planner exploded" in status["error_message"]


def test_handler_returning_failure():
    router = CommandRouter({"home": lambda c: HandlerResult.fail("no IK", 99)})
    status = router.handle(json.dumps({"cmd": "home"}))
    assert status["success"] is False
    assert status["error_code"] == 99
    assert status["error_message"] == "no IK"


# ── service path: handle_request (MoveItExecute Request) ──────────────

def test_handle_request_merges_cmd_and_params():
    record = []
    router, _ = _router(record)
    status = router.handle_request(
        "place", json.dumps({"target_pose_world": [0.4, -0.2, 0.06, 0, 0, 0]}))
    assert status["success"] is True
    assert status["cmd_echo"] == "place"
    assert status["phase"] == "P4"
    assert record[-1] == (
        "place", {"target_pose_world": [0.4, -0.2, 0.06, 0, 0, 0], "cmd": "place"})


def test_handle_request_strips_robot_fixed_keys():
    record = []
    router, _ = _router(record)
    # agent sends (stale) robot-fixed values; server must drop them so config wins
    router.handle_request("pick", json.dumps({
        "hand_frame": "robotiq_2f_85_tcp",          # stripped (D16)
        "grasp_frame_transform": [0, 0, 0.13, 3.14, 0, 0],  # stripped
        "arm_group_name": "panda_arm",              # stripped
        "object": {"name": "x", "pose_world": [0] * 6},     # kept (dynamic)
    }))
    _, passed = record[-1]
    assert "hand_frame" not in passed
    assert "grasp_frame_transform" not in passed
    assert "arm_group_name" not in passed
    assert passed["object"] == {"name": "x", "pose_world": [0] * 6}
    assert passed["cmd"] == "pick"


def test_handle_request_empty_params():
    record = []
    router, _ = _router(record)
    status = router.handle_request("home", "")
    assert status["success"] is True
    assert record[-1] == ("home", {"cmd": "home"})


def test_handle_request_bad_params_json():
    router, _ = _router()
    status = router.handle_request("pick", "{not json")
    assert status["success"] is False
    assert status["error_code"] == ERR_BAD_JSON
    assert status["cmd_echo"] == "pick"


def test_handle_request_non_object_params():
    router, _ = _router()
    status = router.handle_request("pick", "[1, 2, 3]")
    assert status["success"] is False
    assert status["error_code"] == ERR_BAD_JSON
