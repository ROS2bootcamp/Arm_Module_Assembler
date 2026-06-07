"""Tests for Handlers using fake controllers (no ROS / MoveIt needed).

Verifies the decomposed pick/lift/place/release/home sequences issue the right
controller calls in the right order, and that the held-object state persists
across commands.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ur3_moveit_module import handlers as H  # noqa: E402

DEFAULTS = {
    "hand_frame": "robotiq_85_tcp",
    "grasp_frame_transform": [0.0, 0.0, 0.0, math.pi, 0.0, 0.0],
    "approach_max_dist": 0.15,
    "approach_min_dist": 0.08,
    "arm_home_pose": "home",
    "hand_open_pose": "open",
    "hand_close_pose": "close",
    "world_frame": "world",
}


class FakeArm:
    def __init__(self):
        self.calls = []

    def plan_to_pose(self, pose, link):
        self.calls.append(("plan_to_pose", list(pose), link))

    def plan_to_named(self, name):
        self.calls.append(("plan_to_named", name))

    def relative_move(self, link, dx=0.0, dy=0.0, dz=0.0,
                      local_axis=None, local_distance=0.0):
        self.calls.append(("relative_move", link, dx, dy, dz,
                           local_axis, local_distance))


class FakeGripper:
    def __init__(self):
        self.calls = []

    def open(self, named_state=None):
        self.calls.append(("open", named_state))

    def close(self, named_state=None):
        self.calls.append(("close", named_state))


class FakeScene:
    def __init__(self):
        self.calls = []

    def add_object(self, name, shape, dims, pose):
        self.calls.append(("add", name, shape, list(dims), list(pose)))

    def attach(self, name, link, touch_links=None):
        self.calls.append(("attach", name, link))

    def detach(self, name, link):
        self.calls.append(("detach", name, link))


def _make():
    arm, grip, scene = FakeArm(), FakeGripper(), FakeScene()
    return H.Handlers(arm, grip, scene, DEFAULTS), arm, grip, scene


def test_scan_visits_all_waypoints():
    h, arm, _, _ = _make()
    wps = [[0.3, 0.0, 0.5, 0.0, 0.785, 0.0], [0.3, 0.3, 0.5, 0.0, 0.785, 0.3]]
    res = h.scan({"cmd": "scan", "waypoints": wps})
    assert res.success
    assert sum(1 for c in arm.calls if c[0] == "plan_to_pose") == 2


def test_scan_rejects_bad_waypoint():
    h, _, _, _ = _make()
    res = h.scan({"cmd": "scan", "waypoints": [[0.1, 0.2]]})
    assert not res.success


def test_pick_sequence_order():
    h, arm, grip, scene = _make()
    res = h.pick({
        "cmd": "pick",
        "object": {"name": "target_object", "shape": "cylinder",
                   "dimensions": [0.12, 0.025],
                   "pose_world": [0.4, 0.1, 0.0, 0.0, 0.0, 0.0]},
    })
    assert res.success
    # scene add before attach; gripper open before close
    assert scene.calls[0][0] == "add"
    assert scene.calls[-1][0] == "attach"
    assert grip.calls[0][0] == "open"
    assert grip.calls[1][0] == "close"
    # arm: pregrasp pose then straight approach along local z
    assert arm.calls[0][0] == "plan_to_pose"
    assert arm.calls[1][0] == "relative_move"
    assert arm.calls[1][5] == "z"  # local_axis
    # held object remembered
    assert h._held is not None and h._held.name == "target_object"


def test_pick_requires_pose():
    h, _, _, _ = _make()
    assert not h.pick({"cmd": "pick", "object": {"name": "x"}}).success


def test_lift_direction_sign():
    h, arm, _, _ = _make()
    h.lift({"cmd": "lift", "direction": "z+", "distance_m": 0.1})
    assert arm.calls[-1][0] == "relative_move"
    assert abs(arm.calls[-1][4] - 0.1) < 1e-9  # dz = +0.1 (tuple index 4)
    h.lift({"cmd": "lift", "direction": "z-", "distance_m": 0.05})
    assert abs(arm.calls[-1][4] + 0.05) < 1e-9  # dz = -0.05


def test_place_uses_held_object_and_lowers():
    h, arm, _, _ = _make()
    h.pick({"cmd": "pick",
            "object": {"name": "obj", "shape": "cylinder",
                       "dimensions": [0.12, 0.025],
                       "pose_world": [0.4, 0.1, 0.0, 0.0, 0.0, 0.0]}})
    arm.calls.clear()
    res = h.place({"cmd": "place", "object_name": "obj",
                   "target_pose_world": [0.6, -0.2, 0.0, 0.0, 0.0, 0.0]})
    assert res.success
    assert arm.calls[0][0] == "plan_to_pose"    # move above target
    assert arm.calls[1][0] == "relative_move"   # lower
    assert arm.calls[1][4] < 0                  # dz negative (tuple index 4)


def test_place_requires_target():
    h, _, _, _ = _make()
    assert not h.place({"cmd": "place", "object_name": "x"}).success


def test_release_opens_and_detaches():
    h, _, grip, scene = _make()
    h.pick({"cmd": "pick",
            "object": {"name": "obj", "pose_world": [0.4, 0.1, 0.0, 0, 0, 0]}})
    res = h.release({"cmd": "release"})
    assert res.success
    assert grip.calls[-1][0] == "open"
    assert ("detach", "obj", "robotiq_85_tcp") in scene.calls
    assert h._held is None


def test_home_uses_named_state():
    h, arm, _, _ = _make()
    res = h.home({"cmd": "home", "arm_home_pose": "home"})
    assert res.success
    assert arm.calls[-1] == ("plan_to_named", "home")


def test_command_value_overrides_default():
    h, arm, _, _ = _make()
    h.home({"cmd": "home", "arm_home_pose": "custom_pose"})
    assert arm.calls[-1] == ("plan_to_named", "custom_pose")


def test_mock_handlers_validation():
    handlers = H.build_mock_handlers()
    assert handlers["pick"]({"cmd": "pick", "object": {}}).success is False
    assert handlers["pick"]({"cmd": "pick",
                             "object": {"pose_world": [0] * 6}}).success is True
    assert handlers["home"]({"cmd": "home"}).success is True
