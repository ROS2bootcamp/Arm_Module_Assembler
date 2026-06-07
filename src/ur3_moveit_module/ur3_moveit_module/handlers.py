"""Command handlers implementing the decomposed pick & place sequence.

Each handler takes the parsed command dict and returns a HandlerResult. The
handlers honour the LLM_Agent protocol: pick and lift are SEPARATE so the agent
can run YOLO grip verification in between; place moves/lowers while release
opens the gripper and detaches.

Values from the command JSON take precedence; missing fields fall back to the
module defaults (UR3 + Robotiq 2F-85). See MOVEIT_MODULE_DESIGN.md 3.2 / 4.2.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Sequence

from . import grasp_planner as gp
from .command_router import HandlerResult, Handler


@dataclass
class HeldObject:
    name: str
    shape: str
    dimensions: Sequence[float]
    grasp_frame_transform: Sequence[float]


class Handlers:
    """Real-mode handlers wired to arm / gripper / scene controllers."""

    def __init__(self, arm, gripper, scene, defaults: dict, logger=None):
        self._arm = arm
        self._gripper = gripper
        self._scene = scene
        self._d = defaults
        self._log = logger or (lambda _m: None)
        self._held: Optional[HeldObject] = None

    # ── helpers ─────────────────────────────────────────────────────
    def _get(self, cmd: dict, key: str):
        """Command value if present, else module default."""
        if key in cmd and cmd[key] is not None:
            return cmd[key]
        return self._d.get(key)

    # ── handlers ────────────────────────────────────────────────────
    def scan(self, cmd: dict) -> HandlerResult:
        waypoints = cmd.get("waypoints")
        if not waypoints:
            return HandlerResult.fail("scan: 'waypoints' missing or empty")
        link = self._get(cmd, "hand_frame")
        for i, wp in enumerate(waypoints):
            if len(wp) != 6:
                return HandlerResult.fail(
                    f"scan: waypoint {i} must be [x,y,z,r,p,y]")
            self._arm.plan_to_pose(list(wp), link)
            self._log(f"scan: reached waypoint {i + 1}/{len(waypoints)}")
        return HandlerResult.ok()

    def pick(self, cmd: dict) -> HandlerResult:
        obj = cmd.get("object")
        if not obj or "pose_world" not in obj:
            return HandlerResult.fail("pick: 'object.pose_world' required")

        name = obj.get("name", "target_object")
        shape = obj.get("shape", "cylinder")
        dims = obj.get("dimensions", [0.12, 0.025])
        pose_world = obj["pose_world"]
        grasp_tf = self._get(cmd, "grasp_frame_transform")
        hand_frame = self._get(cmd, "hand_frame")
        approach_max = float(self._get_num(cmd, "approach_max_dist"))

        # 1. register object in the planning scene
        self._scene.add_object(name, shape, dims, pose_world)

        # 2. open gripper
        self._gripper.open(self._get(cmd, "hand_open_pose"))

        # 3. compute grasp geometry
        plan = gp.plan_grasp(pose_world, grasp_tf, shape, dims,
                             approach_distance=approach_max)

        # 4. free-space move to pre-grasp
        self._arm.plan_to_pose(plan.pregrasp_hand_pose, hand_frame)

        # 5. straight approach along gripper +Z toward the object
        self._arm.relative_move(hand_frame, local_axis="z",
                                local_distance=plan.approach_distance)

        # 6. close gripper
        self._gripper.close(self._get(cmd, "hand_close_pose"))

        # 7. attach object so it travels with the hand
        self._scene.attach(name, hand_frame)
        self._held = HeldObject(name, shape, dims, grasp_tf)
        return HandlerResult.ok()

    def lift(self, cmd: dict) -> HandlerResult:
        distance = float(cmd.get("distance_m", 0.1))
        direction = cmd.get("direction", "z+")
        hand_frame = self._get(cmd, "hand_frame")
        sign = -1.0 if str(direction).startswith("z-") else 1.0
        self._arm.relative_move(hand_frame, dz=sign * distance)
        return HandlerResult.ok()

    def place(self, cmd: dict) -> HandlerResult:
        target = cmd.get("target_pose_world")
        if not target:
            return HandlerResult.fail("place: 'target_pose_world' required")
        hand_frame = self._get(cmd, "hand_frame")
        surface_offset = float(cmd.get("place_surface_offset", 0.001))

        if self._held is not None:
            shape, dims, grasp_tf = (self._held.shape, self._held.dimensions,
                                     self._held.grasp_frame_transform)
        else:  # no memory of pick (e.g. restarted) — use defaults
            shape, dims = "cylinder", [0.12, 0.025]
            grasp_tf = self._get(cmd, "grasp_frame_transform")

        lower = 0.1
        plan = gp.plan_place(target, grasp_tf, shape, dims,
                             surface_offset=surface_offset, lower_distance=lower)
        # move above target, then lower straight down
        self._arm.plan_to_pose(plan.pregrasp_hand_pose, hand_frame)
        self._arm.relative_move(hand_frame, dz=-lower)
        return HandlerResult.ok()

    def release(self, cmd: dict) -> HandlerResult:
        hand_frame = self._get(cmd, "hand_frame")
        self._gripper.open(self._get(cmd, "hand_open_pose"))
        if self._held is not None:
            self._scene.detach(self._held.name, hand_frame)
            self._held = None
        return HandlerResult.ok()

    def home(self, cmd: dict) -> HandlerResult:
        self._arm.plan_to_named(self._get(cmd, "arm_home_pose"))
        return HandlerResult.ok()

    # ── wiring ──────────────────────────────────────────────────────
    def _get_num(self, cmd: dict, key: str):
        val = self._get(cmd, key)
        return val if val is not None else 0.15

    def as_dict(self) -> Dict[str, Handler]:
        return {
            "scan": self.scan,
            "pick": self.pick,
            "lift": self.lift,
            "place": self.place,
            "release": self.release,
            "home": self.home,
        }


def build_mock_handlers(logger=None) -> Dict[str, Handler]:
    """Handlers that validate minimal fields but perform no motion.

    Used in mock mode to verify the /moveit_command <-> /moveit_status topic
    handshake with the LLM_Agent without a running Gazebo / move_group.
    """
    log = logger or (lambda _m: None)

    def scan(cmd):
        if not cmd.get("waypoints"):
            return HandlerResult.fail("scan: waypoints missing")
        log(f"[mock] scan {len(cmd['waypoints'])} waypoints")
        return HandlerResult.ok()

    def pick(cmd):
        if not cmd.get("object", {}).get("pose_world"):
            return HandlerResult.fail("pick: object.pose_world required")
        log("[mock] pick")
        return HandlerResult.ok()

    def lift(cmd):
        log(f"[mock] lift {cmd.get('distance_m', 0.1)}m {cmd.get('direction', 'z+')}")
        return HandlerResult.ok()

    def place(cmd):
        if not cmd.get("target_pose_world"):
            return HandlerResult.fail("place: target_pose_world required")
        log("[mock] place")
        return HandlerResult.ok()

    def release(cmd):
        log("[mock] release")
        return HandlerResult.ok()

    def home(cmd):
        log("[mock] home")
        return HandlerResult.ok()

    return {"scan": scan, "pick": pick, "lift": lift,
            "place": place, "release": release, "home": home}
