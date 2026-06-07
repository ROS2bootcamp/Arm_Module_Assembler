"""Planning-scene management: add/remove collision objects, attach/detach.

Uses the /apply_planning_scene service with diff scenes — the same mechanism
the PANDA_ENV reference uses (make_box / make_cylinder / spawn_collision_object),
extended with attach/detach so the object travels with the gripper between the
separate pick / lift / place / release commands.
"""

from __future__ import annotations

from typing import List, Sequence

from . import geometry_utils as gu


class SceneError(RuntimeError):
    pass


class SceneManager:
    def __init__(self, node, world_frame: str = "world", logger=None):
        self._node = node
        self._world_frame = world_frame
        self._log = logger or (lambda _m: None)

        from moveit_msgs.srv import ApplyPlanningScene
        self._cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    # ── low level ───────────────────────────────────────────────────
    def _apply(self, scene, timeout_sec: float = 5.0) -> None:
        import time
        if not self._cli.wait_for_service(timeout_sec=5.0):
            raise SceneError("/apply_planning_scene service unavailable")
        from moveit_msgs.srv import ApplyPlanningScene
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._cli.call_async(req)
        # The node is spun by the module's executor in another thread, so we
        # poll the future rather than re-entering the executor here.
        deadline = time.time() + timeout_sec
        while not future.done():
            if time.time() > deadline:
                raise SceneError("ApplyPlanningScene call timed out")
            time.sleep(0.01)
        if future.result() is None:
            raise SceneError("ApplyPlanningScene returned no result")

    def _make_collision_object(self, name: str, shape: str,
                               dimensions: Sequence[float],
                               pose_world: Sequence[float]):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose

        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = self._world_frame
        obj.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        shape = (shape or "cylinder").lower()
        pose_vals = list(pose_world)
        z_center = pose_vals[2]
        if shape == "cylinder":            # dims [height, radius]
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [float(dimensions[0]), float(dimensions[1])]
            z_center = pose_vals[2] + float(dimensions[0]) / 2.0
        elif shape == "box":               # dims [x, y, z]
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(d) for d in dimensions[:3]]
            z_center = pose_vals[2] + float(dimensions[2]) / 2.0
        elif shape == "sphere":            # dims [radius]
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [float(dimensions[0])]
            z_center = pose_vals[2] + float(dimensions[0])
        else:
            raise SceneError(f"unsupported shape '{shape}'")

        p = Pose()
        p.position.x = float(pose_vals[0])
        p.position.y = float(pose_vals[1])
        p.position.z = float(z_center)
        roll = pose_vals[3] if len(pose_vals) > 3 else 0.0
        pitch = pose_vals[4] if len(pose_vals) > 4 else 0.0
        yaw = pose_vals[5] if len(pose_vals) > 5 else 0.0
        qx, qy, qz, qw = gu.rpy_to_quaternion(roll, pitch, yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw

        obj.primitives.append(prim)
        obj.primitive_poses.append(p)
        return obj

    # ── public API ──────────────────────────────────────────────────
    def add_object(self, name: str, shape: str,
                   dimensions: Sequence[float], pose_world: Sequence[float]) -> None:
        from moveit_msgs.msg import PlanningScene
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            self._make_collision_object(name, shape, dimensions, pose_world))
        self._apply(scene)
        self._log(f"scene: added '{name}' ({shape})")

    def add_box(self, name: str, dimensions: Sequence[float],
                pose_world: Sequence[float]) -> None:
        self.add_object(name, "box", dimensions, pose_world)

    def remove_object(self, name: str) -> None:
        from moveit_msgs.msg import PlanningScene, CollisionObject
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = self._world_frame
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        self._apply(scene)
        self._log(f"scene: removed '{name}'")

    def attach(self, name: str, link: str, touch_links: List[str] = None) -> None:
        from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = name
        aco.object.operation = CollisionObject.ADD
        if touch_links:
            aco.touch_links = list(touch_links)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True
        self._apply(scene)
        self._log(f"scene: attached '{name}' to '{link}'")

    def detach(self, name: str, link: str) -> None:
        from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True
        self._apply(scene)
        self._log(f"scene: detached '{name}' from '{link}'")
