#!/usr/bin/env python3
"""
UR3 + Robotiq 2F-85  MoveIt Task Constructor  Pick & Place
=============================================================
Panda C++ 버전(pick_place_task.cpp)의 동일 Plan 로직을 Python으로 이식.

스테이지 구조 (Panda C++ → 이 파일 Python 1:1 대응):
  CurrentState            → stages.CurrentState
  MoveTo (open)           → stages.MoveTo  (gripper open)
  Connect (move-to-pick)  → stages.Connect
  [SerialContainer: pick]
    MoveRelative (접근)   → stages.MoveRelative  + CartesianPath
    GenerateGraspPose     → stages.GenerateGraspPose + ComputeIK
    ModifyPlanningScene   → stages.ModifyPlanningScene (충돌 허용)
    MoveTo (close)        → stages.MoveTo  (gripper close)
    attachObject          → stages.ModifyPlanningScene
    MoveRelative (lift)   → stages.MoveRelative  + CartesianPath
    ModifyPlanningScene   → (object-surface 충돌 금지)
  Connect (move-to-place) → stages.Connect
  [SerialContainer: place]
    MoveRelative (lower)  → stages.MoveRelative  + CartesianPath
    GeneratePlacePose     → stages.GeneratePlacePose + ComputeIK
    MoveTo (open)         → stages.MoveTo
    ModifyPlanningScene   → (충돌 금지 복원)
    detachObject          → stages.ModifyPlanningScene
    MoveRelative (retreat)→ stages.MoveRelative  + CartesianPath
  MoveTo (home)           → stages.MoveTo
"""

import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from moveit.planning import MoveItPy
from moveit.task_constructor import core, stages
from moveit.planning import PlanningSceneMonitor


# ──────────────────────────────────────────────────────────
# 씬 유틸리티
# ──────────────────────────────────────────────────────────

def spawn_collision_object(node: Node, obj: CollisionObject):
    """Planning Scene에 충돌 물체를 추가한다."""
    from moveit_msgs.srv import ApplyPlanningScene
    from moveit_msgs.msg import PlanningScene
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=5.0)
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(obj)
    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)


def make_box(name: str, frame_id: str,
             dims: list, pose_vals: list) -> CollisionObject:
    """박스형 CollisionObject 생성 헬퍼."""
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = frame_id
    obj.operation = CollisionObject.ADD
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = dims
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = pose_vals[:3]
    from scipy.spatial.transform import Rotation
    q = Rotation.from_euler("xyz", pose_vals[3:6]).as_quat()
    p.orientation.x, p.orientation.y = q[0], q[1]
    p.orientation.z, p.orientation.w = q[2], q[3]
    obj.primitives.append(prim)
    obj.primitive_poses.append(p)
    return obj


def make_cylinder(name: str, frame_id: str,
                  height: float, radius: float,
                  pose_vals: list) -> CollisionObject:
    """원통형 CollisionObject 생성 헬퍼."""
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = frame_id
    obj.operation = CollisionObject.ADD
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.CYLINDER
    prim.dimensions = [height, radius]
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x = pose_vals[0]
    p.position.y = pose_vals[1]
    p.position.z = pose_vals[2] + height / 2.0   # 바닥 기준 → 중심 보정
    p.orientation.w = 1.0
    obj.primitives.append(prim)
    obj.primitive_poses.append(p)
    return obj


def setup_demo_scene(node: Node, params: dict):
    """테이블과 파지 물체를 Planning Scene에 배치한다."""
    time.sleep(0.5)   # ApplyPlanningScene 서비스 대기

    if params.get("spawn_table", True):
        td = params["table_dimensions"]
        tp = params["table_pose"]
        table = make_box(
            params["table_name"],
            params["table_reference_frame"],
            td,
            [tp[0], tp[1], tp[2] - td[2] / 2.0, tp[3], tp[4], tp[5]],
        )
        spawn_collision_object(node, table)

    od = params["object_dimensions"]      # [height, radius]
    op = params["object_pose"]
    obj = make_cylinder(
        params["object_name"],
        params["object_reference_frame"],
        od[0], od[1], op,
    )
    spawn_collision_object(node, obj)


# ──────────────────────────────────────────────────────────
# 그리퍼 링크명 조회 유틸
# ──────────────────────────────────────────────────────────

def get_gripper_links(task: core.Task, hand_group: str) -> list:
    """그리퍼 그룹에 속한 충돌 링크 목록 반환."""
    robot_model = task.getRobotModel()
    jmg = robot_model.getJointModelGroup(hand_group)
    if jmg is None:
        return []
    return list(jmg.getLinkModelNamesWithCollisionGeometry())


# ──────────────────────────────────────────────────────────
# MTC Pick & Place Task 빌더
# ──────────────────────────────────────────────────────────

def build_pick_place_task(node: Node, params: dict) -> core.Task:
    """
    UR3 + Robotiq 2F-85 Pick & Place Task를 구성하여 반환한다.
    Panda C++ pick_place_task.cpp 와 동일한 스테이지 구조.
    """
    # ── 플래너 초기화 ──────────────────────────────────────
    sampling_planner = core.PipelinePlanner(node)
    sampling_planner.setProperty("goal_joint_tolerance", 1e-5)

    cartesian_planner = core.CartesianPath()
    cartesian_planner.setMaxVelocityScalingFactor(1.0)
    cartesian_planner.setMaxAccelerationScalingFactor(1.0)
    cartesian_planner.setStepSize(0.01)

    # ── Task 생성 ──────────────────────────────────────────
    task = core.Task()
    task.name = "ur3_pick_and_place"
    task.loadRobotModel(node)

    task.setProperty("group",              params["arm_group_name"])
    task.setProperty("eef",               params["eef_name"])
    task.setProperty("hand",              params["hand_group_name"])
    task.setProperty("hand_grasping_frame", params["hand_frame"])
    task.setProperty("ik_frame",          params["hand_frame"])

    # ── Stage 1: Current State ─────────────────────────────
    current_state_stage = stages.CurrentState("current state")
    task.add(current_state_stage)

    # ── Stage 2: Open Hand ────────────────────────────────
    open_hand = stages.MoveTo("open hand", sampling_planner)
    open_hand.group = params["hand_group_name"]
    open_hand.setGoal(params["hand_open_pose"])
    initial_state_ptr = open_hand      # GenerateGraspPose 모니터링용 참조
    task.add(open_hand)

    # ── Stage 3: Connect (현재 → Pick 접근) ───────────────
    connect_pick = stages.Connect(
        "move to pick",
        [(params["arm_group_name"], sampling_planner)],
    )
    connect_pick.setTimeout(5.0)
    connect_pick.properties().configureInitFrom(core.Stage.PARENT)
    task.add(connect_pick)

    # ── Stage 4: Pick SerialContainer ────────────────────
    pick_container = core.SerialContainer("pick object")
    task.properties().exposeTo(
        pick_container.properties(),
        ["eef", "hand", "group", "ik_frame"],
    )
    pick_container.properties().configureInitFrom(
        core.Stage.PARENT,
        ["eef", "hand", "group", "ik_frame"],
    )

    # 4-1: 물체 접근 (hand_frame Z축 방향으로 직선 이동)
    approach = stages.MoveRelative("approach object", cartesian_planner)
    approach.properties().set("marker_ns", "approach_object")
    approach.properties().set("link", params["hand_frame"])
    approach.properties().configureInitFrom(core.Stage.PARENT, ["group"])
    approach.setMinMaxDistance(
        params["approach_object_min_dist"],
        params["approach_object_max_dist"],
    )
    vec = Vector3Stamped()
    vec.header.frame_id = params["hand_frame"]
    vec.vector.z = 1.0              # hand_frame 기준 +Z (그리퍼 전방)
    approach.setDirection(vec)
    pick_container.insert(approach)

    # 4-2: 파지 자세 자동 생성 + IK 계산
    grasp_gen = stages.GenerateGraspPose("generate grasp pose")
    grasp_gen.properties().configureInitFrom(core.Stage.PARENT)
    grasp_gen.properties().set("marker_ns", "grasp_pose")
    grasp_gen.setPreGraspPose(params["hand_open_pose"])
    grasp_gen.setObject(params["object_name"])
    grasp_gen.setAngleDelta(math.pi / 12)           # 15도 간격, 최대 24후보
    grasp_gen.setMonitoredStage(initial_state_ptr)

    # IK 래퍼: 각 후보에 대해 역기구학 자동 계산
    grasp_ik = stages.ComputeIK("grasp pose IK", grasp_gen)
    grasp_ik.setMaxIKSolutions(8)
    grasp_ik.setMinSolutionDistance(1.0)
    # TCP 오프셋: [x,y,z,r,p,y] → 4×4 변환 행렬
    gft = params["grasp_frame_transform"]   # [x,y,z,roll,pitch,yaw]
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler("xyz", gft[3:6]).as_matrix()
    tf = np.eye(4)
    tf[:3, :3] = rot
    tf[:3, 3] = gft[:3]
    grasp_ik.setIKFrame(tf, params["hand_frame"])
    grasp_ik.properties().configureInitFrom(core.Stage.PARENT, ["eef", "group"])
    grasp_ik.properties().configureInitFrom(core.Stage.INTERFACE, ["target_pose"])
    pick_container.insert(grasp_ik)

    # 4-3: 그리퍼 ↔ 물체 충돌 허용
    allow_collision = stages.ModifyPlanningScene("allow collision (gripper,object)")
    gripper_links = get_gripper_links(task, params["hand_group_name"])
    allow_collision.allowCollisions(params["object_name"], gripper_links, True)
    pick_container.insert(allow_collision)

    # 4-4: 그리퍼 닫기 (파지)
    close_hand = stages.MoveTo("close hand", sampling_planner)
    close_hand.group = params["hand_group_name"]
    close_hand.setGoal(params["hand_close_pose"])
    pick_container.insert(close_hand)

    # 4-5: 물체를 그리퍼에 논리적으로 부착
    attach_obj = stages.ModifyPlanningScene("attach object")
    attach_obj.attachObject(params["object_name"], params["hand_frame"])
    pick_container.insert(attach_obj)

    # 4-6: 물체 ↔ 테이블 충돌 허용 (들어올리는 동안)
    allow_table = stages.ModifyPlanningScene("allow collision (object,table)")
    allow_table.allowCollisions(
        [params["object_name"]],
        [params["surface_link"]],
        True,
    )
    pick_container.insert(allow_table)

    # 4-7: 들어올리기 (world Z축 방향)
    lift = stages.MoveRelative("lift object", cartesian_planner)
    lift.properties().configureInitFrom(core.Stage.PARENT, ["group"])
    lift.setMinMaxDistance(
        params["lift_object_min_dist"],
        params["lift_object_max_dist"],
    )
    lift.setIKFrame(params["hand_frame"])
    lift.properties().set("marker_ns", "lift_object")
    vec_up = Vector3Stamped()
    vec_up.header.frame_id = params["world_frame"]
    vec_up.vector.z = 1.0           # 위쪽
    lift.setDirection(vec_up)
    pick_container.insert(lift)

    # 4-8: 물체 ↔ 테이블 충돌 금지 (이동 중 안전)
    forbid_table = stages.ModifyPlanningScene("forbid collision (object,table)")
    forbid_table.allowCollisions(
        [params["object_name"]],
        [params["surface_link"]],
        False,
    )
    pick_container.insert(forbid_table)

    task.add(pick_container)

    # ── Stage 5: Connect (Pick → Place 이동) ──────────────
    connect_place = stages.Connect(
        "move to place",
        [(params["arm_group_name"], sampling_planner)],
    )
    connect_place.setTimeout(5.0)
    connect_place.properties().configureInitFrom(core.Stage.PARENT)
    task.add(connect_place)

    # ── Stage 6: Place SerialContainer ───────────────────
    place_container = core.SerialContainer("place object")
    task.properties().exposeTo(
        place_container.properties(),
        ["eef", "hand", "group"],
    )
    place_container.properties().configureInitFrom(
        core.Stage.PARENT,
        ["eef", "hand", "group"],
    )

    # 6-1: 내려놓기 (world -Z 방향)
    lower = stages.MoveRelative("lower object", cartesian_planner)
    lower.properties().set("marker_ns", "lower_object")
    lower.properties().set("link", params["hand_frame"])
    lower.properties().configureInitFrom(core.Stage.PARENT, ["group"])
    lower.setMinMaxDistance(0.03, 0.13)
    vec_down = Vector3Stamped()
    vec_down.header.frame_id = params["world_frame"]
    vec_down.vector.z = -1.0        # 아래쪽
    lower.setDirection(vec_down)
    place_container.insert(lower)

    # 6-2: 배치 자세 자동 생성 + IK
    od = params["object_dimensions"]
    pp = params["place_pose"]
    place_pose_msg = PoseStamped()
    place_pose_msg.header.frame_id = params["object_reference_frame"]
    place_pose_msg.pose.position.x = pp[0]
    place_pose_msg.pose.position.y = pp[1]
    place_pose_msg.pose.position.z = pp[2] + od[0] / 2.0 + params["place_surface_offset"]
    place_pose_msg.pose.orientation.w = 1.0

    place_gen = stages.GeneratePlacePose("generate place pose")
    place_gen.properties().configureInitFrom(core.Stage.PARENT, ["ik_frame"])
    place_gen.properties().set("marker_ns", "place_pose")
    place_gen.setObject(params["object_name"])
    place_gen.setPose(place_pose_msg)
    place_gen.setMonitoredStage(pick_container)   # Pick 결과 모니터링

    place_ik = stages.ComputeIK("place pose IK", place_gen)
    place_ik.setMaxIKSolutions(2)
    place_ik.setIKFrame(tf, params["hand_frame"])  # grasp와 동일 TCP 오프셋
    place_ik.properties().configureInitFrom(core.Stage.PARENT, ["eef", "group"])
    place_ik.properties().configureInitFrom(core.Stage.INTERFACE, ["target_pose"])
    place_container.insert(place_ik)

    # 6-3: 그리퍼 열기 (release)
    open_hand_place = stages.MoveTo("open hand", sampling_planner)
    open_hand_place.group = params["hand_group_name"]
    open_hand_place.setGoal(params["hand_open_pose"])
    place_container.insert(open_hand_place)

    # 6-4: 그리퍼 ↔ 물체 충돌 금지 복원
    forbid_collision = stages.ModifyPlanningScene("forbid collision (gripper,object)")
    forbid_collision.allowCollisions(params["object_name"], gripper_links, False)
    place_container.insert(forbid_collision)

    # 6-5: 물체 분리
    detach_obj = stages.ModifyPlanningScene("detach object")
    detach_obj.detachObject(params["object_name"], params["hand_frame"])
    place_container.insert(detach_obj)

    # 6-6: 후퇴 (hand_frame -Z 방향)
    retreat = stages.MoveRelative("retreat after place", cartesian_planner)
    retreat.properties().configureInitFrom(core.Stage.PARENT, ["group"])
    retreat.setMinMaxDistance(0.12, 0.25)
    retreat.setIKFrame(params["hand_frame"])
    retreat.properties().set("marker_ns", "retreat")
    vec_ret = Vector3Stamped()
    vec_ret.header.frame_id = params["hand_frame"]
    vec_ret.vector.z = -1.0         # 그리퍼 후방
    retreat.setDirection(vec_ret)
    place_container.insert(retreat)

    task.add(place_container)

    # ── Stage 7: Move Home ────────────────────────────────
    move_home = stages.MoveTo("move home", sampling_planner)
    move_home.properties().configureInitFrom(core.Stage.PARENT, ["group"])
    move_home.setGoal(params["arm_home_pose"])
    move_home.restrictDirection(stages.MoveTo.FORWARD)
    task.add(move_home)

    return task


# ──────────────────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = rclpy.create_node(
        "ur3_pick_place",
        automatically_declare_parameters_from_overrides=True,
    )

    # ─ 파라미터 로드 ─────────────────────────────────────
    def get(name, default=None):
        try:
            return node.get_parameter(name).value
        except Exception:
            return default

    params = {
        "max_solutions":            get("max_solutions", 10),
        "execute":                  get("execute", True),
        "arm_group_name":           get("arm_group_name",  "ur_manipulator"),
        "eef_name":                 get("eef_name",        "robotiq_2f_85"),
        "hand_group_name":          get("hand_group_name", "gripper"),
        "hand_frame":               get("hand_frame",      "robotiq_85_tcp"),
        "hand_open_pose":           get("hand_open_pose",  "open"),
        "hand_close_pose":          get("hand_close_pose", "close"),
        "arm_home_pose":            get("arm_home_pose",   "home"),
        "world_frame":              get("world_frame",     "world"),
        "table_reference_frame":    get("table_reference_frame", "world"),
        "object_reference_frame":   get("object_reference_frame", "world"),
        "surface_link":             get("surface_link",    "table_surface"),
        "spawn_table":              get("spawn_table",     True),
        "table_name":               get("table_name",      "table"),
        "table_dimensions":         get("table_dimensions", [0.5, 0.5, 0.02]),
        "table_pose":               get("table_pose",      [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        "object_name":              get("object_name",     "cylinder"),
        "object_dimensions":        get("object_dimensions", [0.12, 0.025]),
        "object_pose":              get("object_pose",     [0.4, 0.1, 0.0, 0.0, 0.0, 0.0]),
        "grasp_frame_transform":    get("grasp_frame_transform", [0.0, 0.0, 0.13, 3.1416, 0.0, 0.0]),
        "place_pose":               get("place_pose",      [0.4, -0.2, 0.0, 0.0, 0.0, 0.0]),
        "place_surface_offset":     get("place_surface_offset", 0.001),
        "approach_object_min_dist": get("approach_object_min_dist", 0.08),
        "approach_object_max_dist": get("approach_object_max_dist", 0.15),
        "lift_object_min_dist":     get("lift_object_min_dist",  0.05),
        "lift_object_max_dist":     get("lift_object_max_dist",  0.15),
    }

    # ─ 씬 설정 ───────────────────────────────────────────
    node.get_logger().info("Setting up demo scene...")
    setup_demo_scene(node, params)

    # ─ Task 구성 ─────────────────────────────────────────
    node.get_logger().info("Building pick & place task...")
    task = build_pick_place_task(node, params)

    try:
        task.init()
    except Exception as e:
        node.get_logger().error(f"Task initialization failed: {e}")
        return

    # ─ 계획 실행 ─────────────────────────────────────────
    node.get_logger().info("Planning...")
    if task.plan(params["max_solutions"]):
        node.get_logger().info("Planning succeeded!")
        task.publish(task.solutions[0])   # RViz TaskSolutionPanel에 발행

        if params["execute"]:
            node.get_logger().info("Executing solution...")
            result = task.execute(task.solutions[0])
            if result.val == MoveItErrorCodes.SUCCESS:
                node.get_logger().info("Execution complete!")
            else:
                node.get_logger().error(f"Execution failed: {result.val}")
        else:
            node.get_logger().info("Execution disabled. Inspect in RViz.")
    else:
        node.get_logger().error("Planning failed!")

    # RViz 인트로스펙션 유지
    node.get_logger().info("Keeping node alive for RViz inspection...")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
