"""통합 bringup launch — UR3 LLM Pick & Place 전체 스택 단일 진입점.

기동 순서:
  터미널 1:  ros2 launch ur3_mtc_pick_place ur3_integrated.launch.py
             → Gazebo(UR3+세계) + MoveGroup + Moveit_module + YOLO 시작

  터미널 2:  ros2 run llm_agent agent_node
             → CLI 대화형 에이전트 시작 (별도 터미널에서 수동 실행)

포함 컴포넌트:
  1. ur3_mtc_demo.launch.py  — Gazebo 시뮬·ros2_control·MoveGroup·camera TF·bridge
  2. moveit_module.launch.py — /moveit/execute 서비스 서버 (Moveit_module)
  3. YOLO 노드              — ROBOT_VISION 객체 탐지 발행 (Gazebo 없이)
  4. camera bridge          — /camera/image·/camera/depth_image·/camera/camera_info

빌드 요구사항 (ros2_ws/src/ 아래 있어야 함):
  - ur3_mtc_pick_place   (PANDA_ENV)
  - ur3_moveit_module    (Moveit_module) — llm_agent_msgs 먼저 빌드
  - llm_agent_msgs       (LLM_Agent/src/)
  - robot_vision         (ROBOT_VISION)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 공통 인수 ─────────────────────────────────────────
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur3')
    mock_arg = DeclareLaunchArgument(
        'mock', default_value='false',
        description='Moveit_module mock mode (서비스 핸드셰이크만 검증, MoveIt 미필요).',
    )

    # ── 1. 베이스 bringup ────────────────────────────────
    # Gazebo + ros2_control + robot_state_publisher + MoveGroup + RViz
    # + camera static TF + /clock bridge
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur3_mtc_pick_place'),
            '/launch/ur3_mtc_demo.launch.py',
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
        }.items(),
    )

    # ── 2. Moveit_module ─────────────────────────────────
    # /moveit/execute 서비스 서버.  MoveGroup 기동 후 연결됨.
    moveit_module_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur3_moveit_module'),
            '/launch/moveit_module.launch.py',
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'mock': LaunchConfiguration('mock'),
        }.items(),
    )

    # ── 3. camera bridge (ROBOT_VISION용) ─────────────────
    # /clock은 ur3_mtc_demo에서 이미 브릿지됨.
    # fixed_rgbd_camera.sdf type="camera" sensor → camera_info at /camera/image/camera_info
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_camera',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/image/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
    )

    # ── 4. YOLO 탐지 노드 ─────────────────────────────────
    # Gazebo는 이미 demo_launch에서 기동되므로 launch_gazebo:=false 상당.
    # vision_bringup.launch.py 대신 노드 직접 포함(bridge 중복 방지).
    yolo_node = Node(
        package='robot_vision',
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
    )

    return LaunchDescription([
        ur_type_arg,
        mock_arg,
        demo_launch,
        moveit_module_launch,
        camera_bridge,
        yolo_node,
    ])
