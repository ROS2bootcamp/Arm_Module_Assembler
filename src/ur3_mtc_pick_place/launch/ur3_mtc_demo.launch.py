"""
UR3 + Robotiq 2F-85 MoveIt Task Constructor 데모 런치 파일
=============================================================
실행 순서:
  터미널 1:  ros2 launch ur3_mtc_pick_place ur3_mtc_demo.launch.py
  터미널 2:  ros2 launch ur3_mtc_pick_place ur3_mtc_run.launch.py

통합 스택으로 실행할 경우 ur3_integrated.launch.py 사용.

환경 변수:
  UR3_CONVENIENCE_ENV_PATH  UR3_CONVENIENCE_ENV 클론 경로 (기본: ~/WorkspaceMain/UR3_CONVENIENCE_ENV)
  설정하지 않으면 기본값 사용. IGN_GAZEBO_RESOURCE_PATH를 자동으로 설정한다.

Camera TF (tuning):
  fixed_rgbd_camera.sdf 기준: x=1.2, z=0.5, yaw=PI (facing -X).
  Optical frame 표준: roll=-PI/2, pitch=0, yaw=+PI/2.
  shelf surface z=0.34 m 기준으로 camera z=0.5 에서 수평시야로 충분히 커버됨.
  실제 시뮬레이션에서 ros2 topic echo /vision/detection_results 좌표가
  Gazebo 물체 위치와 일치하는지 확인하며 tuning 필요.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration('ur_type').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)

    # ── URDF 생성 (UR3 + Robotiq 2F-85) ──────────────────
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('ur3_mtc_pick_place'),
                'urdf', 'ur3_with_gripper.urdf.xacro',
            ]),
            ' ur_type:=', ur_type,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # ── SRDF 생성 (name:=ur → ur_manipulator group) ───────
    robot_description_semantic_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('ur3_mtc_pick_place'),
                'srdf', 'ur3_with_gripper.srdf.xacro',
            ]),
            ' name:=ur ',   # D17: hardcoded — generates 'ur_manipulator' group
        ]),
        value_type=str,
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # ── 운동학·플래닝 파이프라인 ─────────────────────────
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'), 'config', 'kinematics.yaml'
    ])
    ompl_planning_yaml = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'), 'config', 'ompl_planning.yaml'
    ])

    # ── Gazebo 시뮬레이션 + ros2_control ──────────────────
    # 시스템 ur_sim_control.launch.py 에 robot_description ParameterValue 버그가 있어
    # 패키지 내 수정 버전(ur_sim_control_fixed.launch.py)을 사용한다.
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur3_mtc_pick_place'),
            '/launch/ur_sim_control_fixed.launch.py',
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'false',
            'world_file': world_file,
        }.items(),
    )

    # ── Static TF: world → base_link (identity) ──────────
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_base',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    # ── Static TF: base_link → camera_link ───────────────
    # fixed_rgbd_camera.sdf: camera looks in link +X; placed at yaw=PI → faces -X world.
    # Optical frame: Z=depth=-X world, X=right=+Y world, Y=down=-Z world.
    # RPY for base_link→camera_link: roll=-PI/2, pitch=0, yaw=+PI/2.
    # camera_z=0.5 chosen so shelf surface (z=0.34) is within vertical FOV (±0.28 m at 0.65 m).
    camera_x  = LaunchConfiguration('camera_x').perform(context)
    camera_y  = LaunchConfiguration('camera_y').perform(context)
    camera_z  = LaunchConfiguration('camera_z').perform(context)
    camera_roll  = LaunchConfiguration('camera_roll').perform(context)
    camera_pitch = LaunchConfiguration('camera_pitch').perform(context)
    camera_yaw   = LaunchConfiguration('camera_yaw').perform(context)

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_link',
        output='log',
        arguments=[
            camera_x, camera_y, camera_z,
            camera_roll, camera_pitch, camera_yaw,
            'base_link', 'camera_link',
        ],
    )

    # ── Robot State Publisher ─────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # ── MoveGroup (MTC ExecuteTaskSolutionCapability) ─────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': ompl_planning_yaml},
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'},
            {'use_sim_time': True},
        ],
    )

    # ── RViz ─────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare('moveit_task_constructor_demo'),
        'config', 'mtc.rviz',
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': True},
        ],
    )

    # ── ros_gz_bridge: /clock ─────────────────────────────
    # Camera image/depth/camera_info topics are bridged by ROBOT_VISION's
    # vision_bringup.launch.py (via ur3_integrated.launch.py).
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_base',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    return [
        gz_sim_launch,
        static_tf_world,
        static_tf_camera,
        robot_state_pub,
        move_group_node,
        rviz_node,
        bridge_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('ur3_mtc_pick_place')

    # ── IGN_GAZEBO_RESOURCE_PATH: 패키지 내 models/ 디렉터리 ──
    # model:// URI (convenience_shelf, coke_can, fixed_rgbd_camera 등) 해석에 필요.
    # 외부 UR3_CONVENIENCE_ENV 환경 변수 불필요 — 모노레포 내장.
    models_path = os.path.join(pkg_share, 'models')
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path,
    )

    default_world = os.path.join(pkg_share, 'worlds', 'ur3_pick_place.sdf')

    return LaunchDescription([
        set_ign_resource_path,
        DeclareLaunchArgument('ur_type', default_value='ur3'),
        DeclareLaunchArgument(
            'world_file',
            default_value=default_world,
            description='Gazebo world SDF. Defaults to UR3_CONVENIENCE_ENV/worlds/ur3_pick_place.sdf.',
        ),
        # Camera TF — matches fixed_rgbd_camera at [1.2, 0.0, 0.5] yaw=PI in SDF.
        # Shelf surface z=0.34 m is visible from camera z=0.5 (within ±0.28 m FOV range).
        # Tune by checking: ros2 topic echo /vision/detection_results
        DeclareLaunchArgument('camera_x',     default_value='1.2'),
        DeclareLaunchArgument('camera_y',     default_value='0.0'),
        DeclareLaunchArgument('camera_z',     default_value='0.5'),
        DeclareLaunchArgument('camera_roll',  default_value='-1.5708'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw',   default_value='1.5708'),
        OpaqueFunction(function=launch_setup),
    ])
