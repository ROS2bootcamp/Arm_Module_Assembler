"""UR3 + Robotiq 2F-85 bringup: Gazebo + ros2_control + move_group (+RViz).

Adapted from PANDA_ENV/ur3_mtc_pick_place/launch/ur3_mtc_demo.launch.py. Brings
up everything the MoveIt Module needs to plan & execute. Run the module itself
separately with moveit_module.launch.py (or include it here).

    ros2 launch ur3_moveit_module ur3_moveit_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    # Robot description (UR3 + Robotiq) — reuse the verified xacro from the
    # ur3_mtc_pick_place package (PANDA_ENV) which the environment provides.
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("ur3_mtc_pick_place"),
            "urdf", "ur3_with_gripper.urdf.xacro",
        ]),
        " ur_type:=", ur_type,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("ur3_mtc_pick_place"),
            "srdf", "ur3_with_gripper.srdf.xacro",
        ]),
        " name:=", ur_type,
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"
    ])
    ompl_planning_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "ompl_planning.yaml"
    ])

    # Gazebo + ros2_control
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_simulation_gz"),
            "/launch/ur_sim_control.launch.py",
        ]),
        launch_arguments={"ur_type": ur_type, "launch_rviz": "false"}.items(),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": ompl_planning_yaml},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            {"use_sim_time": True},
        ],
    )

    nodes = [gz_sim_launch, static_tf, robot_state_pub, move_group_node]

    if launch_rviz.lower() in ("true", "1", "yes"):
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            parameters=[
                robot_description,
                robot_description_semantic,
                {"robot_description_kinematics": kinematics_yaml},
                {"use_sim_time": True},
            ],
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3",
                              description="UR robot type (ur3, ur3e, ...)"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
