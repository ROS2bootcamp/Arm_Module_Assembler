"""Launch the MoveIt Module node.

MoveItPy reads robot_description / _semantic / _kinematics + the planning
pipeline from this node's parameters, so we inject them here (same sources as
the bringup launch). Run AFTER ur3_moveit_bringup.launch.py (real mode), or
with mock:=true to test the /moveit/execute service handshake with the
LLM Agent only (no Gazebo / move_group needed).

    # real motion (move_group + Gazebo already up)
    ros2 launch ur3_moveit_module moveit_module.launch.py

    # mock mode (no MoveIt / Gazebo needed)
    ros2 launch ur3_moveit_module moveit_module.launch.py mock:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    mock = LaunchConfiguration("mock").perform(context)
    is_mock = mock.lower() in ("true", "1", "yes")

    module_params = PathJoinSubstitution([
        FindPackageShare("ur3_moveit_module"), "config", "module_params.yaml"
    ])

    params = [module_params, {"mock": is_mock, "use_sim_time": True}]

    # In real mode, MoveItPy needs the robot model parameters on this node.
    if not is_mock:
        robot_description_content = Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("ur3_mtc_pick_place"),
                "urdf", "ur3_with_gripper.urdf.xacro",
            ]),
            " ur_type:=", ur_type,
        ])
        robot_description_semantic_content = Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("ur3_mtc_pick_place"),
                "srdf", "ur3_with_gripper.srdf.xacro",
            ]),
            " name:=ur ",
        ])
        kinematics_yaml = PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"
        ])
        ompl_planning_yaml = PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "config", "ompl_planning.yaml"
        ])
        params = [
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)},
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": ompl_planning_yaml},
        ] + params

    module_node = Node(
        package="ur3_moveit_module",
        executable="moveit_module_node",
        name="moveit_module",
        output="screen",
        parameters=params,
    )
    return [module_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3"),
        DeclareLaunchArgument("mock", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
