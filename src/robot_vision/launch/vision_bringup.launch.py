"""Launch the ROBOT_VISION YOLO detection stack.

Standalone mode (default):
    ros2 launch robot_vision vision_bringup.launch.py
    → starts Ignition Gazebo (camera_test.sdf) + ros_gz_bridge + YOLO node

Integration mode (used by ur3_integrated.launch.py):
    ros2 launch robot_vision vision_bringup.launch.py launch_gazebo:=false
    → starts bridge + YOLO node only (Gazebo is managed by the parent launch)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_vision')
    world_file = os.path.join(pkg_share, 'worlds', 'camera_test.sdf')

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Launch Ignition Gazebo. Set false when Gazebo is managed '
                    'by a parent launch (e.g. ur3_integrated.launch.py).',
    )

    gazebo_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        cmd=['ign', 'gazebo', world_file],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            # fixed_rgbd_camera.sdf type="camera" sensor publishes camera_info
            # at /camera/image/camera_info (not /camera/camera_info)
            '/camera/image/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
    )

    yolo_node = Node(
        package='robot_vision',
        executable='yolo_node',
        output='screen',
    )

    return LaunchDescription([
        launch_gazebo_arg,
        gazebo_process,
        bridge_node,
        yolo_node,
    ])
