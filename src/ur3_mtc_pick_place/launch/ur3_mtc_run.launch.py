"""
MTC Pick & Place 태스크 노드 실행 런치 파일
터미널 2에서 실행 (터미널 1: ur3_mtc_demo.launch.py 먼저 실행)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            description="UR 로봇 타입",
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="true",
            description="계획 후 자동 실행 여부 (true/false)",
        ),

        Node(
            package="ur3_mtc_pick_place",
            executable="ur3_pick_place.py",
            name="ur3_pick_place",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("ur3_mtc_pick_place"),
                    "config", "ur3_mtc_config.yaml",
                ]),
                {
                    "use_sim_time": True,
                    "execute": LaunchConfiguration("execute"),
                },
            ],
        ),
    ])
