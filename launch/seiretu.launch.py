#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch the seiretu node
        Node(
            package='seiretu',
            executable='seiretu_node',
            name='seiretu_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),
    ])