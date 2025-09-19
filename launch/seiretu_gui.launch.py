#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch the original seiretu node
        Node(
            package='seiretu',
            executable='seiretu_node',
            name='seiretu_node',
            output='screen'
        ),

        # Launch the GUI
        Node(
            package='seiretu',
            executable='seiretu_gui',
            name='seiretu_gui',
            output='screen'
        )
    ])