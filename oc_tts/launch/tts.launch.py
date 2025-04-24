#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oc_tts',
            executable='tts_server',
            name='tts_server',
            output='screen',
        ),
    ])