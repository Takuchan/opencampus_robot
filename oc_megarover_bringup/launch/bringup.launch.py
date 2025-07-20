#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    
    return LaunchDescription([
         # TF静的変換ノード (base_link → mid360)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_livox_publisher',
            arguments=[
                '-0.25',    # X (meters)
                '0.1',    # Y (meters)
                '0.58',    # Z (meters) - 高さに合わせて調整
                '0',  # Roll (radians) ≒ 90度
                '0',     # Pitch (radians)
                '0',  # Yaw (radians) ≒ 180度  ←ここを追加！
                'base_footprint',
                'realsense'
            ],
            output='screen'
        ),
        Node(
            package='oc_megarover_bringup',  # ここを実際のパッケージ名に置換
            executable='bringup',
            name='reverse_twist_relay_node',   # ノード名（任意の名前）
            output='screen',
        ),
        
    ])
