from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    print("おはよう")

    return LaunchDescription([
        # ① TF: camera_init を base_link に固定（位置・回転 = 0）
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '0', '0', '0',
                'base_link', 'camera_init'
            ],
            output='screen'
        ),

        # ② TF: front_lrf_link を base_link の前（例: x=0.2）
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.2', '0', '0', '0', '0', '0',
                'base_link', 'front_lrf_link'
            ],
            output='screen'
        ),

        # ③ TF: back_lrf_link を base_link の後（例: x=-0.2）
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '-0.2', '0', '0', '0', '0', '0',
                'base_link', 'back_lrf_link'
            ],
            output='screen'
        ),

        Node(
            package='oc_fl_nav2_helper',
            executable='fl_scanconverter',
            name='fl_scanconverter',
            output='screen'
        )
        # ④ pointcloud_to_laserscan ノード
        # Node(
        #     package='pointcloud_to_laserscan',
        #     executable='pointcloud_to_laserscan_node',
        #     remappings=[
        #         ('cloud_in', '/converted_pointcloud2'),
        #         ('scan', '/scan')
        #     ],
        #     parameters=[{
        #         'target_frame': 'camera_init',
        #         'transform_tolerance': 0.01,
        #         'min_height': -0.2,
        #         'max_height': 0.2,
        #         'angle_min': -3.1415,
        #         'angle_max': 3.1415,
        #         'angle_increment': 0.0174,
        #         'scan_time': 0.1,
        #         'range_min': 0.1,
        #         'range_max': 70.0,
        #         'use_inf': True,
        #         'inf_epsilon': 1.0
        #     }],
        #     name='pointcloud_to_laserscan'
        # )
    ])
