#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ROSパッケージの共有ディレクトリを取得するためのヘルパ
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # このパッケージ名（例: oc_megarover_bringup）に合わせて変更してください
    package_name = 'oc_megarover_bringup'
    pkg_share = get_package_share_directory(package_name)

    # パラメータやマップのファイルパスを取得
    default_map_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_nav2_params = os.path.join(pkg_share, 'param', 'navigation_param.yaml')

    # Launch 引数を定義 (必要に応じて使う)
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to the ROS2 Navigation param file'
    )

    # 取得した引数を LaunchConfiguration に紐付け
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # map_server ノード
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # AMCL ノード (自己位置推定)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Planner Server ノード (経路プラン)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # Controller Server ノード (経路追従)
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    # ライフサイクルマネージャ (上記ノードのライフサイクル管理)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server'
            ]
        }]
    )

    return LaunchDescription([
        map_yaml_arg,
        nav2_params_arg,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        lifecycle_manager
    ])
