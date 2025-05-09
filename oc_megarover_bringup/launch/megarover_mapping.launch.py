import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    config_dir = os.path.join(get_package_share_directory('oc_megarover_bringup'), 'param')
    config_file = os.path.join(config_dir, 'mapper_params_online_async.yaml')

    rviz_config_dir = os.path.join(get_package_share_directory('oc_megarover_bringup'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'mapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='sync_slam_toolbox_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, config_file]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]),

    ])
