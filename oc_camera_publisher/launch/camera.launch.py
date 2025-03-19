from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory('oc_camera_publisher'),
        'camera_params.yaml'
    )

    print(camera_params)

    return LaunchDescription([
        Node(
            package='oc_camera_publisher',
            executable='camera_node',
            output='screen',
            parameters=[camera_params]
        )
    ])
