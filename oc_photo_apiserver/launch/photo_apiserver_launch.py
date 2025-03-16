import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # FastAPIサーバーを起動するコマンド
    package_share_directory = get_package_share_directory('oc_photo_apiserver')
    fastapi_script_path = os.path.join(package_share_directory,'fastapi_image_server.py')

    fastapi_server = ExecuteProcess(
        cmd=['python3', fastapi_script_path],
        output='screen'
    )

    # PhotoClient ノードを起動
    photo_client = Node(
        package='oc_photo_apiserver',
        executable='photoclient',
        name='photo_client',
        output='screen'
    )

    return LaunchDescription([
        fastapi_server,
        photo_client
    ])
