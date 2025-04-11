from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # ローンチ引数を宣言
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='scan',
        description='Conversion mode: "scan" for LaserScan output or "pointcloud" for PointCloud2 output'
    )
    
    # TF静的変換ノード (base_link → mid360)
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_livox_publisher',
        arguments=[
            '0.15',    # X (meters)
            '0.0',    # Y (meters)
            '0.2',    # Z (meters) - 高さに合わせて調整
            '0',  # Roll (radians) ≒ 90度
            '0',     # Pitch (radians)
            '0',  # Yaw (radians) ≒ 180度  ←ここを追加！
            'base_footprint',
            'mid360'
        ],
        output='screen'
    )

    
    # PointCloud2変換ノード
    pointcloud_node = Node(
        package='oc_livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2_node',
        parameters=[{
            'frame_id': 'mid360'  # 出力のframe_idを設定
        }],
        remappings=[
            ('/livox_pointcloud', '/livox/lidar'),
            ('/converted_pointcloud2', '/livox/lidar/pcd2')
        ],
    )

    # LaserScan変換ノード (デフォルト)
    scan_node = Node(
        package='oc_livox_to_pointcloud2',
        executable='livox_to_scan_node',
        name='livox_to_scan_node',
        parameters=[{
            'frame_id': 'mid360'  # 出力のframe_idを設定
        }],
        remappings=[
            ('/livox_pointcloud', '/livox/lidar'),
            ('/scan', '/scan')
        ],
    )

    return LaunchDescription([
        mode_arg,
        tf_node,        # TF静的変換を追加
        pointcloud_node,
        scan_node
    ])
