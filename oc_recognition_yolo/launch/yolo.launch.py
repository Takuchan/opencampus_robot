#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oc_recognition_yolo',
            executable='yolo_publisher',
            name='yolo_publisher',
            output='screen',
            parameters=[
                {"model_path": "yolo11n.pt",
                 "confidence_threshold": 0.5,
                 "target_class": "person",
                 "image_topic": "/camera/camera/color/image_raw",
                 "output_topic": "/yolo/detection_image"},
                 "frame_id","realsense"
            ]
        ),
        # Node(
        #     package='oc_recognition_yolo',         # DetectionVisualizerノードを含むパッケージ名に書き換え
        #     executable='yolo_realsense_depth_publisher',     # ビルド時に生成された実行ファイル名
        #     name='detection_visualizer',
        #     output='screen',
        #     parameters=[{
        #         'camera_fov_deg': 90.0,                   # カメラの水平視野角（度単位）
        #         'marker_topic': '/visualization_marker',  # RVizに表示するMarkerのトピック
        #         'target_frame': 'base_footprint'          # TF変換先の座標系
        #     }]
        # )
    ])