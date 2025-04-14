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
        )
    ])