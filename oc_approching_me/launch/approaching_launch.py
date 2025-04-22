#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ApproachingPerson のパッケージパスを取得
    pkg_approaching = get_package_share_directory('oc_approching_me')

    # TTSパッケージを取得
    tts_pkg = get_package_share_directory('oc_tts')
    default_params_file = os.path.join(pkg_approaching, 'config', 'approaching_params.yaml')

    # YOLO 認識のパッケージパスを取得
    pkg_yolo = get_package_share_directory('oc_recognition_yolo')
    

    # パラメータファイルの宣言
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Approaching node のパラメータファイルの完全なパス'
    )
    default_music_file = os.path.join(
        get_package_share_directory('oc_approching_me'),
        'sounds',
        'runnning.mp3'
    )
    # 音楽ファイルパスの宣言
    music_arg = DeclareLaunchArgument(
        'music_file',
        default_value=default_music_file,
        description='移動中に再生する音楽ファイルのパス'
    )
    
    # YOLOノードの起動
    yolo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolo, 'launch', 'yolo.launch.py')
        )
    )
    
    # TTSノードの起動
    tts_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tts_pkg,'launch','tts.launch.py')
        )
    )
    # 手検出サービスノード
    hand_detection_node = Node(
        package='oc_approching_me',
        executable='raisehands',
        name='hand_detection_service',
        output='screen'
    )
    
    # TTS サービスノード
    tts_node = Node(
        package='oc_tts',
        executable='tts_server',
        name='tts_server',
        output='screen'
    )
    
    # アプローチングパーソンノード
    approaching_node = Node(
        package='oc_approching_me',
        executable='approaching_person',
        name='approaching_person_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'music_file': LaunchConfiguration('music_file')}
        ]
    )
    
    return LaunchDescription([
        params_arg,
        music_arg,
        yolo_node,
        hand_detection_node,
        tts_node,
        approaching_node
    ])