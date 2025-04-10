#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory

class SoundTestNode(Node):
    def __init__(self):
        super().__init__('sound_test_node')
        sound_path = os.path.join(
            get_package_share_directory('oc_denger_detection'),
            'sounds',
            'uwabikkurishita.wav'
        )
        self.get_logger().info(f"🔊 音声ファイルを再生します: {sound_path}")
        os.system(f'aplay {sound_path}')
        self.get_logger().info("✅ 再生完了！")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SoundTestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
