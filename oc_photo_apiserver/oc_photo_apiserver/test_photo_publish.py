#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # トピック 'oc/image/image_raw' に Image 型のメッセージを発行するパブリッシャを作成
        self.publisher_ = self.create_publisher(Image, 'oc/image/image_raw', 10)
        # 1秒ごとに timer_callback を呼び出すタイマーを設定
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info("Image publisher started.")

    def timer_callback(self):
        # 480x640 の3チャンネル（BGR）の黒画像を生成
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # OpenCVの画像をROSのImageメッセージに変換（encodingは"bgr8"）
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        # ヘッダ情報の設定（タイムスタンプ、フレームIDなど）
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        # トピックへ画像を発行
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing image")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
