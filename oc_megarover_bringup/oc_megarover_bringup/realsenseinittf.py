#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class RealsenseTFPublisher(Node):
    def __init__(self):
        super().__init__('realsense_tf_init')
        self.br = tf2_ros.StaticTransformBroadcaster(self)
        
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "base_footprint"           # 親フレーム
        static_transformStamped.child_frame_id = "realsense_frame"    # 子フレーム（Realsenseのカメラフレーム）
        
        # 例：カメラはロボット前方0.1m、上方0.2mに取り付けられているとする
        static_transformStamped.transform.translation.x = 0.1   # 前方オフセット
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.2   # 高さオフセット
        
        # 回転はここでは無し（単位クォータニオン：0,0,0,1） – 必要に応じて調整してください
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0
        
        self.br.sendTransform(static_transformStamped)
        self.get_logger().info("Static transform published: base_link -> realsense_frame")

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
