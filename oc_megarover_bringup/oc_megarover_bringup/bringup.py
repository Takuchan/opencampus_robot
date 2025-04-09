#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ReverseTwistRelayNode(Node):
    def __init__(self):
        super().__init__('reverse_twist_relay_node')

        # /cmd_vel を購読
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        # /rover_twist を送信
        self.publisher = self.create_publisher(
            Twist,
            'rover_twist',
            10
        )

        self.get_logger().info('Reverse twist relay node started: cmd_vel → rover_twist')

    def twist_callback(self, msg):
        # 受け取ったTwistをそのままrover_twistに流す
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReverseTwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
