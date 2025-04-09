<<<<<<< HEAD
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ReverseTwistRelayNode(Node):
    def __init__(self):
        super().__init__('reverse_twist_relay_node')
=======
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
>>>>>>> main

        # /cmd_vel を購読
        self.subscription = self.create_subscription(
            Twist,
<<<<<<< HEAD
            'cmd_vel',
            self.twist_callback,
            10
        )

        # /rover_twist を送信
=======
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # /rover_twist に転送
>>>>>>> main
        self.publisher = self.create_publisher(
            Twist,
            'rover_twist',
            10
        )

<<<<<<< HEAD
        self.get_logger().info('Reverse twist relay node started: cmd_vel → rover_twist')

    def twist_callback(self, msg):
        # 受け取ったTwistをそのままrover_twistに流す
=======
    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Relaying Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
>>>>>>> main
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
<<<<<<< HEAD
    node = ReverseTwistRelayNode()
=======
    node = TwistRelay()
>>>>>>> main
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
