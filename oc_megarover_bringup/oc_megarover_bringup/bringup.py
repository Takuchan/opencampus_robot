import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')

        # /cmd_vel を購読
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # /rover_twist に転送
        self.publisher = self.create_publisher(
            Twist,
            'rover_twist',
            10
        )

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Relaying Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
