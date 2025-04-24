# marker_nav2_commander.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.action

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

class MarkerNav2Commander(Node):
    def __init__(self):
        super().__init__('marker_nav2_commander')
        self.latest_marker: Marker = None

        # マーカー購読
        self.create_subscription(Marker, 'visualization_marker', self._marker_cb, 10)

        # Trigger サービス
        self.create_service(Trigger, 'go_to_marker', self._on_go_to_marker)

        # Nav2 アクションクライアント
        self._nav_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("MarkerNav2Commander ready. Call 'go_to_marker' to navigate.")

    def _marker_cb(self, msg: Marker):
        self.latest_marker = msg
        self.get_logger().debug(f"Received Marker @[{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]")

    def _on_go_to_marker(self, request, response):
        if self.latest_marker is None:
            response.success = False
            response.message = "マーカー未受信"
            return response

        # PoseStamped に変換
        goal_pose = PoseStamped()
        goal_pose.header = self.latest_marker.header
        goal_pose.pose   = self.latest_marker.pose

        # サーバ待ち
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = "Nav2 action server unreachable"
            return response

        # ゴール送信
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_fut = self._nav_client.send_goal_async(goal)
        send_fut.add_done_callback(self._on_goal_response)

        response.success = True
        response.message = "ゴール送信済み"
        return response

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal was rejected")
            return
        self.get_logger().info("Goal accepted, waiting result...")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerNav2Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
