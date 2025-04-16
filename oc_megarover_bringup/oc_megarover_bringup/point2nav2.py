#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.action

# Nav2の移動アクション
from nav2_msgs.action import NavigateToPose
# MarkerやPoseStampedのメッセージ
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
# サービスでGoサインを受け取るためのTriggerサービス
from std_srvs.srv import Trigger

class MarkerNav2Commander(Node):
    def __init__(self):
        super().__init__('marker_nav2_commander')
        
        # 最新のマーカー情報を格納する変数
        self.latest_marker = None
        
        # visualization_markerトピックからマーカー情報を受信
        self.marker_sub = self.create_subscription(
            Marker,
            'visualization_marker',
            self.marker_callback,
            10
        )
        
        # NavigateToPoseアクションサーバへのアクションクライアント
        self._action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # サービスを提供：TriggerサービスでGoサインが出たら移動開始
        self.srv = self.create_service(Trigger, 'go_to_marker', self.handle_go_to_marker)
        self.get_logger().info("MarkerNav2Commanderノードを開始しました。サービス『go_to_marker』の呼び出しで移動ゴールを送信します。")
    
    def marker_callback(self, msg: Marker):
        """ visualization_markerから受信した最新のマーカー情報を保存 """
        self.latest_marker = msg
        self.get_logger().info("受信マーカー id {}: 座標 [x: {:.2f}, y: {:.2f}, z: {:.2f}]".format(
            msg.id, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ))
    
    def handle_go_to_marker(self, request, response):
        """
        Goサイン（Triggerサービス）を受けたときに、
        最新のマーカー情報をもとに移動ゴールを送信する
        """
        if self.latest_marker is None:
            response.success = False
            response.message = "まだマーカー情報が受信されていません。"
            self.get_logger().error("サービス要求失敗: マーカー未受信")
            return response
        
        # マーカーのヘッダーを使いPoseStampedを作成する
        goal_pose = PoseStamped()
        goal_pose.header = self.latest_marker.header
        goal_pose.pose = self.latest_marker.pose
        
        # Nav2のアクションサーバが立ち上がっているか待つ
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ナビゲーションアクションサーバに接続できませんでした。")
            response.success = False
            response.message = "ナビゲーションアクションサーバに接続できませんでした。"
            return response
        
        self.get_logger().info("マーカーの座標に向けて移動ゴールを送信します。")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 非同期でゴール送信
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        response.success = True
        response.message = "移動ゴールを送信しました。"
        return response
    
    def goal_response_callback(self, future):
        """ ゴール送信結果のコールバック """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("ナビゲーションゴールが拒否されました。")
            return
        
        self.get_logger().info("ナビゲーションゴールが受理されました。結果を待っています...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """ ナビゲーション結果のコールバック """
        result = future.result().result
        self.get_logger().info("ナビゲーション結果: {}".format(result))
        
def main(args=None):
    rclpy.init(args=args)
    node = MarkerNav2Commander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MarkerNav2Commanderノードを終了します。")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
