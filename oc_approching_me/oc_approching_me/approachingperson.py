#!/usr/bin/env python3
# 新しいアプローチングパーソンのプログラム

import rclpy
from rclpy.node import Node
import rclpy.action                                      # ← 追加
from rclpy.action import ActionClient                     # ← 追加
from nav2_msgs.action import NavigateToPose               # ← 追加
from geometry_msgs.msg import PoseStamped                 # ← 追加
import time
import cv2
import numpy as np
from threading import Thread
import pygame
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from oc_recognition_yolo_interfaces.msg import YOLODetection
from oc_approaching_interfaces.srv import CheckHand
from oc_tts_interfaces.srv import TTS

import math
import os 
from ament_index_python.packages import get_package_share_directory


from nav2_msgs.action import NavigateToPose

from action_msgs.msg import GoalStatus

class ApproachingPerson(Node):
    def __init__(self):
        super().__init__('approaching_person_node')

        sound_path = os.path.join(
            get_package_share_directory('oc_approching_me'),
            'sounds',
            'downloaded_audio.wav'
        )

        # パラメータ宣言
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("yolo_topic", "/yolo_detection")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("idle_timeout", 30.0)  # 待機状態が続くと発話する時間（秒）
        self.declare_parameter("music_file", sound_path)
        # Realsense D455 の一般的な画角をパラメータとして指定
        self.declare_parameter("horizontal_fov_deg", 85.0)  # 水平方向の画角（度）
        self.declare_parameter("vertical_fov_deg", 58.0)      # 垂直方向の画角（度）
        # マーカーの大きさ（球の直径）と表示時間（marker.lifetime）
        self.declare_parameter("marker_scale", 0.3)
        self.declare_parameter("marker_lifetime_sec", 3.0)    # RViz上に残る時間（秒）
        # マーカーの色（RGBA）
        self.declare_parameter("color_r", 1.0)
        self.declare_parameter("color_g", 0.0)
        self.declare_parameter("color_b", 0.0)
        self.declare_parameter("color_a", 0.8)
        
        # パラメータ取得
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.yolo_topic = self.get_parameter("yolo_topic").value
        self.marker_frame_id = self.get_parameter("marker_frame_id").value
        self.idle_timeout = self.get_parameter("idle_timeout").value
        self.music_file = self.get_parameter("music_file").value
        self.h_fov = math.radians(self.get_parameter("horizontal_fov_deg").value)
        self.v_fov = math.radians(self.get_parameter("vertical_fov_deg").value)
        self.marker_scale = self.get_parameter("marker_scale").value
        self.marker_lifetime = self.get_parameter("marker_lifetime_sec").value
        
        # OpenCVブリッジ初期化
        self.bridge = CvBridge()
        
        # 最新の画像を保持する変数
        self.latest_rgb_image = None
        
        # 状態管理
        self.STATE_WAITING = 0     # 手を上げた人を探している状態
        self.STATE_MOVING = 1      # 移動中の状態
        self.current_state = self.STATE_WAITING
        
        # サブスクリプションとパブリッシャ
        self.rgb_sub = self.create_subscription(
            Image, 
            self.rgb_topic, 
            self.rgb_callback, 
            10)
            
        self.yolo_sub = self.create_subscription(
            YOLODetection, 
            self.yolo_topic, 
            self.yolo_detection_callback, 
            10)
        


        # Nav2 のアクションクライアントを作成
        self._nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        # サーバ起動を待つ
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 アクションサーバを待機中...')

        # マーカー発行後に目標Poseを格納しておく変数
        self.latest_goal_pose: PoseStamped = None


        # マーカーパブリッシャー
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # 手の検出サービスクライアント
        self.hand_detection_client = self.create_client(CheckHand, '/hand_detection/check_hand')
        while not self.hand_detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('HandDetectionサービスを待機中...')
    
        # 音声合成クライアント
        self.tts_client = self.create_client(TTS, 'tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTS サービスを待機中...')
        
        # 音楽プレーヤー初期化
        pygame.mixer.init()
        
        # タイマー（待機状態でのアイドル発話用）
        self.idle_timer = self.create_timer(1.0, self.check_idle_state)
        self.last_activity_time = time.time()
        
        # 表示用のフォント設定
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        self.get_logger().info('ApproachingPerson ノードを初期化しました')
        cv2.namedWindow("ApproachingPerson", cv2.WINDOW_NORMAL)

    def rgb_callback(self, msg):
        """RGB画像を受信してキャッシュする"""
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'RGB画像変換エラー: {e}')
    
    def yolo_detection_callback(self, msg):
        """YOLODetectionメッセージを受信して処理する"""
        if self.current_state != self.STATE_WAITING:
            # 移動中は検出処理をスキップ
            return
            
        # personクラスのみを処理
        if msg.object_class != "person":
            return
            
        self.get_logger().info(f'Person検出: 座標({msg.startpoint.x}, {msg.startpoint.y}) - ({msg.endpoint.x}, {msg.endpoint.y}), 深度: {msg.depth}m')
        
        # 最新のRGB画像があるか確認
        if self.latest_rgb_image is None:
            self.get_logger().warn('RGB画像がまだありません')
            return
            
        # 検出された人物の領域を切り取る
        try:
            x1, y1 = int(msg.startpoint.x), int(msg.startpoint.y)
            x2, y2 = int(msg.endpoint.x), int(msg.endpoint.y)
            
            # 画像サイズの範囲内に収める
            h, w, _ = self.latest_rgb_image.shape
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w, x2)
            y2 = min(h, y2)
            
            # 領域を切り取る
            person_image = self.latest_rgb_image[y1:y2, x1:x2]
            
            # 手の検出処理を実行
            self.check_hand_raising(person_image, msg)
            
        except Exception as e:
            self.get_logger().error(f'人物画像抽出エラー: {e}')
    
    def check_hand_raising(self, image, detection_msg):
        """手を挙げているか検出する"""
        try:
            # 画像をROS Image形式に変換
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            
            # リクエスト作成
            request = CheckHand.Request()
            request.image = ros_image
            
            # サービス呼び出し
            future = self.hand_detection_client.call_async(request)
            future.add_done_callback(
                lambda f: self.hand_detection_callback(f, detection_msg, image))
                
        except Exception as e:
            self.get_logger().error(f'手の検出リクエストエラー: {e}')
    
    def hand_detection_callback(self, future, detection_msg, person_image):
        """手の検出結果コールバック"""
        try:
            response = future.result()
            
            if response.is_hand_raised:
                self.get_logger().info('🙋 手を挙げている人を検出しました！移動を開始します')
                
                # 状態を移動中に変更
                self.current_state = self.STATE_MOVING
                
                # マーカーを作成して発行
                self.publish_marker(detection_msg)
                
                # 音楽再生開始
                self.play_music()
                
                # 自己位置から目標位置への移動をリクエスト
                self.navigate_to_person()
                
                # デバッグ表示
                display_img = person_image.copy()
                cv2.putText(display_img, "HAND RAISED! MOVING...", (10, 30), 
                           self.font, 0.7, (0, 255, 0), 2)
                cv2.imshow("ApproachingPerson", display_img)
                cv2.waitKey(1)
            else:
                # デバッグ表示 (手は挙がっていない)
                display_img = person_image.copy()
                cv2.putText(display_img, "No hand raised", (10, 30), 
                           self.font, 0.7, (0, 0, 255), 2)
                cv2.imshow("ApproachingPerson", display_img)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'手の検出結果処理エラー: {e}')
    
    def publish_marker(self, detection_msg):
        """検出位置にマーカーを発行する"""

        center_u = (detection_msg.startpoint.x + detection_msg.endpoint.x) /2
        center_v = (detection_msg.startpoint.y + detection_msg.endpoint.y) /2
        image_width = detection_msg.width
        image_height = detection_msg.height

        #商店距離
        fx = (image_width /2 )/ math.tan(self.h_fov /2)
        fy = (image_height/2) / math.tan(self.v_fov/2)

        # カメラ高額フレームでの座標計算
        x_opt = (center_u - image_width /2 ) * detection_msg.depth /fx
        y_opt = (center_v - image_height /2) * detection_msg.depth / fy
        z_opt = detection_msg.depth

        x_robot = z_opt
        y_robot = - x_opt
        z_robot = - y_opt

        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "approaching_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x_robot
        marker.pose.position.y = y_robot
        marker.pose.position.z = z_robot
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # マーカーサイズ
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 色 (赤)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        self.marker_pub.publish(marker)
        self.get_logger().info(f'マーカーを発行しました: x={marker.pose.position.x}, y={marker.pose.position.y}')
        goal = PoseStamped()
        goal.header = marker.header
        goal.pose = marker.pose
        self.latest_goal_pose = goal


    def navigate_to_person(self):
        """Nav2 アクションでマーカー位置へ移動"""
        if self.latest_goal_pose is None:
            self.get_logger().error('目標Poseがセットされていません')
            return

        # ゴールメッセージを作成
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.latest_goal_pose

        # アクション送信
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.on_goal_response)

    
    def on_goal_response(self, future):
        """ゴール送信後のレスポンス処理"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('移動ゴールが拒否されました')
            # 状態を待機に戻す
            self.current_state = self.STATE_WAITING
            return

        self.get_logger().info('移動ゴールが受理されました。結果を待機します...')
        # 結果取得をリクエスト
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """移動アクション結果のコールバック"""
        response = future.result()
        status = response.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('ナビゲーション完了！')
            # 音楽停止
            self.stop_music()
            # TTSで完了メッセージ
            self.speak("移動完了しました！")
        else:
            self.get_logger().error(f'ナビゲーション失敗:')
            self.stop_music()

        # 状態を待機に戻してアイドルタイマーをリセット
        self.current_state = self.STATE_WAITING
        self.last_activity_time = time.time()

    def play_music(self):
        """移動中の音楽再生"""
        try:
            pygame.mixer.music.load(self.music_file)
            pygame.mixer.music.play(-1)  # -1でループ再生
            self.get_logger().info('移動中の音楽を再生します')
        except Exception as e:
            self.get_logger().error(f'音楽再生エラー: {e}')
    
    def stop_music(self):
        """音楽停止"""
        try:
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
                self.get_logger().info('音楽を停止しました')
        except Exception as e:
            self.get_logger().error(f'音楽停止エラー: {e}')
    
    def check_idle_state(self):
        """待機状態が続いた場合のアイドル発話チェック"""
        if self.current_state == self.STATE_WAITING:
            elapsed = time.time() - self.last_activity_time
            if elapsed > self.idle_timeout:
                self.idle_speech()
                self.last_activity_time = time.time()  # タイマーリセット
    
    def idle_speech(self):
        """アイドル状態での発話"""
        idle_messages = [
            "手を挙げた人を探しています。誰かいますか？",
            "こんにちは、私は自律移動ロボットです。",
            "手を挙げていただくと、そちらに向かいます。",
            "お手伝いできることはありませんか？",
            "何かお困りでしたら手を挙げてください。"
        ]
        # ランダムに発話文を選択
        message = np.random.choice(idle_messages)
        self.speak(message)
    
    def speak(self, text):
        """TTSで発話する"""
        request = TTS.Request()
        request.text = text
        
        self.get_logger().info(f'発話: {text}')
        
        # 非同期でTTSサービス呼び出し
        future = self.tts_client.call_async(request)
        future.add_done_callback(self.tts_callback)
    
    def tts_callback(self, future):
        """TTSコールバック"""
        try:
            response = future.result()
            self.get_logger().info(f'TTS応答: {response.message}')
        except Exception as e:
            self.get_logger().error(f'TTS呼び出しエラー: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ApproachingPerson()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ノードを終了します')
        # クリーンアップ
        if pygame.mixer.get_init() and pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
    finally:
        # ウィンドウを閉じる
        cv2.destroyAllWindows()
        # ノード破棄
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()