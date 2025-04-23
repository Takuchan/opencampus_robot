#!/usr/bin/env python3
# 新しいアプローチングパーソンのプログラム

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
import cv2
import numpy as np
import pygame
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from oc_recognition_yolo_interfaces.msg import YOLODetection
from oc_approaching_interfaces.srv import CheckHand
from oc_tts_interfaces.srv import TTS

import math
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor


class ApproachingPerson(Node):
    def __init__(self):
        super().__init__('approaching_person_node')

        # 音楽ファイルパス
        sound_path = os.path.join(
            get_package_share_directory('oc_approching_me'),
            'sounds', 'downloaded_audio.wav'
        )

        # ──────────── パラメータ宣言 ────────────
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("yolo_topic", "/yolo_detection")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("idle_timeout", 30.0)
        self.declare_parameter("music_file", sound_path)
        self.declare_parameter("horizontal_fov_deg", 85.0)
        self.declare_parameter("vertical_fov_deg", 58.0)
        self.declare_parameter("marker_scale", 0.3)
        self.declare_parameter("marker_lifetime_sec", 130.0)
        self.declare_parameter("color_r", 1.0)
        self.declare_parameter("color_g", 0.0)
        self.declare_parameter("color_b", 0.0)
        self.declare_parameter("color_a", 0.8)
        # 移動を行わない最小距離（メートル）
        self.declare_parameter("min_move_distance", 0.5)
        # ───────────────────────────────────

        # ──────────── パラメータ取得 ────────────
        self.rgb_topic        = self.get_parameter("rgb_topic").value
        self.yolo_topic       = self.get_parameter("yolo_topic").value
        self.marker_frame_id  = self.get_parameter("marker_frame_id").value
        self.idle_timeout     = self.get_parameter("idle_timeout").value
        self.music_file       = self.get_parameter("music_file").value
        self.h_fov            = math.radians(self.get_parameter("horizontal_fov_deg").value)
        self.v_fov            = math.radians(self.get_parameter("vertical_fov_deg").value)
        self.marker_scale     = self.get_parameter("marker_scale").value
        self.marker_lifetime  = self.get_parameter("marker_lifetime_sec").value
        self.min_move_distance = self.get_parameter("min_move_distance").value
        # ───────────────────────────────────

        # OpenCV ブリッジ
        self.bridge = CvBridge()
        self.latest_rgb_image = None

        # ── 状態管理 ─────────────────────
        self.STATE_WAITING    = 0  # 待機（手検出待ち）
        self.STATE_COLLECTING = 1  # フレーム収集中
        self.STATE_MOVING     = 2  # ナビゲーション中
        self.current_state    = self.STATE_WAITING
        # ───────────────────────────────

        # フレームベースの収集バッファ
        self.collect_count      = 0
        self.collect_limit      = 100
        self.collected_positions: list[tuple[float, float, float]] = []

        # Subscriber / Publisher
        self.rgb_sub = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10
        )
        self.yolo_sub = self.create_subscription(
            YOLODetection, self.yolo_topic, self.yolo_detection_callback, 10
        )

        # Nav2 アクションクライアント
        self._nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 アクションサーバを待機中...')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # 手検出サービスクライアント
        self.hand_client = self.create_client(CheckHand, '/hand_detection/check_hand')
        while not self.hand_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('HandDetectionサービスを待機中...')

        # TTSサービスクライアント
        self.tts_client = self.create_client(TTS, 'tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTS サービスを待機中...')

        # Pygame 音楽再生初期化
        pygame.init()
        pygame.mixer.init()

        # 待機中アイドル発話タイマー
        self.idle_timer = self.create_timer(1.0, self.check_idle_state)
        self.last_activity_time = time.time()

        # OpenCV 表示用
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.namedWindow("ApproachingPerson", cv2.WINDOW_NORMAL)

        self.get_logger().info('ApproachingPerson ノード 初期化完了')

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'RGB画像変換エラー: {e}')

    def yolo_detection_callback(self, msg):
        # 移動中は完全にスキップ
        if self.current_state == self.STATE_MOVING:
            return
        if msg.object_class != "person" or self.latest_rgb_image is None:
            return

        # 検出領域切り出し
        x1, y1 = int(msg.startpoint.x), int(msg.startpoint.y)
        x2, y2 = int(msg.endpoint.x),   int(msg.endpoint.y)
        h, w, _ = self.latest_rgb_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        person_img = self.latest_rgb_image[y1:y2, x1:x2]

        self.check_hand_raising(person_img, msg)

    def check_hand_raising(self, image, detection_msg):
        # 移動中は無視
        if self.current_state == self.STATE_MOVING:
            return

        try:
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            req = CheckHand.Request()
            req.image = ros_img
            fut = self.hand_client.call_async(req)
            fut.add_done_callback(
                lambda f: self.hand_detection_callback(f, detection_msg, image)
            )
        except Exception as e:
            self.get_logger().error(f'手検出リクエストエラー: {e}')

    def hand_detection_callback(self, future, detection_msg, image):
        # 移動中は無視
        if self.current_state == self.STATE_MOVING:
            return

        try:
            res = future.result()
            if res.is_hand_raised:
                # 待機中なら収集開始
                if self.current_state == self.STATE_WAITING:
                    self.current_state = self.STATE_COLLECTING
                    self.collect_count = 0
                    self.collected_positions.clear()
                    self.speak("100フレーム分、手を挙げた位置を収集します。")

                # 収集中
                if self.current_state == self.STATE_COLLECTING:
                    cu = (detection_msg.startpoint.x + detection_msg.endpoint.x) * 0.5
                    cv = (detection_msg.startpoint.y + detection_msg.endpoint.y) * 0.5
                    fx = (detection_msg.width * 0.5) / math.tan(self.h_fov * 0.5)
                    fy = (detection_msg.height * 0.5) / math.tan(self.v_fov * 0.5)
                    z  = detection_msg.depth
                    x3 = z
                    y3 = -(cu - detection_msg.width * 0.5) * z / fx
                    z3 = -(cv - detection_msg.height * 0.5) * z / fy
                    self.collected_positions.append((x3, y3, z3))
                    self.collect_count += 1

                    if self.collect_count >= self.collect_limit:
                        self.finish_collecting()
            else:
                # デバッグ表示（挙手なし）
                disp = image.copy()
                cv2.putText(disp, "No hand raised", (10,30),
                            self.font, 0.7, (0,0,255), 2)
                cv2.imshow("ApproachingPerson", disp)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'手検出コールバックエラー: {e}')

    def finish_collecting(self):
        # データがなければ戻す
        if not self.collected_positions:
            self.current_state = self.STATE_WAITING
            return

        xs, ys, zs = zip(*self.collected_positions)
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_z = sum(zs) / len(zs)

        # 距離閾値チェック
        dist = math.hypot(avg_x, avg_y)
        if dist < self.min_move_distance:
            self.get_logger().info(f'平均距離 {dist:.2f}m → 近すぎるため移動中断')
            self.speak("近すぎるため移動を中止します。")
            self.current_state = self.STATE_WAITING
            return

        self.speak("データ取得完了。移動を開始します。")

        # ゴール生成
        goal = PoseStamped()
        goal.header.frame_id = self.marker_frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = avg_x
        goal.pose.position.y = avg_y
        goal.pose.position.z = avg_z
        goal.pose.orientation.w = 1.0
        self.latest_goal_pose = goal

        # 音楽＆ナビ開始
        self.play_music()
        self.navigate_to_person()
        self.current_state = self.STATE_MOVING
        self.last_activity_time = time.time()

    def navigate_to_person(self):
        if not hasattr(self, 'latest_goal_pose'):
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.latest_goal_pose
        fut = self._nav_action_client.send_goal_async(goal_msg)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.current_state = self.STATE_WAITING
            return
        handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        status = future.result().status
        if status == 4:  # 成功
            self.stop_music()
            self.speak("移動完了しました！")
        else:
            self.stop_music()
        self.current_state = self.STATE_WAITING
        self.last_activity_time = time.time()

    def play_music(self):
        try:
            pygame.mixer.music.load(self.music_file)
            pygame.mixer.music.play(-1)
        except Exception:
            pass

    def stop_music(self):
        try:
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
        except Exception:
            pass

    def check_idle_state(self):
        if self.current_state == self.STATE_WAITING:
            if time.time() - self.last_activity_time > self.idle_timeout:
                self.speak("手を挙げていただくと、そちらに向かいます。")
                self.last_activity_time = time.time()

    def speak(self, text):
        req = TTS.Request()
        req.text = text
        self.tts_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ApproachingPerson()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
