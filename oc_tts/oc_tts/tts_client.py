#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from oc_recognition_yolo_interfaces.msg import YOLODetection
from oc_approaching_interfaces.srv import CheckHand
from oc_tts_interfaces.srv import TTS
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import os
import time
import pygame
from ament_index_python.packages import get_package_share_directory

class ApproachingPerson(Node):
    def __init__(self):
        super().__init__('approaching_person_node')

        # 音楽ファイルパス
        sound_path = os.path.join(
            get_package_share_directory('oc_approching_me'),
            'sounds',
            'downloaded_audio.wav'
        )

        # CameraInfo と TF2
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 挙手継続検出用変数
        self.raise_start_time: float = None
        self.last_raise_center: tuple = None
        self.raise_duration_threshold: float = 1.0   # 秒
        self.center_move_threshold: float = 20.0     # px

        # パラメータ宣言
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("yolo_topic", "/yolo_detection")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("idle_timeout", 30.0)
        self.declare_parameter("music_file", sound_path)
        self.declare_parameter("marker_scale", 0.3)
        self.declare_parameter("marker_lifetime_sec", 3.0)
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
        self.marker_scale = self.get_parameter("marker_scale").value
        self.marker_lifetime = self.get_parameter("marker_lifetime_sec").value

        # OpenCV ブリッジ
        self.bridge = CvBridge()
        self.latest_rgb_image = None

        # 状態管理
        self.STATE_WAITING = 0
        self.STATE_MOVING = 1
        self.current_state = self.STATE_WAITING

        # トピック購読・発行
        self.rgb_sub = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10)
        self.yolo_sub = self.create_subscription(
            YOLODetection, self.yolo_topic, self.yolo_detection_callback, 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Nav2 アクションクライアント
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 アクションサーバを待機中...')

        self.latest_goal_pose: PoseStamped = None

        # 手挙げ判定サービスクライアント
        self.hand_detection_client = self.create_client(CheckHand, '/hand_detection/check_hand')
        while not self.hand_detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('HandDetectionサービスを待機中...')

        # TTS サービスクライアント
        self.tts_client = self.create_client(TTS, 'tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTS サービスを待機中...')

        # 音楽プレーヤー初期化
        pygame.mixer.init()

        # アイドルタイマー
        self.idle_timer = self.create_timer(1.0, self.check_idle_state)
        self.last_activity_time = time.time()

        # デバッグ用ウィンドウ
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.namedWindow("ApproachingPerson", cv2.WINDOW_NORMAL)

        self.get_logger().info('ApproachingPerson ノードを初期化しました')

    def camera_info_callback(self, msg: CameraInfo):
        # 一度だけ取得
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id
        self.camera_info_sub.unregister()
        self.get_logger().info(f'CameraInfo 取得: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')

    def rgb_callback(self, msg: Image):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'RGB画像変換エラー: {e}')

    def yolo_detection_callback(self, msg: YOLODetection):
        if self.current_state != self.STATE_WAITING:
            return
        if msg.object_class != "person":
            return
        if self.latest_rgb_image is None:
            return

        # 領域切り出し
        x1, y1 = int(msg.startpoint.x), int(msg.startpoint.y)
        x2, y2 = int(msg.endpoint.x), int(msg.endpoint.y)
        h, w, _ = self.latest_rgb_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        person_image = self.latest_rgb_image[y1:y2, x1:x2]

        # 手挙げ判定
        self.check_hand_raising(person_image, msg)

    def check_hand_raising(self, image, detection_msg):
        try:
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            req = CheckHand.Request()
            req.image = ros_img
            future = self.hand_detection_client.call_async(req)
            future.add_done_callback(
                lambda f: self.hand_detection_callback(f, detection_msg, image))
        except Exception as e:
            self.get_logger().error(f'手の検出リクエストエラー: {e}')

    def hand_detection_callback(self, future, detection_msg, person_image):
        try:
            resp = future.result()
            # バウンディングボックス中心
            u = (detection_msg.startpoint.x + detection_msg.endpoint.x) / 2.0
            v = (detection_msg.startpoint.y + detection_msg.endpoint.y) / 2.0
            now = time.time()

            if resp.is_hand_raised:
                # 新しい挙手機会 or 別人ならリセット
                if (self.raise_start_time is None or
                    self.last_raise_center is None or
                    math.hypot(u - self.last_raise_center[0],
                               v - self.last_raise_center[1]) > self.center_move_threshold):
                    self.raise_start_time = now
                    self.last_raise_center = (u, v)
                    return
                # 継続時間をチェック
                if now - self.raise_start_time >= self.raise_duration_threshold:
                    self.get_logger().info('🙋 挙手継続検出：移動開始')
                    self.current_state = self.STATE_MOVING
                    self.publish_marker(detection_msg)
                    self.play_music()
                    self.navigate_to_person()
                    # リセット
                    self.raise_start_time = None
                    self.last_raise_center = None
            else:
                # 挙手途切れでリセット
                self.raise_start_time = None
                self.last_raise_center = None

            # デバッグ表示
            disp = person_image.copy()
            text = "HAND RAISED" if resp.is_hand_raised else "No hand"
            color = (0,255,0) if resp.is_hand_raised else (0,0,255)
            cv2.putText(disp, text, (10,30), self.font, 0.7, color, 2)
            cv2.imshow("ApproachingPerson", disp)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'手の検出結果処理エラー: {e}')

    def publish_marker(self, detection_msg):
        if None in (self.fx, self.fy, self.cx, self.cy):
            self.get_logger().error('CameraInfo 未取得')
            return

        # ピクセル→カメラ座標系
        u = (detection_msg.startpoint.x + detection_msg.endpoint.x) / 2.0
        v = (detection_msg.startpoint.y + detection_msg.endpoint.y) / 2.0
        Z = detection_msg.depth
        x_cam = (u - self.cx) * Z / self.fx
        y_cam = (v - self.cy) * Z / self.fy
        z_cam = Z

        pt_cam = PointStamped()
        pt_cam.header.frame_id = self.camera_frame
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point = Point(x_cam, y_cam, z_cam)

        # TF2 変換
        try:
            pt_map = self.tf_buffer.transform(
                pt_cam, self.marker_frame_id, timeout= rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'TF2変換失敗: {e}')
            return

        # 地面移動なら Z=0
        goal_x, goal_y = pt_map.point.x, pt_map.point.y
        goal_z = 0.0

        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = pt_map.header.stamp
        marker.ns = "approaching_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = goal_z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = self.get_parameter("color_r").value
        marker.color.g = self.get_parameter("color_g").value
        marker.color.b = self.get_parameter("color_b").value
        marker.color.a = self.get_parameter("color_a").value
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        self.marker_pub.publish(marker)

        goal = PoseStamped()
        goal.header = marker.header
        goal.pose = marker.pose
        self.latest_goal_pose = goal
        self.get_logger().info(f'ゴール設定: x={goal_x:.2f}, y={goal_y:.2f}')

    def navigate_to_person(self):
        if not self.latest_goal_pose:
            self.get_logger().error('目標Pose未設定')
            return
        # 向き固定
        self.latest_goal_pose.pose.orientation.w = 1.0
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.latest_goal_pose
        fut = self._nav_action_client.send_goal_async(goal_msg)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('ゴール拒否')
            self.current_state = self.STATE_WAITING
            return
        self.get_logger().info('ゴール受理, 結果待ち...')
        gh.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        res = future.result().result
        if res.success:
            self.get_logger().info('ナビ完了')
            self.stop_music()
            self.speak("移動完了しました！")
        else:
            self.get_logger().error(f'ナビ失敗: {res}')
            self.stop_music()
        self.current_state = self.STATE_WAITING
        self.last_activity_time = time.time()

    def play_music(self):
        try:
            pygame.mixer.music.load(self.music_file)
            pygame.mixer.music.play(-1)
            self.get_logger().info('音楽再生')
        except Exception as e:
            self.get_logger().error(f'音楽再生エラー: {e}')

    def stop_music(self):
        try:
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
                self.get_logger().info('音楽停止')
        except Exception as e:
            self.get_logger().error(f'音楽停止エラー: {e}')

    def check_idle_state(self):
        if self.current_state == self.STATE_WAITING:
            if time.time() - self.last_activity_time > self.idle_timeout:
                self.idle_speech()
                self.last_activity_time = time.time()

    def idle_speech(self):
        msgs = [
            "手を挙げていただくと移動します。",
            "お手伝いできることはありませんか？",
            "何かあれば手を挙げてください。"
        ]
        self.speak(np.random.choice(msgs))

    def speak(self, text: str):
        """ TTSサービスを呼び出して完了まで待機 """
        req = TTS.Request()
        req.text = text
        self.get_logger().info(f'発話: {text}')
        future = self.tts_client.call_async(req)
        # 完了までブロック
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'TTS応答: {future.result().message}')
        else:
            self.get_logger().error('TTSサービス呼び出し失敗')

def main(args=None):
    rclpy.init(args=args)
    node = ApproachingPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ノード終了')
        if pygame.mixer.get_init() and pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
