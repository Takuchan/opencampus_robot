#!/usr/bin/env python3
# 新しいアプローチングパーソンのプログラム

import os
import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped,PointStamped
from visualization_msgs.msg import Marker

from oc_recognition_yolo_interfaces.msg import YOLODetection
from oc_approaching_interfaces.srv import CheckHand
from oc_tts_interfaces.srv import TTS
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
import tf2_geometry_msgs  # 必要: PointStamped 変換ヘルパ

from rclpy.time import Duration, Time

import tf2_ros
from geometry_msgs.msg import TransformStamped

class ApproachingPerson(Node):
    def __init__(self):
        super().__init__('approaching_person_node')

        # サウンドファイル
        sound_path = os.path.join(
            get_package_share_directory('oc_approching_me'),
            'sounds', 'downloaded_audio.wav'
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ─── パラメータ宣言 ─────────────────────────
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("yolo_topic", "/yolo_detection")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("camera_frame_id", "realsense")     # ★ 追加

        self.declare_parameter("idle_timeout", 30.0)
        self.declare_parameter("music_file", sound_path)
        self.declare_parameter("horizontal_fov_deg", 85.0)
        self.declare_parameter("vertical_fov_deg", 58.0)
        self.declare_parameter("marker_scale", 0.3)
        self.declare_parameter("marker_lifetime_sec", 3.0)
        self.declare_parameter("color_r", 1.0)
        self.declare_parameter("color_g", 0.0)
        self.declare_parameter("color_b", 0.0)
        self.declare_parameter("color_a", 0.8)
        self.declare_parameter("frame_collect_count", 30)
        self.declare_parameter("max_position_deviation", 0.5)
        # ────────────────────────────────────────

        # ─── パラメータ取得 ───────────────────────
        self.rgb_topic            = self.get_parameter("rgb_topic").value
        self.yolo_topic           = self.get_parameter("yolo_topic").value
        self.marker_frame_id      = self.get_parameter("marker_frame_id").value
        self.idle_timeout         = self.get_parameter("idle_timeout").value
        self.music_file           = self.get_parameter("music_file").value
        self.h_fov                = math.radians(self.get_parameter("horizontal_fov_deg").value)
        self.v_fov                = math.radians(self.get_parameter("vertical_fov_deg").value)
        self.marker_scale         = self.get_parameter("marker_scale").value
        self.marker_lifetime      = self.get_parameter("marker_lifetime_sec").value
        self.frame_collect_count  = self.get_parameter("frame_collect_count").value
        self.max_position_deviation = self.get_parameter("max_position_deviation").value
        self.camera_frame_id      = self.get_parameter("camera_frame_id").value  # ★ 追加

        # ────────────────────────────────────────

        # OpenCV ブリッジ
        self.bridge = CvBridge()
        self.latest_rgb_image = None

        # ── 状態管理 ───────────────────────────
        self.STATE_WAITING    = 0  # 手検出待ち
        self.STATE_COLLECTING = 1  # フレーム収集中
        self.STATE_MOVING     = 2  # ナビ中
        self.current_state    = self.STATE_WAITING
        # ────────────────────────────────────────

        # バッファ
        self.collected_positions = []  # list of (x,y,z)
        self.collect_count = 0

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10)
        self.yolo_sub = self.create_subscription(
            YOLODetection, self.yolo_topic, self.yolo_detection_callback, 10)

        # Nav2 ActionClient
        self._nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 アクションサーバを待機中...')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # HandDetection ServiceClient
        self.hand_client = self.create_client(
            CheckHand, '/hand_detection/check_hand')
        while not self.hand_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('HandDetectionサービスを待機中...')

        # TTS ServiceClient
        self.tts_client = self.create_client(TTS, 'tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTS サービスを待機中...')

        # タイマー：アイドル発話
        self.idle_timer = self.create_timer(1.0, self.check_idle_state)
        self.last_activity_time = time.time()

        # 表示用
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.namedWindow("ApproachingPerson", cv2.WINDOW_NORMAL)

        self.get_logger().info('ApproachingPerson ノード 初期化完了')

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'RGB画像変換エラー: {e}')

    def yolo_detection_callback(self, msg):
        # 移動中は完全スキップ
        if self.current_state == self.STATE_MOVING:
            return
        if msg.object_class != "person" or self.latest_rgb_image is None:
            return

        x1, y1 = int(msg.startpoint.x), int(msg.startpoint.y)
        x2, y2 = int(msg.endpoint.x),   int(msg.endpoint.y)
        h, w, _ = self.latest_rgb_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        person_img = self.latest_rgb_image[y1:y2, x1:x2]

        self.check_hand_raising(person_img, msg)

    def check_hand_raising(self, image, detection_msg):
        # WAITING と COLLECTING 時は検出継続
        if self.current_state == self.STATE_MOVING:
            return

        try:
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            req = CheckHand.Request(image=ros_img)
            fut = self.hand_client.call_async(req)
            fut.add_done_callback(
                lambda f: self.hand_detection_callback(f, detection_msg, image))
        except Exception as e:
            self.get_logger().error(f'手検出リクエストエラー: {e}')

    def hand_detection_callback(self, future, detection_msg, image):
        # WAITING/COLLECTING 時のみ処理
        if self.current_state not in (self.STATE_WAITING, self.STATE_COLLECTING):
            return

        try:
            res = future.result()
            if res.is_hand_raised:
                if self.current_state == self.STATE_WAITING:
                    # 収集中に切替
                    self.current_state = self.STATE_COLLECTING
                    self.collect_count = 0
                    self.collected_positions.clear()
                    self.speak("ちょっと待ってね")

                # 収集中のフレームを蓄積
                if self.current_state == self.STATE_COLLECTING:
                    u = (detection_msg.startpoint.x + detection_msg.endpoint.x) * 0.5
                    v = (detection_msg.startpoint.y + detection_msg.endpoint.y) * 0.5

                    # 画像全体の幅高さを使って fx, fy を計算
                    img_w = detection_msg.width
                    img_h = detection_msg.height
                    fx = (img_w * 0.5) / math.tan(self.h_fov * 0.5)
                    fy = (img_h * 0.5) / math.tan(self.v_fov * 0.5)
                    depth = detection_msg.depth  # ← Realsense の Z（前方向）[m]

                    # ------ ROS2 カメラ座標系 (X:前, Y:左, Z:上) への逆投影 ------
                    x_cam = depth
                    y_cam = -(u - img_w * 0.5) * depth / fx   # 左が +Y
                    z_cam = -(v - img_h * 0.5) * depth / fy    # 上が +Z
                    # ----------------------------------------------------------------

                    self.collected_positions.append((x_cam, y_cam, z_cam))
                    self.collect_count += 1

                    if self.collect_count >= self.frame_collect_count:
                        self.finish_collecting()
            else:
                # 挙手なし表示
                disp = image.copy()
                cv2.putText(disp, "No hand", (10,30), self.font, 0.7, (0,0,255), 2)
                cv2.imshow("ApproachingPerson", disp)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'手検出コールバックエラー: {e}')

    def finish_collecting(self):
        if not self.collected_positions:
            self.current_state = self.STATE_WAITING
            return

        xs, ys, zs = zip(*self.collected_positions)
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_z = sum(zs) / len(zs)

        # ばらつきチェック
        deviations = [math.hypot(x - avg_x, y - avg_y) for x, y, _ in self.collected_positions]
        if max(deviations) > self.max_position_deviation or avg_x <= 0:
            self.speak("検出が安定しなかったので中止します")
            self.current_state = self.STATE_WAITING
            return



        # ゴール設定
        ps_cam = PointStamped()
        ps_cam.header.frame_id = self.camera_frame_id
        ps_cam.header.stamp = Time(seconds=0).to_msg()   # ← 常に最新 TF を使わせる

        ps_cam.point.x = avg_x
        ps_cam.point.y = avg_y
        ps_cam.point.z = avg_z

        try:
            ps_map = self.tf_buffer.transform(
                ps_cam,
                self.marker_frame_id,
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().error(f'TF 変換失敗: {e}')
            self.current_state = self.STATE_WAITING
            return
        # 移動前発話＋待機
        self.speak("移動します")
        time.sleep(3.0)

         # ---- Nav2 ゴール PoseStamped（map 座標系） ----
        goal = PoseStamped()
        goal.header.frame_id = self.marker_frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = ps_map.point.x
        goal.pose.position.y = ps_map.point.y
        goal.pose.position.z = 0.0              # Nav2 は 2D
        goal.pose.orientation.w = 1.0
        self.latest_goal_pose = goal

        # 目印用 TF と Marker
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.marker_frame_id
        t.child_frame_id  = "goal_frame"
        t.transform.translation.x = ps_map.point.x
        t.transform.translation.y = ps_map.point.y
        t.transform.translation.z = ps_map.point.z
        t.transform.rotation.w    = 1.0
        self.tf_broadcaster.sendTransform(t)
        # 可視化マーカー
        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = t.header.stamp
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = ps_map.point.x
        marker.pose.position.y = ps_map.point.y
        marker.pose.position.z = ps_map.point.z
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = self.get_parameter("color_r").value
        marker.color.g = self.get_parameter("color_g").value
        marker.color.b = self.get_parameter("color_b").value
        marker.color.a = self.get_parameter("color_a").value
        marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
        self.marker_pub.publish(marker)

        self.play_music()
        self.navigate_to_person()
        self.current_state = self.STATE_MOVING

    def navigate_to_person(self):
        goal_msg = NavigateToPose.Goal(pose=self.latest_goal_pose)
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
        self.stop_music()
        if status == 4:
            self.speak("移動完了しました！")
        self.current_state = self.STATE_WAITING
        self.last_activity_time = time.time()

    def play_music(self):
        try:
            import pygame
            pygame.mixer.init()
            pygame.mixer.music.load(self.music_file)
            pygame.mixer.music.play(-1)
        except Exception:
            pass

    def stop_music(self):
        try:
            import pygame
            if pygame.mixer.get_init() and pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
        except Exception:
            pass

    def check_idle_state(self):
        if self.current_state == self.STATE_WAITING and \
           time.time() - self.last_activity_time > self.idle_timeout:
            self.speak("手を挙げていただくと、そちらに向かいます。")
            self.last_activity_time = time.time()

    def speak(self, text):
        req = TTS.Request(text=text)
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
