#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from oc_approaching_interfaces.srv import CheckHand


class HandDetectionService(Node):
    def __init__(self):
        super().__init__('hand_detection_service')

        # サービスサーバーの作成
        self.srv = self.create_service(
            CheckHand,
            '/hand_detection/check_hand',
            self.handle_check_hand
        )

        # CvBridgeの初期化
        self.bridge = CvBridge()

        # MediaPipe Pose の初期化
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("Hand detection service is ready.")

    def handle_check_hand(self, request, response):
        """
        Poseランドマークから、直立して手を挙げているかを精度高く判定
        - 肩より手首が十分上にある
        - 肘も肩より上にある
        - 手首が頭（鼻）よりも上にある
        """
        # 画像変換
        try:
            image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"画像変換エラー: {e}")
            response.is_hand_raised = False
            return response

        # MediaPipe は RGB 前提
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.pose.process(image_rgb)

        if not results.pose_landmarks:
            self.get_logger().warn("Pose検出失敗：ランドマークが検出されませんでした")
            response.is_hand_raised = False
            return response

        # 各ランドマークを取得
        lm = results.pose_landmarks.landmark
        # インデックス参照（MediaPipe Pose）
        nose         = lm[0]
        left_sh      = lm[11]
        right_sh     = lm[12]
        left_elb     = lm[13]
        right_elb    = lm[14]
        left_wrist   = lm[15]
        right_wrist  = lm[16]

        # 閾値（画像高さの割合）
        margin_shoulder = 0.10  # 肩より10%上
        margin_head     = 0.02  # 鼻より2%上

        # 左手判定
        left_wrist_above_sh = left_wrist.y < (left_sh.y - margin_shoulder)
        left_elbow_above_sh = left_elb.y  < (left_sh.y - margin_shoulder / 2)
        left_wrist_above_head = left_wrist.y < (nose.y - margin_head)
        left_hand_raised = left_wrist_above_sh and left_elbow_above_sh and left_wrist_above_head

        # 右手判定
        right_wrist_above_sh = right_wrist.y < (right_sh.y - margin_shoulder)
        right_elbow_above_sh = right_elb.y  < (right_sh.y - margin_shoulder / 2)
        right_wrist_above_head = right_wrist.y < (nose.y - margin_head)
        right_hand_raised = right_wrist_above_sh and right_elbow_above_sh and right_wrist_above_head

        if left_hand_raised or right_hand_raised:
            self.get_logger().info("🙆‍♂️ 挙手が検出されました")
            response.is_hand_raised = True
        else:
            self.get_logger().info("❌ 挙手は検出されませんでした")
            response.is_hand_raised = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down hand detection service...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
