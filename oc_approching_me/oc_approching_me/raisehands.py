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
        self.srv = self.create_service(CheckHand, '/hand_detection/check_hand', self.handle_check_hand)
        
        # CvBridgeの初期化
        self.bridge = CvBridge()
        
        # MediaPipe Pose の初期化
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5,
                                       min_tracking_confidence=0.5)
        
        self.get_logger().info("Hand detection service is ready.")

    def handle_check_hand(self, request, response):
        """
        サービスリクエストで渡された画像からPose検出を行い、
        左右の手首がそれぞれ対応する肩より上にあるか判定して、手挙げをチェックする。
        ※ 画像中ではy座標が小さいほど上部となるため、例えば
          left_wrist.y < left_shoulder.y なら左手が上がっていると判断する。
        """
        try:
            # ROS ImageメッセージからOpenCV画像（BGR）へ変換
            image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"画像変換エラー: {e}")
            response.is_hand_raised = False
            return response

        # 画像処理の前に書き込みフラグをFalseに設定（効率化のため）
        image.flags.writeable = False
        # BGR→RGBに変換（MediaPipeはRGB画像を前提）
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # MediaPipe Pose による検出
        results = self.pose.process(image_rgb)
        
        # 検出結果がなければFalseとする
        if not results.pose_landmarks:
            self.get_logger().warn("Pose検出失敗：ランドマークが検出されませんでした")
            response.is_hand_raised = False
            return response
        
        # ランドマークの抽出（normalized座標: 0〜1, x,y）
        landmarks = results.pose_landmarks.landmark
        
        # 左肩: 11, 右肩: 12, 左手首: 15, 右手首: 16  (MediaPipe Poseの仕様)
        left_shoulder = landmarks[11]
        right_shoulder = landmarks[12]
        left_wrist = landmarks[15]
        right_wrist = landmarks[16]
        
        # 左右それぞれにおいて、手首のy座標が肩のy座標より小さい（＝上にある）かを判定
        left_hand_raised = left_wrist.y < left_shoulder.y
        right_hand_raised = right_wrist.y < right_shoulder.y
        
        if left_hand_raised or right_hand_raised:
            self.get_logger().info("🙆挙手が検出されました")
            response.is_hand_raised = True
        else:
            self.get_logger().info("❌挙手は検出されませんでした")
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
