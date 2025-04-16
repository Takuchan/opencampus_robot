#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from oc_recognition_yolo_interfaces.msg import YOLODetection
import math

class YOLOMarkerPublisher(Node):
    def __init__(self):
        super().__init__('yolo_marker_publisher')
        
        # Realsense D455 の一般的な画角をパラメータとして指定（必要に応じて調整してください）
        self.declare_parameter("horizontal_fov_deg", 85.0)  # 水平方向の画角（度）
        self.declare_parameter("vertical_fov_deg", 58.0)      # 垂直方向の画角（度）
        # マーカーの大きさ（球の直径）と表示時間（marker.lifetime）
        self.declare_parameter("marker_scale", 0.3)
        self.declare_parameter("marker_lifetime_sec", 1.0)    # RViz上に残る時間（秒）
        # マーカーの色（RGBA）
        self.declare_parameter("color_r", 1.0)
        self.declare_parameter("color_g", 0.0)
        self.declare_parameter("color_b", 0.0)
        self.declare_parameter("color_a", 0.8)
        
        # ラジアンに変換
        self.h_fov = math.radians(self.get_parameter("horizontal_fov_deg").value)
        self.v_fov = math.radians(self.get_parameter("vertical_fov_deg").value)
        self.marker_scale = self.get_parameter("marker_scale").value
        self.marker_lifetime = self.get_parameter("marker_lifetime_sec").value
        
        # YOLODetectionのメッセージを購読
        self.sub = self.create_subscription(
            YOLODetection,
            '/yolo_detection',
            self.detection_callback,
            10)
        
        # RViz上でマーカーを表示するためのパブリッシャー
        self.marker_pub = self.create_publisher(
            Marker,
            'visualization_marker',
            10
        )
        self.marker_id = 0  # 検出ごとに異なるIDを設定
        
    def detection_callback(self, msg: YOLODetection):
        # バウンディングボックスの中心座標（ピクセル）を計算
        center_u = (msg.startpoint.x + msg.endpoint.x) / 2.0
        center_v = (msg.startpoint.y + msg.endpoint.y) / 2.0
        image_width = msg.width
        image_height = msg.height
        
        # 焦点距離fx, fyの計算（ピクセル単位）
        fx = (image_width / 2.0) / math.tan(self.h_fov / 2.0)
        fy = (image_height / 2.0) / math.tan(self.v_fov / 2.0)
        
        # カメラ光学フレームでの座標計算（通常のpin-holeモデル）
        # optical frame (x:右, y:下, z:前) における値
        x_opt = (center_u - image_width / 2.0) * msg.depth / fx
        y_opt = (center_v - image_height / 2.0) * msg.depth / fy
        z_opt = msg.depth
        
        # カメラのTFがロボットの前進方向（X軸）に合わせているため、以下の変換を適用
        # robot base frame (x:前, y:左, z:上)
        x_robot = z_opt            # 前進方向
        y_robot = - x_opt          # 右側が左方向に反転
        z_robot = - y_opt          # 下が上に反転
        
        # マーカー作成
        marker = Marker()
        marker.header = msg.header   # YOLODetection に含まれるヘッダー（TF情報）
        marker.ns = "yolo_detections"
        marker.id = self.marker_id   # 検出ごとに一意のIDを設定
        self.marker_id += 1
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # マーカーの位置と姿勢
        marker.pose.position.x = x_robot
        marker.pose.position.y = y_robot
        marker.pose.position.z = z_robot
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # マーカーのスケール
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        
        # マーカーの色（RGBA）
        marker.color.r = self.get_parameter("color_r").value
        marker.color.g = self.get_parameter("color_g").value
        marker.color.b = self.get_parameter("color_b").value
        marker.color.a = self.get_parameter("color_a").value
        
        # マーカーの寿命：一定時間後に自動削除
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        self.marker_pub.publish(marker)
        self.get_logger().info("Published marker id {}: Robot frame coords [X: {:.2f}, Y: {:.2f}, Z: {:.2f}]"
                               .format(marker.id, x_robot, y_robot, z_robot))
        
def main(args=None):
    rclpy.init(args=args)
    node = YOLOMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
