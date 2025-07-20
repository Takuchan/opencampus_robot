#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

# カスタムメッセージのインポート（パッケージ名は適宜置換）
from oc_recognition_yolo_interfaces.msg import YOLODetection
from geometry_msgs.msg import Point

class YoloDepthDetector(Node):
    def __init__(self):
        super().__init__('yolo_depth_detector')
        # パラメータ宣言（launchファイルからの上書き可能）
        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("target_class", "person")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("detection_topic", "/yolo_detection")
        self.declare_parameter("frame_id", "realsense")
        
        # パラメータ取得
        model_path = self.get_parameter("model_path").value
        self.conf_thres = self.get_parameter("confidence_threshold").value
        self.target_class = self.get_parameter("target_class").value
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.detection_topic = self.get_parameter("detection_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        # YOLOモデルのロード
        self.get_logger().info("Loading YOLO model from: {}".format(model_path))
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error("Error loading YOLO model: {}".format(str(e)))
            rclpy.shutdown()

        self.bridge = CvBridge()

        # 最新の深度画像用の変数を初期化
        self.latest_depth_image = None

        # 購読：RGB画像と深度画像
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10)
        
        # 検出結果パブリッシャー（カスタムメッセージをパブリッシュ）
        self.detection_pub = self.create_publisher(YOLODetection, self.detection_topic, 10)
        
    def depth_callback(self, msg: Image):
        try:
            # 深度画像は16UC1（例：mm単位またはメートル単位に合わせ変換）
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error("Failed to convert depth image: {}".format(str(e)))
        
    def rgb_callback(self, msg: Image):
        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image received yet")
            return

        try:
            # RGB画像をOpenCV画像に変換
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Failed to convert RGB image: {}".format(str(e)))
            return

        height, width = rgb_image.shape[:2]

        # YOLOによる推論
        results = self.model(rgb_image)
        # 検出結果を走査
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                # ボックス情報：xyxyの座標、信頼度、クラスID
                coords = box.xyxy[0].cpu().numpy().astype(int)  # [xmin, ymin, xmax, ymax]
                conf = float(box.conf.cpu().numpy())
                cls_id = int(box.cls.cpu().numpy())

                # 信頼度フィルタ
                if conf < self.conf_thres:
                    continue

                # クラス名の取得（通常、COCO定義の場合 person は 0 番）
                class_name = result.names.get(cls_id, "unknown")
                if class_name != self.target_class:
                    continue

                # 検出領域の中心座標を計算
                cx = int((coords[0] + coords[2]) / 2)
                cy = int((coords[1] + coords[3]) / 2)
                
                # 深度画像から中心の深度値を抽出（※深度画像とRGB画像がアライメント済みの場合）
                depth_val = self.get_depth_at(cx, cy)
                
                # カスタムメッセージに各情報をセット
                detection_msg = YOLODetection()
                # ヘッダー情報
                detection_msg.header = Header()
                detection_msg.header.stamp = self.get_clock().now().to_msg()
                detection_msg.header.frame_id = self.frame_id
                # 画像サイズ
                detection_msg.width = width
                detection_msg.height = height
                # 検出パラメータ
                detection_msg.threshold = self.conf_thres
                sp = Point()
                sp.x = float(coords[0])
                sp.y = float(coords[1])
                sp.z = 0.0
                detection_msg.startpoint = sp
                ep = Point()
                ep.x = float(coords[2])
                ep.y = float(coords[3])
                ep.z = 0.0
                detection_msg.endpoint = ep
                # クラス名
                detection_msg.object_class = class_name
                # 深度（m単位）
                detection_msg.depth = depth_val

                # パブリッシュ
                self.detection_pub.publish(detection_msg)

                # 結果画像上に bounding box とテキスト表示（クラス名、信頼度、深度情報）
                cv2.rectangle(rgb_image, (coords[0], coords[1]), (coords[2], coords[3]), (0, 255, 0), 2)
                annotation = f"{class_name}: {conf:.2f}, depth: {depth_val:.2f}m"
                cv2.putText(rgb_image, annotation, (coords[0], coords[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # デバッグ用：検出結果画像を表示
        cv2.imshow("YOLO Detection", rgb_image)
        cv2.waitKey(1)
    
    def get_depth_at(self, x: int, y: int) -> float:
        """
        指定された(x,y)座標における深度値を抽出します。
        深度画像のエンコーディングに合わせて、必要なスケーリングを行ってください。
        ここでは16ビットの深度画像（mm値）をメートルに変換する例です。
        """
        try:
            # 配列境界のチェック
            if y < 0 or y >= self.latest_depth_image.shape[0] or x < 0 or x >= self.latest_depth_image.shape[1]:
                return 0.0
            depth_pixel = self.latest_depth_image[y, x]
            # 例：16ビット深度画像（mm単位）→ メートルに変換
            depth_in_m = float(depth_pixel) / 1000.0
            return depth_in_m
        except Exception as e:
            self.get_logger().error("Failed to get depth at point ({}, {}): {}".format(x, y, str(e)))
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
