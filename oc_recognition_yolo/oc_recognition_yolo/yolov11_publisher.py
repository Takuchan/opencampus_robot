#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import numpy as np

# カスタムメッセージとサービスのインポート
from oc_recognition_yolo_interfaces.msg import DetectedObjects,Vector2DYOLO
from oc_recognition_yolo_interfaces.srv import CheckObjects, CheckPosition, SelectPerson

# vision_msgs の検出用メッセージ
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Vector3
from vision_msgs.msg import Pose2D  # 名前が被らないように別名で



class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.bridge = CvBridge()
        self.model = YOLO("yolo11n.pt")
        self.depth_image = None
        self.color_image = None
        self.latest_header = None
        
        # クラスごとの色を定義
        self.class_colors = {
            'person': (0, 0, 255),    # 赤
            'chair': (0, 255, 0),     # 緑
            'cup': (255, 0, 0),       # 青
            'bottle': (255, 165, 0)   # オレンジ
        }
        
        # クラスIDとクラス名の対応
        self.class_names = {
            0: 'person',
            56: 'chair',
            41: 'cup',
            39: 'bottle'
        }
        
        # 検出されたオブジェクトを保存するリスト
        self.current_detections = []
        # 人物検出結果を保存するリスト
        self.detected_persons = []
        
        # パブリッシャーの設定
        self.image_pub = self.create_publisher(
            Image, 
            '/yolo_detected_with_depth', 
            10)
            
        self.objects_pub = self.create_publisher(
            DetectedObjects,
            '/detected/detected_objects',
            10)
        
        # 新たに /yolo_detections トピックへのパブリッシャーを追加
        self.yolo_detections_pub = self.create_publisher(
            Detection2DArray,
            '/detected/yolo_detections',
            10)
            
        # サービスの設定
        self.check_objects_srv = self.create_service(
            CheckObjects,
            '/detected/check_objects',
            self.check_objects_callback)
            
        self.check_position_srv = self.create_service(
            CheckPosition,
            '/detected/check_position',
            self.check_position_callback)
            
        self.select_person_srv = self.create_service(
            SelectPerson,
            '/detected/select_person',
            self.select_person_callback)

        # サブスクライバーの設定
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)
            
        self.depth_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10
        )

        self.get_logger().info('YOLO detector node has been started')

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
            self.process_images()
        except CvBridgeError as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def color_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.latest_header = data.header
            self.process_images()
        except CvBridgeError as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')

    def get_depth_from_bbox(self, bbox, depth_image, height, width):
        """
        バウンディングボックス内の深度値を解析する補助関数
        """
        x1, y1, x2, y2 = bbox
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(width - 1, int(x2)), min(height - 1, int(y2))
        
        box_width = x2 - x1
        box_height = y2 - y1
        
        # ボックスサイズに応じてサンプリング数を調整
        if box_width < 100 or box_height < 100:
            divisions = 3
        else:
            divisions = 5

        depth_values = []
        
        # グリッドサンプリング
        for y in np.linspace(y1, y2, divisions):
            for x in np.linspace(x1, x2, divisions):
                y_idx = int(y)
                x_idx = int(x)
                if 0 <= y_idx < height and 0 <= x_idx < width:
                    depth_value = depth_image[y_idx, x_idx]
                    if depth_value > 0:  # 有効な深度値のみを追加
                        depth_values.append(depth_value)
        
        if not depth_values:
            return None
            
        # ヒストグラムによる深度値の解析
        hist, bins = np.histogram(depth_values, bins=20)
        most_common_depth = bins[np.argmax(hist)]
        
        # 信頼度スコアの計算
        confidence_score = len(depth_values) / (divisions * divisions)
        
        return most_common_depth / 1000.0, confidence_score  # メートルに変換

    def check_objects_callback(self, request, response):
        response.exists = any(obj[0] == request.object_name for obj in self.current_detections)
        return response

    def check_position_callback(self, request, response):
        obj1_depth = None
        obj2_depth = None
        
        # 各オブジェクトの深度を取得
        for obj in self.current_detections:
            if obj[0] == request.object1:
                obj1_depth = obj[1]
            if obj[0] == request.object2:
                obj2_depth = obj[1]
        
        # 両方のオブジェクトが検出されていない場合
        if obj1_depth is None or obj2_depth is None:
            response.relative_position = "NO_OBJ"
            return response
        
        # 深度差に基づいて位置関係を判定
        depth_diff = obj1_depth - obj2_depth
        if abs(depth_diff) < 0.3:  # 30cm以内なら同じ位置とみなす
            response.relative_position = f"{request.object1} is at the same distance as {request.object2}"
        elif depth_diff > 0:
            response.relative_position = f"{request.object1} is behind {request.object2}"
        else:
            response.relative_position = f"{request.object1} is in front of {request.object2}"
        
        return response

    def select_person_callback(self, request, response):
        # 条件に合う人物のみをフィルタリング
        filtered_persons = [
            person for person in self.detected_persons
            if person[2] <= request.max_distance  # 指定距離以内
        ]
        
        # 右から左へソート（x座標の降順）
        filtered_persons.sort(key=lambda x: x[0], reverse=True)
        
        # 応答の初期化
        response.success = False
        response.selected_person_depth = 0.0
        response.x_center = 0
        response.y_center = 0
        
        # 指定されたインデックスが有効かチェック
        if 0 < request.person_index_from_right <= len(filtered_persons):
            # インデックスは1始まりなので-1する
            selected_person = filtered_persons[request.person_index_from_right - 1]
            response.success = True
            response.selected_person_depth = selected_person[2]  # depth
            response.x_center = selected_person[0]  # x_center
            response.y_center = selected_person[1]  # y_center
            
            # ログ出力
            self.get_logger().info(
                f"Selected person: right_index={request.person_index_from_right}, "
                f"depth={response.selected_person_depth:.2f}m, "
                f"position=({response.x_center}, {response.y_center})"
            )
        else:
            self.get_logger().warn(
                f"No person found matching criteria: "
                f"index={request.person_index_from_right}, "
                f"max_distance={request.max_distance}m"
            )
        
        return response

    def publish_detected_objects(self):
        msg = DetectedObjects()
        msg.object_names = [obj[0] for obj in self.current_detections]
        msg.depths = [obj[1] for obj in self.current_detections]
        msg.confidences = [obj[2] for obj in self.current_detections]
        self.objects_pub.publish(msg)

    def publish_vision_detections(self, detection_msgs):
        """
        vision_msgs を用いて Detection2DArray を生成し、/yolo_detections トピックに発行
        """
        detection_array = Detection2DArray()
        # ヘッダは受信した画像のものを流用（なければ現在時刻を設定）
        if self.latest_header is not None:
            detection_array.header = self.latest_header
        else:
            from rclpy.clock import Clock
            detection_array.header.stamp = Clock().now().to_msg()
        detection_array.detections = detection_msgs
        self.yolo_detections_pub.publish(detection_array)
        self.get_logger().debug("Published vision_msgs detections on /yolo_detections")

    def process_images(self):
        if self.depth_image is None or self.color_image is None:
            return

        try:
            output_image = self.color_image.copy()
            height, width = self.depth_image.shape[:2]
            
            # 検出結果を格納するリストをクリア
            self.current_detections = []
            self.detected_persons = []
            
            # vision_msgs 用の検出結果リスト
            vision_detections = []

            results = self.model(output_image)
            
            for result in results[0].boxes:
                box = result.xyxy[0].cpu().numpy()
                conf = result.conf[0].cpu().numpy()
                cls = int(result.cls[0].cpu().numpy())
                
                if cls not in self.class_names:
                    continue
                
                # 深度値と信頼度スコアの取得
                depth_result = self.get_depth_from_bbox(box, self.depth_image, height, width)
                if depth_result is None:
                    continue
                    
                depth_value, confidence_score = depth_result
                
                # 信頼度スコアが低い場合はスキップ
                if confidence_score < 0.5:
                    continue

                x1, y1, x2, y2 = map(int, box[:4])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                class_name = self.class_names[cls]
                color = self.class_colors[class_name]

                # 人物の場合、専用リストにも追加
                if class_name == 'person':
                    self.detected_persons.append(
                        (center_x, center_y, depth_value, (x1, y1, x2, y2))
                    )

                # 描画処理
                cv2.rectangle(output_image, 
                              (x1, y1), 
                              (x2, y2), 
                              color, 2)
                cv2.circle(output_image, (center_x, center_y), 5, color, -1)
                label = f"{class_name} Conf:{conf:.2f} Depth:{depth_value:.2f}m"
                cv2.putText(output_image, label,
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_PLAIN, 1, color, 1)
                
                # 検出結果をリストに追加（カスタムメッセージ用）
                self.current_detections.append([class_name, depth_value, confidence_score])
                
                # vision_msgs 用の Detection2D を生成
                detection = Detection2D()
                # ヘッダは各検出に付与（ここでは最新の画像ヘッダを流用）
                if self.latest_header is not None:
                    detection.header = self.latest_header
                else:
                    from rclpy.clock import Clock
                    detection.header.stamp = Clock().now().to_msg()
                # バウンディングボックス情報の設定
                bbox = BoundingBox2D()
                center = Pose2D()
                center.position.x = float(center_x)
                center.position.y = float(center_y)
                center.theta = 0.0
                bbox.center = center
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)

                detection.bbox = bbox
                # 検出結果（ObjectHypothesisWithPose）の設定
                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = class_name
                hypo.hypothesis.score = float(conf)
                # ※ pose は必須の場合に設定。ここでは不要であればデフォルト（0値）のままとする。
                detection.results.append(hypo)
                # vision_msgs の検出結果リストに追加
                vision_detections.append(detection)

            # 検出結果の公開（カスタムメッセージ）
            self.publish_detected_objects()
            # vision_msgs 型の検出結果のパブリッシュ
            self.publish_vision_detections(vision_detections)

            # 人物検出結果のデバッグ出力
            if self.detected_persons:
                self.get_logger().info(f"\nDetected {len(self.detected_persons)} persons:")
                for i, person in enumerate(sorted(self.detected_persons, key=lambda x: x[0], reverse=True)):
                    self.get_logger().info(
                        f"Person {i+1} from right: "
                        f"depth={person[2]:.2f}m, "
                        f"position=({person[0]}, {person[1]})"
                    )

            # 画像の公開（検出結果描画済み）
            ros_image = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
            ros_image.header = self.latest_header
            self.image_pub.publish(ros_image)
            
            # 画像バッファのクリア
            self.depth_image = None
            self.color_image = None
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = YoloDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
