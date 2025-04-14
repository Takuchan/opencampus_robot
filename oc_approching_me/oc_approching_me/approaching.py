#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# YOLO検知結果として、vision_msgs.msg.Detection2DArrayを使用する例です。
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from oc_approaching_interfaces.srv import StartDetection 
from oc_approaching_interfaces.srv import CheckHand

class ApproachingPerson(Node):
    def __init__(self):
        super().__init__('approaching_person_node')

        # TF変換用オブジェクト（buffer と listener）の初期化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # YOLO検知結果のサブスクライバー：ここでは /detected/yolo_detections トピックから検知結果を受信
        self.create_subscription(Detection2DArray, "/detected/yolo_detections", self.detection_callback, 10)
        
        # カメラ画像（color）のサブスクライバーを追加して、最新画像を保持
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_image_callback, 10)
        self.latest_color_image = None
        self.latest_color_header = None

        self.start_detection_srv = self.create_service(
            StartDetection,
            '/start_approaching_person',
            self.handle_start_detection
        )

        # ナビゲーション用目標位置として /goal_pose に PoseStamped をパブリッシュ
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # 検知結果のウィンドウ（5秒間）を蓄積して平均検出数を評価
        self.detection_window = []  # (timestamp, [person_detection, ...]) のタプルを格納
        self.window_duration = 5.0   # 検知ウィンドウの長さ（秒）

        # 複数人検知時の挙手判定サービス通信のための設定（トライアル数など）
        self.max_hand_detection_trials = 3
        
        self.bridge = CvBridge()
        self.get_logger().info("ApproachingPerson node started.")

    def handle_start_detection(self, request, response):
        self.get_logger().info("Received service call: start_detection")
        self.evaluate_detections()
        response.success = True
        return response

    def color_image_callback(self, msg: Image):
        """カメラの色画像を受信し、最新画像として保持する"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_color_image = cv_img
            self.latest_color_header = msg.header
        except CvBridgeError as e:
            self.get_logger().error(f"カラ―画像変換エラー: {str(e)}")

    def detection_callback(self, msg: Detection2DArray):
        """
        YOLO検知結果のコールバック  
        各検出メッセージの結果から、ラベルが 'person' であるものだけ抽出してウィンドウに追加する
        """
        persons = []
        for detection in msg.detections:
            print("今の出てクションは",detection)
            # 仮に detection.results[0].id に物体ラベル（例："person"）が入っている前提
            # if detection.results and detection.results[0].id == "person":
            #     persons.append(detection)
            
        # 現在時刻と person 検出リストをウィンドウに蓄積
        self.detection_window.append((self.get_clock().now().to_msg(), persons))
        self.get_logger().debug("Detection callback: {} person(s) detected.".format(len(persons)))

    def evaluate_detections(self):
        """
        検知ウィンドウ内の情報から、平均検出数で動作分岐する  
        1人の場合はそのままゴール位置設定、複数人の場合は各検出領域について挙手判定を実施
        """
        if not self.detection_window:
            self.get_logger().info("5秒間で検出データなし")
            return

        # ウィンドウ内の全検出数の合計とメッセージ数で平均を算出
        total_persons = sum(len(persons) for (_, persons) in self.detection_window)
        avg_persons = total_persons / len(self.detection_window)
        self.get_logger().info("検知ウィンドウ内平均検出数: {:.2f}".format(avg_persons))

        # 1人検知の場合
        if int(round(avg_persons)) == 1:
            self.get_logger().info("1人検知: 対象者に対してゴール位置設定を実施")
            # 最新ウィンドウから1件選択（ここでは最初の検出を採用）
            target_detection = self.detection_window[-1][1][0]
            goal_pose = self.compute_goal_pose(target_detection)
            if goal_pose:
                self.goal_pose_pub.publish(goal_pose)
                self.get_logger().info("1人検知: /goal_pose へパブリッシュ")
        # 複数人検知の場合
        elif int(round(avg_persons)) > 1:
            self.get_logger().info("複数人検知: 挙手判定のため各検出領域をチェックします")
            target_detection = self.handle_hand_detection()
            if target_detection:
                goal_pose = self.compute_goal_pose(target_detection)
                if goal_pose:
                    self.goal_pose_pub.publish(goal_pose)
                    self.get_logger().info("複数人検知: 挙手確認済み対象の /goal_pose をパブリッシュ")
            else:
                self.get_logger().warn("挙手判定に失敗：手を挙げた人物が認識できませんでした")
        # ウィンドウのクリア（次回のウィンドウに備える）
        self.detection_window.clear()

    def handle_hand_detection(self):
        """
        複数人検知時の処理  
        最新の検知ウィンドウ（person検出のみ）から各検出の bounding box を画像から切り出し、
        挙手判定サービスに問い合わせを行う。  
        1人ずつ順次問い合わせ、挙手判定が True となった対象を返す。
        """
        if self.latest_color_image is None:
            self.get_logger().error("最新のカメラ画像が未取得のため、挙手判定ができません")
            return None

        # 最新ウィンドウ内の person 検出リスト（例：最新の受信結果を使用）
        person_detections = self.detection_window[-1][1]
        for i, detection in enumerate(person_detections):
            try:
                # detection.bbox (BoundingBox2D) から領域計算（中心とサイズから）
                cx = detection.bbox.center.x
                cy = detection.bbox.center.y
                w  = detection.bbox.size.x
                h  = detection.bbox.size.y
                x1 = int(cx - w / 2)
                y1 = int(cy - h / 2)
                x2 = int(cx + w / 2)
                y2 = int(cy + h / 2)
                self.get_logger().debug(
                    f"検出 {i}: bounding box: ({x1}, {y1}) ~ ({x2}, {y2})"
                )

                # 最新のカラ―画像から領域を切り出す
                cropped_image = self.latest_color_image[y1:y2, x1:x2]
                if cropped_image.size == 0:
                    self.get_logger().warn(f"検出 {i}: 画像切り出し結果が空です")
                    continue

                # 切り出した画像をROSのImageメッセージに変換
                cropped_img_msg = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
                cropped_img_msg.header = self.latest_color_header

                # 挙手判定サービスへのクライアント作成（サービス名と型は環境に合わせる）
                hand_client = self.create_client(CheckHand, '/hand_detection/check_hand')
                if not hand_client.wait_for_service(timeout_sec=3.0):
                    self.get_logger().error("挙手判定サービスが利用できません")
                    continue

                request = CheckHand.Request()
                request.image = cropped_img_msg
                # サービス呼び出し（同期的に待機）
                future = hand_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    response = future.result()
                    # response.is_hand_raised（bool型）により挙手判定
                    if response.is_hand_raised:
                        self.get_logger().info(f"検出 {i}: 挙手判定がTrue。対象採用")
                        return detection
                    else:
                        self.get_logger().info(f"検出 {i}: 挙手判定がFalse")
                else:
                    self.get_logger().error("挙手判定サービス呼び出しに失敗しました")
            except Exception as e:
                self.get_logger().error(f"挙手判定処理中エラー: {e}")
            # 各対象ごとに少し待機（オプション）
            time.sleep(0.5)
        # どの検出も挙手判定がTrueにならなかった場合
        return None

    def compute_goal_pose(self, detection) -> PoseStamped:
        """
        YOLO検出結果から対象者の位置を算出し、
        Realsense座標系から /map 座標系へ TF 変換し、PoseStamped として返す。  
        ※ 実際にはカメラ内部パラメータを用いた3次元再構成が必要ですが、ここではサンプルとして固定値を用いています。
        """
        # Realsenseカメラ座標系における対象者位置（例：カメラ前方1.0m位置）
        person_pose_cam = PoseStamped()
        person_pose_cam.header.frame_id = "realsense_frame"
        person_pose_cam.header.stamp = self.get_clock().now().to_msg()
        person_pose_cam.pose.position.x = 1.0  # 例：1m先
        person_pose_cam.pose.position.y = 0.0
        person_pose_cam.pose.position.z = 0.0
        person_pose_cam.pose.orientation.x = 0.0
        person_pose_cam.pose.orientation.y = 0.0
        person_pose_cam.pose.orientation.z = 0.0
        person_pose_cam.pose.orientation.w = 1.0

        try:
            # TF変換：realsense_frame から map 座標系へ
            transform = self.tf_buffer.lookup_transform("map",
                                                        "realsense_frame",
                                                        rclpy.time.Time())
            person_pose_map = do_transform_pose(person_pose_cam, transform)
            return person_pose_map
        except Exception as e:
            self.get_logger().error(f"TF変換エラー: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ApproachingPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ApproachingPerson node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
