import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("カメラ映像のトピックを配信を始めました。")

        # パラメータ取得
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_id', 0),
                ('width', 640),
                ('height', 480),
                ('fps', 30),
                ('topic_name', '/camera/image_raw')
            ])

        camera_id = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        topic_name = self.get_parameter('topic_name').value

        # OpenCVの初期化
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit(1)

        # トピック初期化
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()

        # タイマー設定
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
        else:
            self.get_logger().warn("Failed to capture image")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraPublisherNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
