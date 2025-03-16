import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os 
import datetime

class PhotoClient(Node):
    def __init__(self):
        super().__init__('photo_client')
        self.get_logger().info('photo_clientノードを起動しました。')
        self.subscription = self.create_subscription(
            Image,
            'oc/image/image_raw',  # 受信するトピック名
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.save_dir = 'oc_saved_images'

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def image_callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{self.save_dir}/{timestamp}.jpg'

            cv2.imwrite(filename,cv_image)
            self.get_logger().info(f'画像の保存に成功しました: {filename}')
        except Exception as e:
            self.get_logger().error(f'写真の保存に失敗しました')

def main(args=None):
    rclpy.init(args=args)
    node = PhotoClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()