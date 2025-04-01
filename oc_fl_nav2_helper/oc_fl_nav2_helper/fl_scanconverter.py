import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Header
import math

class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan')
        self.sub = self.create_subscription(PointCloud2, '/cloud_registered', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.count = 0
    def callback(self, msg):
        self.count = self.count + 1
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(0.5)  # 解像度
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 50.0
        ranges = [float('inf')] * int((scan.angle_max - scan.angle_min) / scan.angle_increment)

        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[0], point[1], point[2]

            # 高さ方向でフィルタ（zが一定範囲内の点のみ使う）
            if abs(z) > 0.1:
                continue

            angle = math.atan2(y, x)
            dist = math.hypot(x, y)

            index = int((angle - scan.angle_min) / scan.angle_increment)
            if 0 <= index < len(ranges):
                if dist < ranges[index]:
                    ranges[index] = dist

        scan.ranges = ranges
        self.get_logger().debug(f'scanデータはこちら{self.count}')
        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
