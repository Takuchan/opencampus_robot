import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import struct
import os
import time

from ament_index_python.packages import get_package_share_directory



class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/converted_pointcloud2',
            self.pointcloud_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_clear_time = time.time()
        self.is_stopped = False
        self.alert_played = False

    def play_alert_sound(self):
        if not self.alert_played:
            sound_path = os.path.join(
                get_package_share_directory('oc_denger_detection'),
                'sounds',
                'uwabikkurishita.wav'
            )
            os.system(f"aplay {sound_path}")
            self.alert_played = True

    def pointcloud_callback(self, msg):
        danger = False
        for i in range(0, len(msg.data), msg.point_step):
            x = struct.unpack_from('f', msg.data, i + msg.fields[0].offset)[0]
            y = struct.unpack_from('f', msg.data, i + msg.fields[1].offset)[0]
            z = struct.unpack_from('f', msg.data, i + msg.fields[2].offset)[0]

            distance = (x ** 2 + y ** 2 + z ** 2) ** 0.5
            if distance < 0.6:
                danger = True
                break

        current_time = time.time()

        if danger:
            self.last_clear_time = current_time
            if not self.is_stopped:
                self.stop_robot()
                self.play_alert_sound()
                self.is_stopped = True
        else:
            if self.is_stopped and (current_time - self.last_clear_time) > 5:
                self.resume_robot()
                self.is_stopped = False
                self.alert_played = False

    def stop_robot(self):
        self.get_logger().info("🔴 障害物検知！緊急停止！")
        self.play_alert_sound()
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def resume_robot(self):
        self.get_logger().info("🟢 安全確認。ロボット再始動！")
        # ここでは再始動時の動きは止まったまま。必要に応じて速度を設定。

    def play_alert_sound(self):
        if not self.alert_played:
            os.system("aplay uwabikkurishita.wav")  # 適切な音声ファイルのパスに変更
            self.alert_played = True

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
