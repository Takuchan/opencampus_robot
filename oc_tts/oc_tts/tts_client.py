#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from oc_tts_interfaces.srv import TTS

class TTSClient(Node):
    def __init__(self):
        super().__init__('tts_client')
        self.cli = self.create_client(TTS, 'tts_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTSサービスを待機中...')
        self.request = TTS.Request()

    def send_request(self, text):
        self.request.text = text
        self.future = self.cli.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    client = TTSClient()
    text = ' '.join(sys.argv[1:]) if len(sys.argv) > 1 else "Hello, this is a test of the TTS service."
    client.send_request(text)
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error("サービス呼び出しエラー: %r" % (e,))
            else:
                client.get_logger().info('レスポンス: %s' % response.message)
            break
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
