#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from oc_tts_interfaces.srv import TTS  # カスタムサービス
import subprocess
import simpleaudio as sa

class TTSServer(Node):
    def __init__(self):
        super().__init__('tts_server')
        self.srv = self.create_service(TTS, 'tts_service', self.tts_callback)
        self.get_logger().info('TTSサービスサーバーが起動しました。')

        # 音声保存用ディレクトリ
        self.home_dir = os.path.expanduser('~')
        self.save_dir = os.path.join(self.home_dir, 'tts_voice_tmp')
        os.makedirs(self.save_dir, exist_ok=True)

        # スピーカーIDを指定（VOICEVOXのものに合わせて変更可能）
        self.speaker_id = 888753760

    def tts_callback(self, request, response):
        text = request.text
        self.get_logger().info(f"受信テキスト: {text}")

        try:
            # text.txt への書き込み
            text_path = os.path.join(self.save_dir, "text.txt")
            with open(text_path, "w", encoding="utf-8") as f:
                f.write(text)

            # クエリ生成と音声合成
            query_path = os.path.join(self.save_dir, "query.json")
            audio_path = os.path.join(self.save_dir, "tts_output.wav")

            cmd_audio_query = [
                "curl", "-s", "-X", "POST",
                f"127.0.0.1:10101/audio_query?speaker={self.speaker_id}",
                "--get", "--data-urlencode", f"text@{text_path}"
            ]
            query_json = subprocess.check_output(cmd_audio_query)

            with open(query_path, "wb") as f:
                f.write(query_json)

            cmd_synthesis = [
                "curl", "-s", "-H", "Content-Type: application/json",
                "-X", "POST", "-d", f"@{query_path}",
                f"127.0.0.1:10101/synthesis?speaker={self.speaker_id}"
            ]
            audio_wav = subprocess.check_output(cmd_synthesis)

            with open(audio_path, "wb") as f:
                f.write(audio_wav)

            # 音声再生
            wave_obj = sa.WaveObject.from_wave_file(audio_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()

            response.success = True
            response.message = "音声の再生に成功しました。"
        except Exception as e:
            self.get_logger().error(f"エラー発生: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TTSServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
