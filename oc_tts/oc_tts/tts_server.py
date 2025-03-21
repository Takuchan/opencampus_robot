#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from oc_tts_interfaces.srv import TTS  # 先ほど定義したカスタムサービスをインポート
from kokoro import KPipeline
import soundfile as sf
import simpleaudio as sa
import torch

class TTSServer(Node):
    def __init__(self):
        super().__init__('tts_server')
        self.srv = self.create_service(TTS, 'tts_service', self.tts_callback)
        # 必要に応じて、lang_codeやvoiceの指定を変更してください
        self.pipeline = KPipeline(lang_code='j')
        self.get_logger().info('TTSサービスサーバーが起動しました。')
        
        # ホームディレクトリ内に "tts_voice_tmp" ディレクトリを作成（存在しなければ）
        self.home_dir = os.path.expanduser('~')
        self.save_dir = os.path.join(self.home_dir, 'tts_voice_tmp')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"ディレクトリ {self.save_dir} を作成しました。")
        else:
            self.get_logger().info(f"ディレクトリ {self.save_dir} が既に存在します。")

    def tts_callback(self, request, response):
        text = request.text
        self.get_logger().info(f'受信テキスト: {text}')
        try:
            # Kokoroパイプラインで音声合成（ここでは最初のセグメントのみ利用）
            generator = self.pipeline(text, voice='af_heart', speed=1.2, split_pattern=r'\n+')
            audio_data = None
            for i, (gs, ps, audio) in enumerate(generator):
                audio_data = audio
                break  # 複数セグメントの場合は適宜連結処理を追加してください
            if audio_data is None:
                raise Exception("音声生成に失敗しました。")
            
            # ホームディレクトリ内の "tts_voice_tmp" に音声ファイルを保存
            wav_file = os.path.join(self.save_dir, 'tts_output.wav')
            sf.write(wav_file, audio_data, 24000)
            self.get_logger().info(f"音声ファイルを {wav_file} に保存しました。")
            
            # simpleaudioを利用して再生
            wave_obj = sa.WaveObject.from_wave_file(wav_file)
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
