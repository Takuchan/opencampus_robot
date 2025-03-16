import os
import cv2
import datetime
import numpy as np
import pytest
import rclpy
from oc_photo_apiserver.photoclient import PhotoClient  # プログラムのファイル名が photo_client.py と仮定

# ROS の初期化／終了を各テストで自動的に行うフィクスチャ
@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# ダミーロガーを用意してログ出力を検証
class DummyLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, msg):
        self.info_messages.append(msg)

    def error(self, msg):
        self.error_messages.append(msg)


# ダミーの CvBridge クラス。常に指定のダミー画像（numpy 配列）を返す
class DummyCvBridge:
    def __init__(self, dummy_image):
        self.dummy_image = dummy_image

    def imgmsg_to_cv2(self, msg, desired_encoding):
        return self.dummy_image


# ダミーの Image メッセージ（sensor_msgs.msg.Image を模す）
class DummyImage:
    pass


def test_save_directory_created(tmp_path, monkeypatch):
    """
    PhotoClient の初期化時に保存用ディレクトリ 'saved_images' が作成されることを確認するテスト
    """
    # テスト用の一時ディレクトリに移動
    monkeypatch.chdir(tmp_path)
    node = PhotoClient()
    # ノード初期化時に 'saved_images' ディレクトリが存在するはず
    assert os.path.exists('saved_images'), "'saved_images' ディレクトリが作成されていません"


def test_image_callback_success(monkeypatch, tmp_path):
    """
    image_callback() の正常系テスト。ダミーの画像を用いて cv2.imwrite の呼び出しとログ出力を検証
    """
    monkeypatch.chdir(tmp_path)
    # ダミー画像（10x10 の黒画像）
    dummy_image = np.zeros((10, 10, 3), dtype=np.uint8)

    node = PhotoClient()
    # ダミー CvBridge を注入して、必ずダミー画像が返るようにする
    node.bridge = DummyCvBridge(dummy_image)

    # ダミーのロガーを注入してログ出力を検証
    dummy_logger = DummyLogger()
    monkeypatch.setattr(node, "get_logger", lambda: dummy_logger)

    # cv2.imwrite の呼び出しをモニタリングするために、ダミー関数に置き換え
    write_called = False

    def fake_imwrite(filename, image):
        nonlocal write_called
        write_called = True
        # ファイル名が 'saved_images/' で始まっていることを確認
        assert filename.startswith("saved_images/")
        # 渡された画像がダミー画像と一致していることを確認
        np.testing.assert_array_equal(image, dummy_image)
        return True

    monkeypatch.setattr(cv2, "imwrite", fake_imwrite)

    # ダミーの Image メッセージを生成してコールバックを呼び出す
    dummy_msg = DummyImage()
    node.image_callback(dummy_msg)

    assert write_called, "cv2.imwrite が呼ばれていません"
    # 成功ログが出力されているか確認
    assert any("画像の保存に成功しました" in msg for msg in dummy_logger.info_messages), "成功ログが出力されていません"


def test_image_callback_failure(monkeypatch, tmp_path):
    """
    CvBridge での変換処理で例外が発生した場合、エラーログが出力されることを検証するテスト
    """
    monkeypatch.chdir(tmp_path)
    node = PhotoClient()

    # 例外を発生させるダミー CvBridge を注入
    class FailingCvBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding):
            raise Exception("変換エラー")

    node.bridge = FailingCvBridge()

    dummy_logger = DummyLogger()
    monkeypatch.setattr(node, "get_logger", lambda: dummy_logger)

    dummy_msg = DummyImage()
    node.image_callback(dummy_msg)

    # エラーログに「写真の保存に失敗しました」が含まれているか検証
    assert any("写真の保存に失敗しました" in msg for msg in dummy_logger.error_messages), "エラーログが出力されていません"
