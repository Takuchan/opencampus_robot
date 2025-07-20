# oc_recognition_yolo

written by Gemini2.5 pro →最終チェック＆校正Takuchan
## 概要

このパッケージは、YOLO (You Only Look Once) モデルを使用してリアルタイム物体検出を行います。特に、Intel Realsense D455のような深度カメラでの使用に最適化されています。検出結果（バウンディングボックス、クラス、信頼度、深度情報）およびRViz用の可視化マーカーをパブリッシュします。

## 機能
- YOLOで認識したオブジェクトを`/yolo_detection`で全て配信。型は`oc_recognition_yolo_interfaces`の`YOLODetection`型である。
-   カラー画像ストリームを購読します。
-   入力画像に対してYOLO推論（例：YOLOv11）を実行します。
-   クラス（例："person"）と信頼度の閾値に基づいて検出結果をフィルタリングします。
-   検出された物体の深度情報を抽出します（位置合わせされた深度データが利用可能であると仮定）。
-   バウンディングボックスが描画された処理済み画像をパブリッシュします。
-   座標、深度、クラスを含む構造化された検出データをパブリッシュします。
-   RViz内で検出された物体を3D空間に表現するための `visualization_msgs/Marker` メッセージをパブリッシュします (`yolo_marker_publisher` ノード)。

## トピックとサービス

| 種別         | 名前                         | 型                                                     | 説明                                                                 |
| :----------- | :--------------------------- | :----------------------------------------------------- | :------------------------------------------------------------------- |
| Subscription | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image`                                | 入力カラー画像ストリーム (`yolo.launch.py` の `image_topic` で設定可) |
| Subscription | (Implicit)                   | `sensor_msgs/msg/CameraInfo`                           | 深度計算に使用されるカメラ情報                                       |
| Subscription | (Implicit)                   | `tf2_msgs/msg/TFMessage`                               | 座標変換に使用されるTF                                               |
| Publication  | `/yolo/detection_image`      | `sensor_msgs/msg/Image`                                | 検出矩形が描画された出力画像 (`yolo.launch.py` の `output_topic` で設定可) |
| Publication  | `/yolo_detection`            | `oc_recognition_yolo_interfaces/msg/YOLODetection`     | 検出結果の構造化データ（クラス、信頼度、座標、深度、画像サイズ）     |
| Publication  | `/visualization_marker`      | `visualization_msgs/msg/Marker`                        | 検出物体の3D位置を示すRViz用マーカー (`yolo_marker_publisher` が発行) |

## 事前準備

-   カラー画像（および理想的には位置合わせされた深度情報）をパブリッシュするカメラノード（例：`realsense2_camera`）が必要です
-   `oc_recognition_yolo_interfaces` パッケージがビルドされている必要があります。
-   YOLOモデルファイル（例：`yolov11n.pt`）がノードからアクセス可能である必要があります（パスは `yolo.launch.py` で設定）。
-   正しいTF変換がパブリッシュされている必要があります（例：`map` -> `odom` -> `base_footprint` -> `realsense`）。`yolo.launch.py` の `frame_id` パラメータはカメラのフレームと一致させる必要があります。

## 起動方法

```bash
ros2 launch realsense2_camera rs_ali・・・・
ros2 launch oc_recognition_yolo yolo.launch.py
```
この launch ファイルは `yolo_publisher` ノードを起動します。`yolo_marker_publisher` ノードは必要に応じて別途起動する必要があります。

## パラメータ

`yolo_publisher` ノード (`yolo.launch.py` 経由) は以下のパラメータを使用します:

| パラメータ名             | 説明                               | 例         |
| :----------------------- | :--------------------------------- | :--------- |
| `model_path`             | YOLOモデルファイルへのパス         | `yolov11n.pt` |
| `confidence_threshold` | 検出の最小信頼度                   | `0.5`      |
| `target_class`           | 検出対象のクラス                   | `"person"` |
| `image_topic`            | 入力画像トピック名                 | `/camera/camera/color/image_raw` |
| `output_topic`           | 出力画像（注釈付き）トピック名     | `/yolo/detection_image` |
| `frame_id`               | カメラに関連付けられたTFフレームID | `"realsense"` |

`yolo_marker_publisher` ノードは以下のパラメータを使用します:

| パラメータ名             | 説明                     | 例    |
| :----------------------- | :----------------------- | :---- |
| `horizontal_fov_deg`   | カメラの水平視野角（度） | `85.0` |
| `vertical_fov_deg`     | カメラの垂直視野角（度） | `58.0` |
| `marker_scale`         | マーカーの大きさ         | `0.3` |
| `marker_lifetime_sec`  | マーカーの表示時間（秒） | `1.0` |
| `color_r`, `g`, `b`, `a` | マーカーの色（RGBA）     | `1.0, 0.0, 0.0, 0.8` |

## テスト

標準的なROS 2リンター（copyright, pep257, flake8）が設定されています。テストは以下のコマンドで実行します:

```bash
colcon test --packages-select oc_recognition_yolo
colcon test-result --all
```
