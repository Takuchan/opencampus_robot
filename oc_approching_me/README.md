# oc_approching_me

Written by Gemini2.5 pro → check Takuchan
## 概要

このパッケージは、OpenCampus Robotがコンピュータビジョンを用いて挙手している人物を検出し、Nav2を使用してその人物に接近することを可能にします。物体検出（YOLO）、手の姿勢推定、テキスト読み上げ（TTS）、ナビゲーション機能を統合しています。

## 機能

-   **人物検出:** YOLOの検出結果 (`oc_recognition_yolo`) を購読し、カメラ視野内の人物を特定します。
-   **挙手検出:** 検出された人物が手を挙げているかどうかを判断するために、サービス (`/hand_detection/check_hand`、`raisehands` ノードが提供) を使用します。→カスタムトピックの型です。`oc_approaching_interfaces`に型が保存されています。
-   **ターゲット捕捉:** 挙手が確認されると、YOLO検出からの深度データとTF変換を使用して、数フレームにわたって人物の3D位置を収集します。平均位置を計算し、安定性を確認します。
-   **ナビゲーション:** 計算された安定した位置をNav2スタック (`/navigate_to_pose`) にゴールとして送信し、ロボットを人物に向かって移動させます。
-   **ユーザーフィードバック:** TTS (`/tts_service`) を使用して音声フィードバック（例：「移動します」、「少々お待ちください」、「移動が完了しました」）を提供します。
-   **エンターテイメント:** ナビゲーション中にBGMを再生します (`pygame` を使用)。
-   **可視化:** 計算されたゴール位置を示すために、マーカーとTFフレームをRVizにパブリッシュします。
-   **アイドル時プロンプト:** ロボットが一定時間アイドル状態の場合、定期的にユーザーに手を挙げるよう促します。

## トピック、サービス、アクション

| 種別            | 名前                             | 型                                                         | 説明                                                                     |
| :-------------- | :------------------------------- | :--------------------------------------------------------- | :----------------------------------------------------------------------- |
| Subscription    | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image`                                    | 入力カラー画像 (パラメータ: `rgb_topic`)                                   |
| Subscription    | `/yolo_detection`                | `oc_recognition_yolo_interfaces/msg/YOLODetection`         | 入力YOLO検出結果 (パラメータ: `yolo_topic`)                                |
| Subscription    | `/tf`, `/tf_static`              | `tf2_msgs/msg/TFMessage`                                   | 座標変換用 (例: `camera_frame` から `map`)                               |
| Publication     | `/visualization_marker`          | `visualization_msgs/msg/Marker`                            | 計算されたゴール位置にマーカーを発行                                     |
| Publication     | `/tf`                            | `tf2_msgs/msg/TFMessage`                                   | `map` に対してゴール位置に一時的なTFフレーム (`goal_frame`) を発行         |
| Service Client  | `/hand_detection/check_hand`     | `oc_approaching_interfaces/srv/CheckHand`                  | 指定された画像クロップに対する挙手状態を要求                             |
| Service Client  | `/tts_service`                   | `oc_tts_interfaces/srv/TTS`                                | 音声フィードバックのための音声合成を要求                                 |
| Action Client   | `/navigate_to_pose`              | `nav2_msgs/action/NavigateToPose`                          | Nav2スタックにナビゲーションゴールを送信                                 |

## 事前準備

-   **物体検出:** `oc_recognition_yolo` パッケージが実行中で、`/yolo_detection` をパブリッシュしていること。
    ```bash
    ros2 launch oc_recognition_yolo yolo.launch.py
    ```
-   **カメラ:** カメラノード（例：`realsense2_camera`）が `/camera/camera/color/image_raw`（または設定されたトピック）にカラー画像をパブリッシュしていること。
    ```bash
    ros2 launch realsense2_camera rs_ali・・・
    ```
-   **挙手検出サービス:** このパッケージの `raisehands` ノードが実行されていること。
    ```bash
    ros2 run oc_approching_me raisehands
    ```
-   **TTSサービス:** TTSサービスプロバイダ（`oc_tts`）が実行中で、`/tts_service` を提供していること。
    ```bash
    ros2 run oc_tts tts_server.py
    ```
-   **ナビゲーション:** Nav2スタックが実行中で、設定されていること（例：`oc_megarover_bringup` 経由）。
    ```bash
    ros2 launch oc_megarover_bringup nav2_with_map_launch.py
    ```
-   **TF:** 正しいTF変換が利用可能であること。特に `map` -> `base_footprint` -> `realsense`（または設定された `camera_frame_id`）。
-   **インターフェース:** `oc_recognition_yolo_interfaces`, `oc_approaching_interfaces`, `oc_tts_interfaces` パッケージがビルドされていること。
-   **Pythonライブラリ:** 音楽再生のための `pygame` (`pip install pygame`)。

## 起動方法

```bash
ros2 launch oc_approching_me approaching_launch.py
```
このlaunchファイルは通常、`approaching_person` ノード、`raisehands` サービス、YOLOノード、TTSノードを起動します。

## パラメータ

`approaching_person` ノードはいくつかのパラメータを使用します（`approachingperson.py` で定義され、`config/approaching_params.yaml` やlaunchファイルで上書きされる可能性があります）:

| パラメータ名                 | 説明                                               | 例                               |
| :--------------------------- | :------------------------------------------------- | :------------------------------- |
| `rgb_topic`                | 入力RGB画像トピック名                              | `/camera/camera/color/image_raw` |
| `yolo_topic`               | 入力YOLO検出トピック名                             | `/yolo_detection`                |
| `marker_frame_id`          | マーカーとゴールのTFフレーム（通常 "map"）         | `"map"`                          |
| `camera_frame_id`          | 深度を提供するカメラのTFフレーム                   | `"realsense"`                    |
| `idle_timeout`             | ユーザーにプロンプトするまでのアイドル時間（秒）   | `30.0`                           |
| `music_file`               | ナビゲーション中に再生する音楽ファイルのパス       | (launchで設定)                   |
| `horizontal_fov_deg`       | カメラの水平視野角（度）                           | `85.0`                           |
| `vertical_fov_deg`         | カメラの垂直視野角（度）                           | `58.0`                           |
| `marker_scale`             | マーカーのスケール                                 | `0.3`                            |
| `marker_lifetime_sec`      | マーカーの表示時間（秒）                           | `3.0`                            |
| `color_r`, `g`, `b`, `a`   | マーカーの色（RGBA）                               | `1.0, 0.0, 0.0, 0.8`             |
| `frame_collect_count`      | 位置を平均化するためのフレーム数                   | `30`                             |
| `max_position_deviation` | 安定性チェックのための収集位置の最大許容偏差       | `0.5`                            |

## テスト

標準的なROS 2リンター（copyright, pep257）が設定されています。テストは以下のコマンドで実行します:

```bash
colcon test --packages-select oc_approching_me
colcon test-result --all
```
