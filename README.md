# OpenCampus Robot プロジェクト

## 概要

| 完成図 | カメラ | LiDAR | 台車 | PC | 電源 |
|:---|:---|:---|:---|:---|:---|
| <img src="https://github.com/user-attachments/assets/86b69489-43be-4f11-a376-a5b328aca8e0"> | Realsense D455 | Livox MID360 | メガローバーVer3 | Dell Inspiron 5310 | 鉛蓄電池24V＋拡張電源基盤（LiDAR電源供給） |

![挙手検出と自動接近が可能なAI会話ロボット](https://github.com/user-attachments/assets/519200eb-dc76-4420-aca6-6ced097f531e)

このリポジトリは、オープンキャンパスでのデモンストレーション用に開発されたROS 2ベースのロボットシステムです。主な機能として、人物検出、挙手認識、音声合成、自律移動（ナビゲーション）を統合し、インタラクティブな体験を提供します。

---

# 事前準備

本リポジトリ `opencampus_robot` を、ROS 2ワークスペースの `src` ディレクトリ（例：`~/ros2_ws/src`）にクローンし、ワークスペースのルート（例：`~/ros2_ws`）で `colcon build` を実行してください。

```bash
cd ~/ros2_ws/src
git clone <リポジトリURL> opencampus_robot
cd ~/ros2_ws
colcon build --symlink-install
```

動作には以下の外部ライブラリやハードウェアセットアップが必要です。

## ハードウェア関連ドライバ・設定

### MID360 3D-LiDARを使用する場合
-   [こちらの記事](https://proc-cpuinfo.fixstars.com/2023/01/livox-mid360-ros1-ros2/) を参考にセットアップしてください。
-   **注意**: IPアドレス設定のため、USB-LANハブを別途用意することを推奨します。
-   ドライバ: `livox_ros_driver2`

### HOKUYO 2D-LiDARを使用する場合
-   ドライバ: [urg_node2](https://github.com/Hokuyo-aut/urg_node2) を使用してください。
-   シリアル通信に切り替えるため、`urg_node2` の `README.md` の手順に従って設定変更してください。
-   `urg_node2` の `config/params_serial.yaml` にて、測定範囲を半円（`angle_max: 3.14` → `angle_max: 1.57` など）に変更することを検討してください（ロボット前方のみ使用する場合）。
-   Nav2利用時には、コストマップ設定ファイル (`oc_megarover_bringup/param/nav2_params.yaml` 内の `scan` トピック設定) で `frame_id` がLiDARのフレーム名（例: `laser` や `hokuyo_link`）と一致しているか確認してください。

### Realsense D455を使用する場合
-   ドライバ: [Realsense-ROS公式リポジトリ](https://github.com/IntelRealSense/realsense-ros) を参考にセットアップしてください。
-   座標系（TF）に注意してください。詳細は[こちら](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems) を参照してください。

### メガローバーVer3 台車
-   ドライバ: [メガローバーVer3 ROS2パッケージ](https://github.com/vstoneofficial/megarover3_ros2) をセットアップしてください。
-   Micro-ROS Agent が必要です。

## ソフトウェア関連

-   **OS:** Ubuntu 22.04
-   **ROS:** ROS 2 Humble Hawksbill
-   **AI/Vision:**
    -   MediaPipe (Python) - 挙手検出用
    -   YOLOv11 (Ultralytics版) - 物体検出用
-   **音声合成:**
    -   [Aivis Speech Engine](https://github.com/Aivis-Project/AivisSpeech-Engine) - Docker CPU版推奨
-   **ナビゲーション:**
    -   Nav2 (ROS 2 Navigation Stack)
    -   SLAM Toolbox (地図作成用)

---

# 注意事項

-   本リポジトリには、RealsenseやMID360用の**URDFモデルは含まれていません**。RViz上では座標軸（TF）のみ確認できますが、基本的な動作には問題ありません。
-   ロボットの物理的な構成に合わせて、**TF（座標変換）を必ず設定し直してください。** 静的なTFは主に以下のlaunchファイルで設定されています。
    -   RealsenseのTF設定 → `oc_megarover_bringup/launch/bringup.launch.py` (`static_transform_publisher` ノード)
    -   Livox MID360（またはHOKUYO）のTF設定 → `oc_livox_to_pointcloud2/launch/livox_converter.launch.py` (`static_transform_publisher` ノード)

---

# 実行方法 (推奨手順)

各ステップは個別のターミナルで実行してください。

## 1. Micro-ROS Agent 起動（メガローバー台車制御基板との通信）

```bash
# /dev/ttyUSB0 は環境によって異なる場合があります
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
```
*ヒント: 同一ネットワーク内で複数PCを使用している場合、干渉を避けるために `export ROS_DOMAIN_ID=<任意の数字>` を各ターミナルで設定することを推奨します。*

## 2. メガローバーベースドライバ起動 (Odometry発行含む)

```bash
ros2 launch megarover3_bringup robot.launch.py
```

## 3. LiDARドライバと変換ノード起動

### MID360 の場合
```bash
# ドライバ起動
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# CustomMsg -> LaserScan/PointCloud2 変換 + TF発行
ros2 launch oc_livox_to_pointcloud2 livox_converter.launch.py
```

### HOKUYO 2D-LiDAR の場合
```bash
# ドライバ起動 (シリアル設定済みと仮定)
ros2 launch urg_node2 urg_node2.launch.py
# TF発行 (oc_livox_to_pointcloud2 の launch ファイルを流用または別途設定)
# 例: ros2 run tf2_ros static_transform_publisher 0.2 0 0.3 0 0 0 base_footprint laser
# 上記の座標 (0.2 0 0.3) は実際の取り付け位置に合わせてください
```

## 4. 台車制御用ノード + Realsense TF設定起動

```bash
ros2 launch oc_megarover_bringup bringup.launch.py
```
*これにより、`/cmd_vel_nav` (Nav2が出力) を `/cmd_vel` (メガローバードライバが購読) に中継するノードや、RealsenseカメラのTFが発行されます。*

## 5. 初期TF設定 (`map` -> `odom`)

SLAMやAMCLが起動するまで、`map` フレームと `odom` フレームを接続しておきます。

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
*注意: SLAM ToolboxやNav2 (AMCL) が起動すると、これらのノードが `map` -> `odom` のTFを提供するようになります。この静的TF発行は、それらが起動するまでの仮設定、またはSLAM Toolboxのマッピングモードで初期位置を固定するために使用します。*

## 6. Realsenseカメラノード起動

```bash
# カラー画像と位置合わせされた深度画像をパブリッシュ
ros2 launch realsense2_camera rs_ali・・・・
```

## 7. YOLO物体検出ノード起動

```bash
ros2 launch oc_recognition_yolo yolo.launch.py
```

## 8. 音声合成エンジン (Aivis) 起動

Dockerコンテナを起動します（初回以降はバックグラウンドで起動している場合もあります）。

```bash
docker run --rm -p '10101:10101' -v ~/.local/share/AivisSpeech-Engine:/home/user/.local/share/AivisSpeech-Engine-Dev ghcr.io/aivis-project/aivisspeech-engine:cpu-latest
```

## 9. ナビゲーション関連ノード起動

### A. マップ作成 (SLAM) - 初回またはマップを作り直す場合

```bash
# SLAM Toolbox を起動 (パラメータファイルは環境に合わせて要確認・修正)
ros2 launch slam_toolbox online_async_launch.py params_file:=$(ros2 pkg prefix oc_megarover_bringup)/share/oc_megarover_bringup/param/mapper_params_online_async.yaml use_sim_time:=false
```
*マッピング中は、ロボットを動かして環境を探査してください。RVizでマップが生成される様子を確認できます。*

**マップの保存:**
```bash
# マッピング完了後、マップを保存 (ファイル名は任意、例: my_map)
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
*保存された `my_map.yaml` と `my_map.pgm` を `oc_megarover_bringup/maps/` ディレクトリにコピーし、`oc_megarover_bringup/maps/map.yaml` の内容を `my_map.yaml` の内容で更新（またはファイル名を `map.yaml`, `map.pgm` に変更）してください。*

### B. ナビゲーション (Nav2) - 既存マップを使用する場合

```bash
# Nav2 スタックを起動 (oc_megarover_bringup/maps/map.yaml を使用)
ros2 launch oc_megarover_bringup nav2_with_map_launch.py use_sim_time:=false map:=$(ros2 pkg prefix oc_megarover_bringup)/share/oc_megarover_bringup/maps/map.yaml
```
*RVizが起動し、地図上にロボットが表示されます。「Nav2 Goal」ツールで目標地点を指定すると、ロボットが自律移動を開始します。初期位置がずれている場合は、「Initial Pose」ツールで現在位置を指定してください。*

## 10. 挙手検出・接近システム起動

```bash
ros2 launch oc_approching_me approaching_launch.py
```

## 11. 完了

すべてのノードが正常に起動すれば、システムが利用可能になります。カメラの前で手を挙げると、ロボットが音声で応答し、接近を開始します。


