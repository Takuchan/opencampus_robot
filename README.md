# Overview

| 完成図 | カメラ | LiDAR | 台車 | PC | 電源 |
|:---|:---|:---|:---|:---|:---|
| <img src="https://github.com/user-attachments/assets/86b69489-43be-4f11-a376-a5b328aca8e0"> | Realsense D455 | Livox MID360 | メガローバーVer3 | Dell Inspiron 5310 | 鉛蓄電池24V＋拡張電源基盤（LiDAR電源供給） |

![挙手検出と自動接近が可能なAI会話ロボット](https://github.com/user-attachments/assets/519200eb-dc76-4420-aca6-6ced097f531e)

---

# 事前準備

本リポジトリ`opencampus_robot`を、`ros2_ws/src`にクローンし、`colcon build`してください。

動作には以下の外部ライブラリやハードウェアセットアップが必要です。

## ハードウェア関連

### MID360 3D-LiDARを使用する場合
- [こちらの記事](https://proc-cpuinfo.fixstars.com/2023/01/livox-mid360-ros1-ros2/)を参考にセットアップしてください。
- **注意**：IPアドレス設定のため、USB-LANハブを別途用意することを推奨します。

### HOKUYO 2D-LiDARを使用する場合
- [urg_node2](https://github.com/Hokuyo-aut/urg_node2)を使用してください。
- シリアル通信に切り替えるため、`README.md`の手順に従って設定変更してください。
- `config/params_serial.yaml`にて、測定範囲を半分（3.14 → 1.57）に変更してください。
- Nav2利用時には、`frame_id: 'laser'`を`frame_id: 'mid360'`に変更してください。

### Realsense D455を使用する場合
- [Realsense-ROS公式リポジトリ](https://github.com/IntelRealSense/realsense-ros)を参考にセットアップしてください。
- 座標系（TF）に注意してください。詳細は[こちら](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems)。

## ソフトウェア関連

- ROS2 Humble (Ubuntu22.04)
- MediaPipe
- YOLOv11 (Ultralytics版)
- [Aivis Speech Engine](https://github.com/Aivis-Project/AivisSpeech-Engine)（Docker CPU版推奨）
- [メガローバーVer3 ROS2パッケージ](https://github.com/vstoneofficial/megarover3_ros2)

---

# 注意事項

本リポジトリには、RealsenseやMID360用の**URDFモデルはありません**。RViz上では座標軸（TF）のみ確認できますが、動作には問題ありません。

ただし、ロボットの環境に合わせて**TFを必ず設定し直してください。**

- RealsenseのTF設定 → `oc_megarover_bringup/launch/bringup.launch.py`
- Livox（またはHOKUYO）のTF設定 → `oc_livox_to_pointcloud2/launch/livox_converter.launch.py`

---

# 実行方法

## 1. 通信開始（メガローバー台車制御基板）

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
```
※ 同一ネットワーク内で複数PCを使用している場合、ROS_DOMAIN_IDの変更を推奨します。

## 2. ODOMの発行
```
ros2 launch megarover3_bringup robot.launch.py
```
## 3. LiDARの実行
```
ros2 launch livox_ros_driver2 msg_MID360_launch.py 
ros2 launch oc_livox_to_pointcloud2 livox_converter.launch.py
```

HOKUYO 2D-LiDARを利用する場合は以下のみ
```
ros2 launch urg_node2 urg_node2.launch.py
```
## 4. 台車制御を`cmd_vel`のトピックで制御できるようにする＋RealsenseのTFを設定する
```
ros2 launch oc_megarover_bringup bringup.launch.py
```

## 5.TFを揃える(1回目)
```
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom
```

## 6. YOLO+Realsenseのノードを実行する
```
ros2 launch realsense2_camera rs_align_depth_launch.py 

```
## 7. 音声合成のDockerコンテナの起動（一度インストール済みだと常時起動済みの場合がある）
```
 docker run --rm -p '10101:10101'   -v ~/.local/share/AivisSpeech-Engine:/home/user/.local/share/AivisSpeech-Engine-Dev   ghcr.io/aivis-project/aivisspeech-engine:cpu-latest
```

## 8. ナビゲーション
### マップ作成
マップを作成したい場合（実行パスをご自身の環境に応じて必ず変更してください）
```
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/tk/ros2_ws/src/opencampus_robot/oc_megarover_bringup/param/mapper_params_online_sync.yaml
```
TFを揃える(2回目)
```
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom
```
保存時
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```
注意： 保存場所は`slam_toolbox`を実行したカレントディレクトリ内です。

### ナビゲーション
ナビゲーションのローンチを行う(ただし、oc_megarover_bringupパッケージのmap.yamlで保存すること。paramのmapのパス名に注意)
```
ros2 launch oc_megarover_bringup nav2_with_map_launch.py
```
TFを揃える(2回目)
```
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom
```

## 9. 挙手判定後、接近システム
```
ros2 launch approching_me approaching_launch.py
```

## 10. 完了
しばらくすると利用可能になります。


