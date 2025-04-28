![挙手検出と自動接近が可能なAI会話ロボット](https://github.com/user-attachments/assets/519200eb-dc76-4420-aca6-6ced097f531e)

心が折れそうなときに研究室におれそうになる、俺用のコマンド
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
```
ODOMの発行
```
ros2 launch megarover3_bringup robot.launch.py
```
```
ros2 launch livox_ros_driver2 msg_MID360_launch.py 
```
```
ros2 launch oc_livox_to_pointcloud2 livox_converter.launch.py
```
url_node2を確認する(今は不要）
```
ros2 launch urg_node2 urg_node2.launch.py
```
cmd_vel:=rover_twistに変更
RealsenseのTFを設定する
```
ros2 launch oc_megarover_bringup bringup.launch.py
```
TFを揃える
```
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom
```

YOLO+Realsenseのノードを実行する
```
ros2 launch realsense2_camera rs_align_depth_launch.py 

```
音声合成のDockerコンテナの起動
```
 docker run --rm -p '10101:10101'   -v ~/.local/share/AivisSpeech-Engine:/home/user/.local/share/AivisSpeech-Engine-Dev   ghcr.io/aivis-project/aivisspeech-engine:cpu-latest
```
## マップ作成
マップを作成したい場合
```
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/tk/ros2_ws/src/opencampus_robot/oc_megarover_bringup/param/mapper_params_online_sync.yaml
```
保存時
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```
## ナビゲーション
ナビゲーションのローンチを行う(ただし、oc_megarover_bringupパッケージのmap.yamlで保存すること。paramのmapのパス名に注意)
```
ros2 launch oc_megarover_bringup nav2_with_map_launch.py
```




## 🛠 インストール & セットアップ

### 1️⃣ **ROS2 のセットアップ**
```
# ROS2 Humble のインストール（Ubuntu）
sudo apt update && sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash

# ROS2 ワークスペースを作成 & ビルド
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2️⃣ **Python の環境セットアップ**
```
# 必要なライブラリをインストール
pip install numpy open3d torch torchvision
```

### 3️⃣ **Go サーバーのセットアップ**

---

## 🎮 使い方

---

## 🔍 今後の改善点

---

