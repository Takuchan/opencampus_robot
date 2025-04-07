心が折れそうなときに研究室におれそうになる、俺用のコマンド
```
ros2 launch livox_ros_driver2 msg_MID360_launch.py 
```
```
ros2 launch oc_livox_to_pointcloud2 livox_converter.launch.py
```
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
```
```
ros2 launch megarover3_bringup robot.launch.py
```
```
ros2 launch urg_node2 urg_node2.launch.py
```
```
ros2 run oc_megarover_bringup bringup
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
ナビゲーションのローンチを行う
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=/home/tk/ros2_ws/src/opencampus_robot/oc_megarover_bringup/maps/map.yaml params_file:=/home/tk/ros2_ws/src/opencampus_robot/oc_megarover_bringup/param/nav2_param.yaml
```


# 自動記念撮影ロボットシステム 🚀📸
（開発中）

本プロジェクトは、**ロボットが部屋の中を自動的に移動し、カメラの画角内に収まるように記念写真を撮影するシステム** を開発するものです。その一環として、**3D LiDAR を用いた人検出機能** を実装し、ロボットが適切な位置に移動することで、理想的な記念撮影を行うことを目指します。

## 🏗 システム概要

1. **人検出**  
   - 3D LiDAR を用いて **部屋内の人を検出** し、ロボットの移動計画を立案
   - 点群データをクラスタリングして **人物領域を抽出**
   
2. **自律移動**  
   - ROS2 を用いた **SLAM（自己位置推定）** により、ロボットが部屋のレイアウトを認識
   - 人を中心に適切な位置に移動し、構図を調整

3. **記念撮影**  
   - ロボットが **最適な位置に到達後、カメラを調整し自動撮影**
   - 必要なら複数の撮影パターンを提供

4. **学習型改善**  
   - ユーザーのフィードバックをもとに、ロボットの移動や撮影位置を調整
   - **Go 言語の Web API でフィードバックを収集** し、機械学習で改善

---

## 🚀 使用技術
- **ROS2 Humble** - ロボットの制御 & 3D LiDAR 処理
- **Python (Open3D, NumPy, PyTorch)** - 点群データ処理 & AI
- **Go 言語（バックエンド）** - フィードバック API & データ管理
- **Docker & MLOps** - 学習モデルの継続的改善

---


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

