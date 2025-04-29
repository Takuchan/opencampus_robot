# oc_megarover_bringup
Written by Gemini2.5 PRO → Check Takuchan.
## 概要

メガローバーVer3.0のロボットをROS2で操作するためのlaunchファイル、設定ファイル、およびユーティリティノードを提供します。特に、ナビゲーション（Nav2）とマッピング（SLAM Toolbox）の統合に焦点がされています。

## 機能

-   **Launchファイル:**
    -   `bringup.launch.py`: 基本的なTF変換（例：`base_footprint` -> `realsense`）や、`cmd_vel`リレーなどのヘルパーノードを起動します。
    -   `megarover_mapping.launch.py`: マップ作成のためにSLAM Toolboxを起動します。
    -   `nav2_with_map_launch.py`: 既存のマップとカスタムパラメータを使用してNav2スタックを起動します。
    -   `navigation_launch.py`: (内部用) 他のlaunchファイルからインクルードされるNav2のlaunchファイル。
-   **設定ファイル:**
    -   `param/nav2_params.yaml`: Nav2スタック（コントローラー、プランナー、コストマップなど）のカスタムパラメータ。
    -   `param/mapper_params_online_async.yaml`: SLAM Toolbox用のパラメータ。
    -   `maps/`: 保存されたマップファイル（`map.yaml`, `map.pgm`）。この場所に保存すると、ナビゲーションが可能になります。
    -   `rviz/`: ナビゲーションとマッピングの可視化のためのRViz設定ファイル。
-   **ノード:**
    -   `bringup`: (`bringup.py` の実行ファイル) `cmd_vel`の処理や基本的なロボットインターフェースタスクに関連する可能性があります。
    -   `realsensetfinit`: (`realsenseinittf.py` の実行ファイル) Realsenseカメラ用の静的TFを発行します（`tf2_ros static_transform_publisher` の代替）。
    -   `point2nav2`: (`point2nav2.py` の実行ファイル) ポイントに基づいてゴールを送信することに関連する可能性があります（要確認）。

## トピックとサービス

このパッケージは主に標準的なROS 2ノード（Nav2, SLAM, TF, RViz）を起動・設定します。これらのシステムが提供/使用するトピックやサービスと連携します。

**主要なトピック**

| トピック名        | 型                                           | 説明                               |
| :---------------- | :------------------------------------------- | :--------------------------------- |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage`                     | 座標変換情報                       |
| `/scan`           | `sensor_msgs/msg/LaserScan`                  | SLAMおよびNav2コストマップの入力   |
| `/odom`           | `nav_msgs/msg/Odometry`                      | ロボットベースからのオドメトリ情報 |
| `/cmd_vel`        | `geometry_msgs/msg/Twist`                    | ロボットベースへの速度指令         |
| `/map`            | `nav_msgs/msg/OccupancyGrid`                 | SLAMまたはmap_serverからの地図データ |
| `/initialpose`    | `geometry_msgs/msg/PoseWithCovarianceStamped` | ロボットの初期姿勢設定             |
| `/goal_pose`      | `geometry_msgs/msg/PoseStamped`              | ナビゲーションゴールの送信         |
| `/clicked_point`  | `geometry_msgs/msg/PointStamped`             | RVizでクリックされた点             |

**発行される静的TFの例 (`bringup.launch.py` より)**

-   `base_footprint` -> `realsense`

## 事前準備

-   **ハードウェア/ドライバ:** MegaRover Ver3ハードウェアが接続され、`megarover3_ros2` パッケージが実行されていること。
    ```bash
    # ロボット基板との通信
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
    # ベースドライバとオドメトリの起動
    ros2 launch megarover3_bringup robot.launch.py
    ```
-   **LiDAR:** `/scan` データをパブリッシュするLiDARノード（例：`urg_node2` または `oc_livox_to_pointcloud2`）が実行されていること。
    ```bash
    # Livox MID360 (oc_livox_to_pointcloud2経由) の例
    ros2 launch oc_livox_to_pointcloud2 livox_converter.launch.py
    # Hokuyoの例
    # ros2 launch urg_node2 urg_node2.launch.py
    ```
-   **ナビゲーション/マッピングパッケージ:** Nav2およびSLAM Toolboxパッケージがインストールされていること。
    ```bash
    sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox
    ```
-   **TF:** 必須のベースTF（`odom` -> `base_footprint`）がロボットドライバ（`megarover3_bringup`）によってパブリッシュされていること。初期状態では静的な `map` -> `odom` のTFが必要になる場合があります。
    ```bash
    # SLAM/AMCLがまだ提供していない場合の初期 map->odom TF
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```

## 実行例

### マッピング (地図作成)

```bash
# カスタムパラメータとRVizでSLAM Toolboxを起動
ros2 launch oc_megarover_bringup megarover_mapping.launch.py
# 完了したらマップを保存
ros2 run nav2_map_server map_saver_cli -f ~/map # 現在のディレクトリに保存
# 保存されたマップファイル (map.yaml, map.pgm) を oc_megarover_bringup/maps/ に移動し、
# 必要に応じて oc_megarover_bringup/maps/map.yaml を更新してください。
```

### ナビゲーション

```bash
# 保存されたマップとカスタムパラメータでNav2を起動
ros2 launch oc_megarover_bringup nav2_with_map_launch.py map:=$(ros2 pkg prefix oc_megarover_bringup)/share/oc_megarover_bringup/maps/map.yaml
```

### 基本的な起動 (TF)

```bash
# パッケージで定義されたTF発行ノードなどを起動
ros2 launch oc_megarover_bringup bringup.launch.py
```

## テスト

標準的なROS 2リンター（copyright）が設定されています。テストは以下のコマンドで実行します:

```bash
colcon test --packages-select oc_megarover_bringup
colcon test-result --all
```
