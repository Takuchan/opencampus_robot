# oc_livox_to_pointcloud2

Written by Gemini2.5 pro → check Takuchan
## 概要
PointCloud2とLaserScanにMID360のデータを変換するパッケージ。以上。
また、launchを起動すると、MID360のTFが設定されます。このTFを利用してNav2を制御します。このパッケージはナビゲーションを行うために必要なものになります。
このパッケージは、Livox LiDARセンサーからの独自の `livox_ros_driver2/msg/CustomMsg` メッセージを、標準的なROS 2メッセージ型である `sensor_msgs/msg/PointCloud2` および `sensor_msgs/msg/LaserScan` に変換するノードを提供します。また、変換ノードを起動し、センサー用の静的TF変換を発行するためのlaunchファイルも含まれています。

## 機能

-   **Livox to PointCloud2:**
    -   `livox_ros_driver2/msg/CustomMsg` を購読します。
    -   カスタムポイントデータを標準的な `sensor_msgs/msg/PointCloud2` メッセージに変換します。
    -   オプションでポイントをフィルタリングします（現在の実装では、前方約180度（`x > 0` かつ `abs(y) <= x`）のポイントのみを保持します）。
    -   結果のPointCloud2メッセージをパブリッシュします。
-   **Livox to LaserScan:**
    -   `livox_ros_driver2/msg/CustomMsg` を購読します。
    -   ポイントをZ座標（高さ）に基づいてフィルタリングします。
    -   フィルタリングされた3Dポイントを2D平面に投影します。
    -   センサー周りの異なる角度に対する距離（range）を計算します。
    -   2D LiDARスキャンをシミュレートする `sensor_msgs/msg/LaserScan` メッセージをパブリッシュします。
-   **TF Publisher:**
    -   `base_footprint` から `mid360` への静的変換を発行します（launchファイルで設定可能）。

## トピックとサービス

| 種別         | 名前                   | 型                                      | 説明                                                                     |
| :----------- | :--------------------- | :-------------------------------------- | :----------------------------------------------------------------------- |
| Subscription | `/livox/lidar`         | `livox_ros_driver2/msg/CustomMsg`       | Livoxの生データ入力トピック (launchファイルで `/livox_pointcloud` からリマップ) |
| Publication  | `/livox/lidar/pcd2`    | `sensor_msgs/msg/PointCloud2`           | 変換されたPointCloud2データ出力トピック (launchファイルで `/converted_pointcloud2` からリマップ)。`livox_to_pointcloud2_node` が発行。 |
| Publication  | `/scan`                | `sensor_msgs/msg/LaserScan`             | 合成された2D LaserScanデータ出力トピック。`livox_to_scan_node` が発行。      |
| Publication  | `/tf_static`           | `tf2_msgs/msg/TFMessage`                | LiDARセンサー用の静的変換を発行 (例: `base_footprint` -> `mid360`)。       |

## 事前準備

-   **Livoxドライバ:** `livox_ros_driver2` パッケージがインストールされ、実行中で、`CustomMsg` データを発行している必要があります。
    ```bash
    # MID360用の起動例
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
    ```
-   **TF:** ロボットのオドメトリまたはローカリゼーションシステムによってパブリッシュされる `base_footprint` フレームがTFツリーに存在する必要があります。

## 起動方法

```bash
ros2 launch oc_livox_to_pointcloud2 livox_converter.launch.py
```
このlaunchファイルは以下を起動します:
-   静的TF発行ノード (`base_footprint` -> `mid360`)。
-   `livox_to_pointcloud2_node`。
-   `livox_to_scan_node`。

**注意:** `mode` launch引数は宣言されていますが、現在ノードを条件付きで起動するためには使用されていません。デフォルトでは両方の変換ノードが起動します。

## パラメータ

-   **`livox_to_scan_node`:**
    -   `min_z_`, `max_z_`: LaserScan変換のためにポイントをフィルタリングするZ座標範囲 (`livox_to_scan.cpp` 内でハードコード)。
    -   `angle_min_`, `angle_max_`, `num_samples_`: LaserScanの視野角と解像度 (`livox_to_scan.cpp` 内でハードコード)。
-   **Launchファイル (`livox_converter.launch.py`):**
    -   TF引数: `base_footprint` に対する `mid360` フレームの位置と向き。**ロボットの物理的なセットアップに合わせてこれらの値を調整してください。**

## テスト

現在、`package.xml` で設定されている基本的なリンティング以外に、特定のユニットテストや統合テストは提供されていません。リンターを実行するには:

```bash
colcon test --packages-select oc_livox_to_pointcloud2
colcon test-result --all
```