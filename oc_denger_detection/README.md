# oc_danger_detection

このパッケージは、OpenCampus Robotプロジェクトの危険検知モジュールです。センサーデータを使用して周囲の危険を検出し、適切な警告を生成します。

## 機能

- 障害物検知
- 衝突予測
- 危険通知アラート

## 依存関係

- ROS 2 (Humble推奨)
- sensor_msgs
- std_msgs
- geometry_msgs

## インストール方法

```bash
colcon build --packages-select oc_danger_detection
```

## 使用方法

以下のコマンドでノードを起動します：

```bash
ros2 run oc_danger_detection stopfrontperson.py
```
