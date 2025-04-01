#!/usr/bin/env python3
import os
import numpy as np
import open3d as o3d
import tensorflow as tf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from huggingface_hub import hf_hub_download
from huggingface_hub import from_pretrained_keras


class PointNetROS2Node(Node):
    def __init__(self):
        super().__init__('pointnet_ros2_node')
        
        # 1. Hugging Face Hub からモデルファイルをダウンロードしてロード
        self.get_logger().info("モデルをダウンロード中...")
        self.model = from_pretrained_keras("keras-io/PointNet")
        self.get_logger().info("PointNet モデルをロードしました。")
        
        # 2. ROS2 の PointCloud2 型のトピック（例: "point_cloud"）をサブスクライブ
        self.subscription = self.create_subscription(
            PointCloud2,
            'point_cloud',  # ※適宜トピック名を変更してください
            self.point_cloud_callback,
            10)
        self.subscription  # 未使用警告回避
        
        # 3. Open3D ビジュアライザの初期化
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="点群とバウンディングボックス", width=800, height=600)
        self.pcd_geometry = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd_geometry)
        self.bounding_box_geometry = None  # バウンディングボックス用
        
        # 4. 定期タイマーでビジュアライザを更新（0.1秒間隔）
        self.timer = self.create_timer(0.1, self.update_visualization)
        
        # 5. 最新の点群データとバウンディングボックスを保持する変数
        self.latest_points = None
        self.latest_bounding_box = None
        
        # 6. クラスIDとクラス名のマッピング（例示）
        self.class_mapping = {
            0: "ベッド",
            1: "椅子",
            2: "机",
            3: "ソファ",
            4: "キャビネット",
            5: "その他"
        }
    
    def point_cloud_callback(self, msg: PointCloud2):
        # 点群メッセージを NumPy 配列 (Nx3) に変換
        points = self.pointcloud2_to_xyz(msg)
        if points.size == 0:
            self.get_logger().warn("空の点群を受信しました。")
            return
        self.get_logger().info(f"受信した点群の点数: {points.shape[0]}")
        
        # 1024点にサンプリング（点数が多い場合はランダム抽出、少ない場合は繰り返し補完）
        N = 1024
        if points.shape[0] >= N:
            idx = np.random.choice(points.shape[0], N, replace=False)
            sampled_points = points[idx, :]
        else:
            repeats = int(np.ceil(N / points.shape[0]))
            sampled_points = np.concatenate([points] * repeats, axis=0)[:N, :]
        
        # 前処理：重心を原点に移動、最大距離で正規化
        centroid = np.mean(sampled_points, axis=0)
        sampled_points = sampled_points - centroid
        max_dist = np.max(np.sqrt(np.sum(sampled_points**2, axis=1)))
        if max_dist > 0:
            sampled_points = sampled_points / max_dist
        
        # モデル入力用に形状変換 (1, N, 3)
        input_points = np.expand_dims(sampled_points, axis=0)
        
        # 推論実行
        predictions = self.model.predict(input_points)
        predicted_class = np.argmax(predictions, axis=-1)[0]
        recognized_class = self.class_mapping.get(predicted_class, "不明")
        self.get_logger().info(f"予測されたクラスID: {predicted_class}  認識結果: {recognized_class}")
        
        # 受信した元の点群から Open3D 点群を作成し、バウンディングボックスを計算
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        bounding_box = pcd.get_axis_aligned_bounding_box()
        bounding_box.color = (1, 0, 0)  # バウンディングボックスは赤色
        
        # 最新データとして保存（ビジュアライザ更新用）
        self.latest_points = points
        self.latest_bounding_box = bounding_box

    def update_visualization(self):
        if self.latest_points is not None:
            # 点群ジオメトリの更新
            self.pcd_geometry.points = o3d.utility.Vector3dVector(self.latest_points)
            self.vis.update_geometry(self.pcd_geometry)
            
            # バウンディングボックスの更新（既存のものがあれば削除）
            if self.bounding_box_geometry is not None:
                self.vis.remove_geometry(self.bounding_box_geometry, reset_bounding_box=False)
            if self.latest_bounding_box is not None:
                self.bounding_box_geometry = self.latest_bounding_box
                self.vis.add_geometry(self.bounding_box_geometry)
            
            self.vis.poll_events()
            self.vis.update_renderer()
    
    def pointcloud2_to_xyz(self, cloud_msg: PointCloud2):
        """
        sensor_msgs/PointCloud2 のデータを (N,3) の NumPy 配列に変換するヘルパー関数
        """
        points_list = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = PointNetROS2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info("ノードを終了します。")
    node.destroy_node()
    rclpy.shutdown()
    # Open3D ウィンドウの終了
    node.vis.destroy_window()

if __name__ == '__main__':
    main()
