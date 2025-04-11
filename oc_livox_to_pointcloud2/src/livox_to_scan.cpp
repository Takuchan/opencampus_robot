#include "livox_to_scan.hpp"
#include <cmath>
#include <vector>
#include <limits>

LivoxToScan::LivoxToScan() : Node("livox_to_scan")
{
    // パラメータの設定
    min_z_ = 0.0f;  // Z軸の最小値 (m)
    max_z_ = 0.2f;     // Z軸の最大値 (m)
    
    // LaserScanのパラメータ設定 - 360度全周をカバー
    angle_min_ = -M_PI / 2;       // -180度
    angle_max_ = M_PI / 2;        // 180度
    num_samples_ = 720;       // 分解能（サンプル数）- 0.5度ごと
    angle_increment_ = (angle_max_ - angle_min_) / static_cast<float>(num_samples_ - 1);
    
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "livox_pointcloud", 10, std::bind(&LivoxToScan::callback, this, std::placeholders::_1));
}

void LivoxToScan::callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    // LaserScanメッセージの初期化
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = msg->header;
    scan_msg.header.frame_id = "mid360";
    scan_msg.angle_min = angle_min_;
    scan_msg.angle_max = angle_max_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;  // 10Hz想定
    scan_msg.range_min = 0.1;  // 最小測定距離 (m)
    scan_msg.range_max = 200.0;  // 最大測定距離 (m)
    
    // レーザスキャンデータの初期化
    scan_msg.ranges.resize(num_samples_, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(num_samples_, 0.0);
    
    // 360度すべてのデータを処理
    for (const auto& point : msg->points)
    {
        // Z軸が指定範囲内のポイントのみ処理
        if (point.z >= min_z_ && point.z <= max_z_)
        {
            float distance = std::sqrt(point.x * point.x + point.y * point.y);
            float angle = std::atan2(point.y, point.x);
            
            // 角度が範囲内かチェック（-π〜π）
            if (angle >= angle_min_ && angle <= angle_max_)
            {
                // インデックスの計算
                int index = static_cast<int>((angle - angle_min_) / angle_increment_);
                
                // インデックスが範囲内かの確認
                if (index >= 0 && index < num_samples_)
                {
                    // 既存のデータより近い場合のみ更新
                    if (distance < scan_msg.ranges[index] || std::isinf(scan_msg.ranges[index]))
                    {
                        scan_msg.ranges[index] = distance;
                        scan_msg.intensities[index] = static_cast<float>(point.reflectivity);
                    }
                }
            }
        }
    }
    
    // LaserScanメッセージを公開
    publisher_->publish(scan_msg);
}
