#include "livox_to_pointcloud2.hpp"
#include <cmath>

using namespace std::chrono_literals;

LivoxToPointCloud2::LivoxToPointCloud2() : Node("livox_to_pointcloud2")
{
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("converted_pointcloud2", 10);
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "livox_pointcloud", 10, std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));
}

void LivoxToPointCloud2::callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header;
    output.header.frame_id = "mid360";  // フレームIDを設定
    output.fields.resize(6);

    output.fields[0].name = "x";
    output.fields[0].offset = 0;
    output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[0].count = 1;

    output.fields[1].name = "y";
    output.fields[1].offset = 4;
    output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[1].count = 1;

    output.fields[2].name = "z";
    output.fields[2].offset = 8;
    output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[2].count = 1;

    output.fields[3].name = "intensity";
    output.fields[3].offset = 12;
    output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[3].count = 1;

    output.fields[4].name = "tag";
    output.fields[4].offset = 16;
    output.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    output.fields[4].count = 1;

    output.fields[5].name = "line";
    output.fields[5].offset = 17;
    output.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
    output.fields[5].count = 1;

    output.point_step = 18;
    
    // 一時的なバッファにポイントを格納
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(output.point_step * msg->point_num);  // 最大サイズで予約
    
    uint32_t filtered_point_count = 0;
    
    for (const auto& point : msg->points)
    {
        // 前方180度のデータのみをフィルタリング（X軸が正で、Y軸の絶対値がX軸以下の点）
        if (point.x > 0.0f && std::abs(point.y) <= point.x)  // 前方約180度のデータ
        {
            uint8_t point_data[18];
            
            *(reinterpret_cast<float*>(point_data + 0)) = point.x;
            *(reinterpret_cast<float*>(point_data + 4)) = point.y;
            *(reinterpret_cast<float*>(point_data + 8)) = point.z;
            *(reinterpret_cast<float*>(point_data + 12)) = static_cast<float>(point.reflectivity);
            *(point_data + 16) = point.tag;
            *(point_data + 17) = point.line;
            
            filtered_data.insert(filtered_data.end(), point_data, point_data + output.point_step);
            filtered_point_count++;
        }
    }
    
    // フィルタリングされたデータを出力に設定
    output.data.resize(output.point_step * filtered_point_count);
    std::copy(filtered_data.begin(), filtered_data.end(), output.data.begin());
    
    output.width = filtered_point_count;
    output.height = 1;
    output.row_step = output.point_step * filtered_point_count;
    output.is_bigendian = false;
    output.is_dense = true;

    publisher_->publish(output);
}
