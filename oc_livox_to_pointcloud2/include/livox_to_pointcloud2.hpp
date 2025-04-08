#ifndef LIVOX_TO_POINTCLOUD2_HPP_
#define LIVOX_TO_POINTCLOUD2_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
  LivoxToPointCloud2();

private:
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};

#endif  // LIVOX_TO_POINTCLOUD2_HPP_
