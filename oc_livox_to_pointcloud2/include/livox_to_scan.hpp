#ifndef LIVOX_TO_SCAN_HPP_
#define LIVOX_TO_SCAN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

class LivoxToScan : public rclcpp::Node
{
public:
  LivoxToScan();

private:
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  
  // 変換パラメータ
  float min_z_;
  float max_z_;
  float angle_min_;
  float angle_max_;
  int num_samples_;
  float angle_increment_;
};

#endif  // LIVOX_TO_SCAN_HPP_
