#include "rclcpp/rclcpp.hpp"
#include "livox_to_scan.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToScan>());
    rclcpp::shutdown();
    return 0;
}
