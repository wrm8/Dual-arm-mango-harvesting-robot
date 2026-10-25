#include "robot_arm_driver/arm_driver_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_arm_driver::ArmDriverNode>());
  rclcpp::shutdown();
  return 0;
}
