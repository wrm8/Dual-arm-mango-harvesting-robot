#include "robot_arm_driver/ik_arm_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_arm_driver::IkArmNode>());
  rclcpp::shutdown();
  return 0;
}
