#pragma once

#include "robot_arm_interfaces/msg/ultrasonic_array.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <chrono>
#include <mutex>

namespace robot_chassis
{
class UltrasonicNode final : public rclcpp::Node
{
public:
  UltrasonicNode();

private:
  struct Reading
  {
    double value_m{0.0};
    bool valid{false};
    std::chrono::steady_clock::time_point stamp{};
  };

  void update_front(const sensor_msgs::msg::Range::SharedPtr message);
  void update_rear(const sensor_msgs::msg::Range::SharedPtr message);
  void update_left(const sensor_msgs::msg::Range::SharedPtr message);
  void update_right(const sensor_msgs::msg::Range::SharedPtr message);
  void publish();
  static Reading convert(const sensor_msgs::msg::Range &message);

  double stale_timeout_s_{0.2};
  std::mutex mutex_;
  Reading front_;
  Reading rear_;
  Reading left_;
  Reading right_;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rear_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sub_;
  rclcpp::Publisher<robot_arm_interfaces::msg::UltrasonicArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robot_chassis
