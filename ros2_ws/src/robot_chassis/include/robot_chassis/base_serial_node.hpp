#pragma once

#include "robot_arm_interfaces/msg/base_state.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace robot_chassis
{
class BaseSerialNode final : public rclcpp::Node
{
public:
  BaseSerialNode();
  ~BaseSerialNode() override;

private:
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void watchdog_callback();
  void read_loop();
  void parse_line(const std::string &line);
  void publish_state();
  void open_serial();
  void close_serial();
  void write_command(const geometry_msgs::msg::Twist &command);
  void send_zero_once();

  std::string serial_device_;
  int baud_rate_{115200};
  bool dry_run_{true};
  bool allow_motion_{false};
  double command_timeout_s_{0.2};
  double publish_rate_hz_{20.0};

  int fd_{-1};
  std::atomic_bool running_{false};
  std::thread read_thread_;
  std::mutex io_mutex_;
  std::mutex state_mutex_;
  std::string receive_buffer_;

  geometry_msgs::msg::Twist last_command_;
  std::chrono::steady_clock::time_point last_command_time_;
  bool timeout_active_{true};
  double vx_mps_{0.0};
  double vy_mps_{0.0};
  double wz_rad_s_{0.0};
  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rad_{0.0};
  double gyro_z_rad_s_{0.0};
  bool have_odom_{false};
  bool have_imu_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<robot_arm_interfaces::msg::BaseState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
}  // namespace robot_chassis
