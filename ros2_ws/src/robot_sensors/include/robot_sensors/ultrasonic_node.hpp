#pragma once

#include "robot_arm_interfaces/msg/ultrasonic_array.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace robot_sensors
{
class UltrasonicNode final : public rclcpp::Node
{
public:
  UltrasonicNode();
  ~UltrasonicNode() override;

private:
  struct Reading
  {
    double value_m{0.0};
    bool valid{false};
    std::chrono::steady_clock::time_point stamp{};
  };

  struct SerialChannel
  {
    std::string device;
    std::string frame_id;
    int fd{-1};
    std::string receive_buffer;
    Reading reading;
    std::chrono::steady_clock::time_point last_trigger{};
  };

  void update_front(const sensor_msgs::msg::Range::ConstSharedPtr message);
  void update_rear(const sensor_msgs::msg::Range::ConstSharedPtr message);
  void update_left(const sensor_msgs::msg::Range::ConstSharedPtr message);
  void update_right(const sensor_msgs::msg::Range::ConstSharedPtr message);
  void open_serial_channels();
  void close_serial_channels();
  void poll_serial();
  void parse_serial_channel(std::size_t index);
  void publish_centimeters(
    const Reading &reading,
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &publisher);
  void publish_range(const Reading &reading, const std::string &frame_id,
                     const rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr &publisher,
                     const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &cm_publisher);
  void publish();
  static Reading convert(const sensor_msgs::msg::Range &message);

  bool serial_enabled_{false};
  int serial_baud_rate_{115200};
  int serial_read_buffer_size_{128};
  double serial_min_range_m_{0.03};
  double serial_max_range_m_{5.0};
  double field_of_view_rad_{1.22};
  double controlled_trigger_period_s_{0.20};
  bool distance_data_is_microseconds_{false};
  std::string uart_output_mode_{"auto"};
  double stale_timeout_s_{0.6};
  std::mutex mutex_;
  Reading front_;
  Reading rear_;
  Reading left_;
  Reading right_;

  std::array<SerialChannel, 4> serial_channels_;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rear_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rear_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr left_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr right_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_cm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_cm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_cm_pub_;
  rclcpp::Publisher<robot_arm_interfaces::msg::UltrasonicArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robot_sensors
