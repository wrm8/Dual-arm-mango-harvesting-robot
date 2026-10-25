#pragma once

#include "robot_arm_interfaces/action/follow_row.hpp"
#include "robot_arm_interfaces/action/shift_lane.hpp"
#include "robot_arm_interfaces/action/stop_base.hpp"
#include "robot_arm_interfaces/msg/ultrasonic_array.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

namespace robot_chassis
{
class RowNavigationNode final : public rclcpp::Node
{
public:
  RowNavigationNode();

private:
  using FollowRow = robot_arm_interfaces::action::FollowRow;
  using ShiftLane = robot_arm_interfaces::action::ShiftLane;
  using StopBase = robot_arm_interfaces::action::StopBase;
  using FollowHandle = rclcpp_action::ServerGoalHandle<FollowRow>;
  using ShiftHandle = rclcpp_action::ServerGoalHandle<ShiftLane>;
  using StopHandle = rclcpp_action::ServerGoalHandle<StopBase>;

  struct Sensors
  {
    robot_arm_interfaces::msg::UltrasonicArray ultrasonic;
    nav_msgs::msg::Odometry odom;
    bool have_ultrasonic{false};
    bool have_odom{false};
    std::chrono::steady_clock::time_point ultrasonic_time{};
    std::chrono::steady_clock::time_point odom_time{};
  };

  enum class RoutePhase : uint8_t
  {
    Idle,
    FollowRow,
    Braking,
    Stopped,
    ShiftLane,
    ShiftBraking,
    Paused,
    Fault
  };

  rclcpp_action::GoalResponse follow_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowRow::Goal> goal);
  rclcpp_action::CancelResponse follow_cancel(const std::shared_ptr<FollowHandle> &);
  void follow_accepted(const std::shared_ptr<FollowHandle> &handle);
  void execute_follow(const std::shared_ptr<FollowHandle> &handle);

  rclcpp_action::GoalResponse shift_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ShiftLane::Goal> goal);
  rclcpp_action::CancelResponse shift_cancel(const std::shared_ptr<ShiftHandle> &);
  void shift_accepted(const std::shared_ptr<ShiftHandle> &handle);
  void execute_shift(const std::shared_ptr<ShiftHandle> &handle);

  rclcpp_action::GoalResponse stop_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const StopBase::Goal> goal);
  rclcpp_action::CancelResponse stop_cancel(const std::shared_ptr<StopHandle> &);
  void stop_accepted(const std::shared_ptr<StopHandle> &handle);
  void execute_stop(const std::shared_ptr<StopHandle> &handle);

  void ultrasonic_callback(const robot_arm_interfaces::msg::UltrasonicArray::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  Sensors sensors() const;
  bool fresh(const Sensors &snapshot, bool need_odom) const;
  void publish_command(const geometry_msgs::msg::Twist &command);
  void publish_zero();
  void set_phase(RoutePhase phase);
  static double clamp(double value, double lower, double upper);
  static double speed_scale(double distance, double slow_distance, double stop_distance);
  static double normalize_angle(double angle);
  static double planar_lateral_displacement(
    const nav_msgs::msg::Odometry &start, const nav_msgs::msg::Odometry &current);

  double loop_hz_{20.0};
  double sensor_timeout_s_{0.3};
  double side_kp_{1.2};
  double yaw_kp_{1.0};
  double max_lateral_speed_mps_{0.15};
  double max_yaw_rate_rad_s_{0.4};
  double stop_speed_epsilon_mps_{0.03};
  double stop_angular_epsilon_rad_s_{0.08};
  double stop_settle_s_{0.2};
  double default_front_slow_m_{0.8};
  double default_front_stop_m_{0.35};
  double default_safety_distance_m_{0.12};

  mutable std::mutex sensor_mutex_;
  Sensors sensors_;
  mutable std::mutex phase_mutex_;
  RoutePhase phase_{RoutePhase::Idle};
  int32_t active_row_id_{-1};
  int8_t active_direction_{1};
  std::atomic_bool active_{false};

  rclcpp::Subscription<robot_arm_interfaces::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp_action::Server<FollowRow>::SharedPtr follow_server_;
  rclcpp_action::Server<ShiftLane>::SharedPtr shift_server_;
  rclcpp_action::Server<StopBase>::SharedPtr stop_server_;
};
}  // namespace robot_chassis
