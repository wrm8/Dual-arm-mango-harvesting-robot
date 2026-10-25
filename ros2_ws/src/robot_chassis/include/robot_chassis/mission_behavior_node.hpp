#pragma once

#include "robot_arm_interfaces/msg/mango_target.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace robot_chassis
{

class MissionBehaviorNode final : public rclcpp::Node
{
public:
  struct RouteParameters
  {
    int row_count{7};
    int row_index{0};
    int8_t direction{1};
    int8_t shift_direction{1};
    double row_speed_mps{0.20};
    double shift_distance_m{0.30};
    double shift_speed_mps{0.10};
    double side_target_m{0.05};
    double front_slow_distance_m{0.80};
    double front_stop_distance_m{0.35};
    double shift_safety_distance_m{0.12};
  };

  MissionBehaviorNode();

  bool latch_mango_if_ready();
  robot_arm_interfaces::msg::MangoTarget latched_mango() const;
  void mark_mango_handled();
  bool mango_handling() const;
  bool mission_fault() const;
  void set_mission_fault(const std::string &message);

  RouteParameters route_parameters() const;
  bool route_complete() const;
  bool next_action_is_shift() const;
  void follow_completed();
  void shift_completed();
  const std::string &tree_xml() const;

private:
  void mango_callback(const robot_arm_interfaces::msg::MangoTarget::SharedPtr message);
  void tick_tree();

  mutable std::mutex mutex_;
  RouteParameters route_;
  bool next_action_shift_{false};
  bool route_complete_{false};
  bool fault_{false};
  std::string fault_message_;

  double mango_min_confidence_{0.75};
  int mango_stable_frames_{3};
  double mango_timeout_s_{0.5};
  double mango_rearm_delay_s_{1.0};
  std::chrono::steady_clock::time_point latest_mango_time_{};
  std::chrono::steady_clock::time_point last_pick_time_{};
  int stable_count_{0};
  uint32_t last_track_id_{0};
  bool target_valid_{false};
  bool mango_handling_{false};
  bool waiting_for_target_clear_{false};
  robot_arm_interfaces::msg::MangoTarget latest_mango_;
  robot_arm_interfaces::msg::MangoTarget latched_mango_;

  double mango_approach_height_m_{0.10};
  double mango_grasp_offset_x_m_{0.0};
  double mango_grasp_offset_y_m_{0.0};
  double mango_grasp_offset_z_m_{0.0};
  double mango_tool_tilt_rad_{1.5};
  double mango_joint5_rad_{0.0};
  double pick_duration_s_{2.0};
  bool pick_return_home_{true};

  std::string tree_xml_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::Subscription<robot_arm_interfaces::msg::MangoTarget>::SharedPtr mango_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace robot_chassis
