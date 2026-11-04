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
    int step_count{16};
    int step_index{0};
    int row_index{0};
    int8_t direction{1};
    int8_t shift_direction{-1};
    double row_speed_mps{0.20};
    double shift_distance_m{0.30};
    double shift_speed_mps{0.10};
    bool shift_vision_stop_enabled{false};
    double side_target_m{0.05};
    double front_slow_distance_m{0.30};
    double front_stop_distance_m{0.05};
    uint8_t side_sensor{0};
    double shift_safety_distance_m{0.05};
    double reverse_speed_mps{0.50};
    //double reverse_distance_kp{2.0};
    double reverse_distance_m{0.70};
    bool reverse_by_encoder{true};
    bool shift_by_side_wall{true};
    bool hold_front_distance{true};
    bool use_side_centering{true};
    double shift_front_target_m{0.05};
    uint8_t shift_wall_sensor{1};
    double shift_wall_target_m{0.05};
    double shift_wall_enable_tolerance_m{0.10};
    bool shift_disable_wall_control_on_side_distance{false};
    double shift_wall_control_cancel_side_distance_m{0.25};
    double shift_wall_control_cancel_side_tolerance_m{0.02};
    bool reverse_stop_on_zero_velocity{false};
    bool mango_interruptible{true};
  };

  MissionBehaviorNode();

  bool latch_mango_if_ready();
  void refresh_latched_mango();
  robot_arm_interfaces::msg::MangoTarget latched_mango() const;
  void mark_mango_handled();
  bool mango_handling() const;
  bool mission_fault() const;
  void set_mission_fault(const std::string &message);

  RouteParameters route_parameters() const;
  bool route_complete() const;
  bool next_action_is_shift() const;
  bool current_step_mango_interruptible() const;
  void follow_completed();
  void shift_completed();
  const std::string &tree_xml() const;

private:
  void mango_callback(const robot_arm_interfaces::msg::MangoTarget::SharedPtr message);
  void tick_tree();
  bool mango_geometry_is_usable(const robot_arm_interfaces::msg::MangoTarget &message) const;

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
  double mango_min_distance_m_{0.30};
  double mango_max_distance_m_{1.50};
  double mango_center_tolerance_m_{0.10};
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
