#pragma once

#include "robot_arm_driver/coordinate_bus.hpp"
#include "robot_arm_interfaces/action/move_cartesian.hpp"
#include "robot_arm_interfaces/action/pick_point.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace robot_arm_driver
{
class CoordinateArmNode final : public rclcpp::Node
{
  public:
    CoordinateArmNode();

  private:
    using MoveCartesian = robot_arm_interfaces::action::MoveCartesian;
    using PickPoint = robot_arm_interfaces::action::PickPoint;
    using MoveHandle = rclcpp_action::ServerGoalHandle<MoveCartesian>;
    using PickHandle = rclcpp_action::ServerGoalHandle<PickPoint>;

    bool valid_point(
      const geometry_msgs::msg::PointStamped &point, std::string *reason) const;
    uint16_t duration_ms(const builtin_interfaces::msg::Duration &duration) const;
    void send_pose(
      const geometry_msgs::msg::PointStamped &point, double pitch_deg, double roll_deg,
      double claw, uint16_t duration);
    void send_gripper(uint16_t position);

    template<typename HandleT>
    bool wait_or_cancel(const std::shared_ptr<HandleT> &handle, uint16_t duration);

    rclcpp_action::GoalResponse claim_goal(std::string &reason);
    rclcpp_action::GoalResponse accept_move(const MoveCartesian::Goal &goal);
    rclcpp_action::GoalResponse accept_pick(const PickPoint::Goal &goal);
    void execute_move(const std::shared_ptr<MoveHandle> &handle);
    void execute_pick(const std::shared_ptr<PickHandle> &handle);

    std::unique_ptr<CoordinateBus> bus_;
    std::string base_frame_;
    bool require_base_frame_{};
    double x_min_{};
    double x_max_{};
    double y_min_{};
    double y_max_{};
    double z_min_{};
    double z_max_{};
    double pick_pitch_{};
    double pick_roll_{};
    uint16_t default_motion_ms_{};
    uint16_t gripper_open_{};
    uint16_t gripper_close_{};
    uint16_t gripper_speed_{};
    uint8_t gripper_id_{};
    geometry_msgs::msg::PointStamped home_;

    std::atomic_bool busy_{false};
    rclcpp_action::Server<MoveCartesian>::SharedPtr move_server_;
    rclcpp_action::Server<PickPoint>::SharedPtr pick_server_;
};
}  // namespace robot_arm_driver
