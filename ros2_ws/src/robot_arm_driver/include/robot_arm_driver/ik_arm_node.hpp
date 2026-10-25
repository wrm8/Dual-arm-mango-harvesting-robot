#pragma once

#include "robot_arm_driver/planar_arm_kinematics.hpp"
#include "robot_arm_driver/servo_bus.hpp"
#include "robot_arm_interfaces/action/move_cartesian.hpp"
#include "robot_arm_interfaces/action/pick_point.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robot_arm_driver
{
class IkArmNode final : public rclcpp::Node
{
  public:
    IkArmNode();

  private:
    using MoveCartesian = robot_arm_interfaces::action::MoveCartesian;
    using PickPoint = robot_arm_interfaces::action::PickPoint;
    using MoveHandle = rclcpp_action::ServerGoalHandle<MoveCartesian>;
    using PickHandle = rclcpp_action::ServerGoalHandle<PickPoint>;

    static constexpr double kPi = 3.14159265358979323846;

    bool valid_point(const geometry_msgs::msg::PointStamped &point, std::string &reason) const;
    uint16_t duration_ms(const builtin_interfaces::msg::Duration &duration) const;
    uint16_t tick(std::size_t index, double radians) const;
    void publish(const std::array<double, 5> &joints, uint16_t gripper);
    void move(
      const geometry_msgs::msg::PointStamped &point, double pitch_deg, double roll_deg,
      uint16_t gripper, uint16_t duration);

    template<typename HandleT>
    bool wait(const std::shared_ptr<HandleT> &handle, uint16_t duration) const;

    rclcpp_action::GoalResponse claim();
    rclcpp_action::GoalResponse accept_move(const MoveCartesian::Goal &goal);
    rclcpp_action::GoalResponse accept_pick(const PickPoint::Goal &goal);
    void execute_move(const std::shared_ptr<MoveHandle> &handle);
    void execute_pick(const std::shared_ptr<PickHandle> &handle);

    std::unique_ptr<ServoBus> bus_;
    std::unique_ptr<PlanarArmKinematics> kinematics_;
    std::vector<std::string> joint_names_;
    std::vector<int64_t> servo_ids_;
    std::vector<int64_t> min_ticks_;
    std::vector<int64_t> max_ticks_;
    std::vector<double> zero_ticks_;
    std::vector<double> ticks_per_radian_;
    std::vector<double> directions_;

    std::string base_frame_;
    bool require_base_frame_{};
    bool elbow_up_{};
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
    geometry_msgs::msg::PointStamped home_;

    std::atomic_bool busy_{false};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp_action::Server<MoveCartesian>::SharedPtr move_server_;
    rclcpp_action::Server<PickPoint>::SharedPtr pick_server_;
};
}  // namespace robot_arm_driver
