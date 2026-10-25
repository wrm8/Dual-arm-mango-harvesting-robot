#pragma once

#include "robot_arm_driver/coordinate_bus.hpp"
#include "robot_arm_interfaces/action/move_joints.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_arm_driver
{
class HxArmDriverNode final : public rclcpp::Node
{
  public:
    HxArmDriverNode();

  private:
    using MoveJoints = robot_arm_interfaces::action::MoveJoints;
    using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoints::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveJoints> &);
    void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> &goal_handle);
    void execute(const std::shared_ptr<GoalHandleMoveJoints> &goal_handle);
    bool validate_goal(const MoveJoints::Goal &goal, std::string &reason) const;
    std::vector<int16_t> goal_to_ticks(const MoveJoints::Goal &goal) const;
    sensor_msgs::msg::JointState state_from_ticks(const std::vector<int16_t> &ticks) const;
    void publish_state(const std::vector<int16_t> &ticks);
    std::optional<int16_t> read_position(uint8_t id);
    void write_checked(uint8_t id, const std::vector<uint8_t> &frame, const std::string &operation);
    void set_position_mode(uint8_t id);
    void set_torque(uint8_t id, bool enabled);
    void emergency_stop();

    std::unique_ptr<CoordinateBus> bus_;
    std::vector<std::string> joint_names_;
    std::vector<int64_t> servo_ids_;
    std::vector<double> zero_ticks_;
    std::vector<double> ticks_per_radian_;
    std::vector<double> directions_;
    std::vector<int64_t> min_ticks_;
    std::vector<int64_t> max_ticks_;
    std::unordered_map<std::string, size_t> joint_index_;
    std::vector<int16_t> last_ticks_;
    std::vector<bool> online_;
    uint16_t goal_speed_{100};
    uint8_t goal_acceleration_{0};
    int position_tolerance_ticks_{20};
    std::atomic_bool moving_{false};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp_action::Server<MoveJoints>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
};
}  // namespace robot_arm_driver
