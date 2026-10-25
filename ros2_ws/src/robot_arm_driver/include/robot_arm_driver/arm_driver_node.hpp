#pragma once

#include "robot_arm_driver/servo_bus.hpp"
#include "robot_arm_interfaces/action/move_joints.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_arm_driver
{
class ArmDriverNode final : public rclcpp::Node
{
  public:
    ArmDriverNode();

  private:
    using MoveJoints = robot_arm_interfaces::action::MoveJoints;
    using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory =
      rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveJoints::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    rclcpp_action::GoalResponse handle_trajectory_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_trajectory_cancel(
      const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    void handle_trajectory_accepted(
      const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    void execute_trajectory(
      const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    bool validate_trajectory(
      const FollowJointTrajectory::Goal &goal, std::string &reason) const;
    bool validate_goal(const MoveJoints::Goal &goal, std::string &reason) const;
    std::vector<uint16_t> goal_to_ticks(const MoveJoints::Goal &goal) const;
    sensor_msgs::msg::JointState state_from_ticks(const std::vector<uint16_t> &ticks) const;
    void publish_state(const std::vector<uint16_t> &ticks);
    void poll_feedback();
    void emergency_stop();

    std::unique_ptr<ServoBus> bus_;
    std::vector<std::string> joint_names_;
    std::vector<int64_t> servo_ids_;
    std::vector<double> zero_ticks_;
    std::vector<double> ticks_per_radian_;
    std::vector<double> directions_;
    std::vector<int64_t> min_ticks_;
    std::vector<int64_t> max_ticks_;
    std::unordered_map<std::string, size_t> joint_index_;
    std::vector<uint16_t> last_ticks_;
    bool read_feedback_on_completion_{false};
    bool allow_motion_{false};
    std::atomic_bool moving_{false};
    int feedback_timeout_ms_{50};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp_action::Server<MoveJoints>::SharedPtr action_server_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_server_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
};
}  // namespace robot_arm_driver
