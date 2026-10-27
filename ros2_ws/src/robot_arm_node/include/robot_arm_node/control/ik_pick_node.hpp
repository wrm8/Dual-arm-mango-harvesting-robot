#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_arm_node/motion/urdf_kinematics.hpp"
#include "robot_arm_interfaces/action/pick_point.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


#include <mutex>
#include <string>
#include <vector>
#include <memory>//make_unique

namespace robot_arm_node::control
{

class IkPickNode final : public rclcpp::Node
{
public:
    explicit IkPickNode(
        const std::string & node_name = "ik_pick_node",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    using PickPoint = robot_arm_interfaces::action::PickPoint;
    using PickGoalHandle = rclcpp_action::ServerGoalHandle<PickPoint>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using FollowGoalHandle =
        rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickPoint::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<PickGoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<PickGoalHandle> goal_handle);


    void robot_description_callback(const std_msgs::msg::String::SharedPtr message);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
    void execute_pick(const std::shared_ptr<PickGoalHandle> goal_handle);
    void send_trajectory(const std::vector<std::vector<double>> &target_points);

    std::unique_ptr<motion::UrdfKinematics> kinematics_;

    rclcpp_action::Server<PickPoint>::SharedPtr pick_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    std::vector<double> current_joints_;
    bool have_joint_state_{false};
    std::mutex state_mutex_; 

    std::string base_link_;
    std::string tip_link_;
    double pitch_rad_{};
    double roll_rad_{};

};

}
