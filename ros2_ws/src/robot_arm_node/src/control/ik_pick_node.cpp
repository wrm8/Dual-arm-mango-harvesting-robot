#include "robot_arm_node/control/ik_pick_node.hpp"

#include <algorithm> // 为了 std::find
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>

namespace robot_arm_node::control
{

IkPickNode::IkPickNode(
    const std::string & node_name, 
    const rclcpp::NodeOptions & options) : Node(node_name, options)
{
    // 1. 读取参数
    base_link_ = declare_parameter("base_link", "base_link");
    tip_link_ = declare_parameter("tip_link", "tcp_link");
    // 默认 pitch 给 1.5708（约 90度），表示水平向前抓
    pitch_rad_ = declare_parameter("pitch", 1.5708);
    roll_rad_ = declare_parameter("roll", 0.0);

    description_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).reliable().transient_local(),
        //std::bind(&IkPickNode::robot_description_callback(), this, std::placeholders::_1)
        [this](const std_msgs::msg::String::SharedPtr msg){
            this->robot_description_callback(msg);
        });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(), // 传感器数据用这个 QoS，保证低延迟
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { 
            this->joint_state_callback(msg); 
        });
    pick_server_ = rclcpp_action::create_server<PickPoint>(
        this, "pick_point",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickPoint::Goal> goal){
            return this->handle_goal(uuid, goal);},
        [this](const std::shared_ptr<PickGoalHandle> goal_handle){
            return this->handle_cancel(goal_handle);},
        [this](const std::shared_ptr<PickGoalHandle> goal_handle){
            this->handle_accepted(goal_handle);});
     // 创建发轨迹的客户端
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/arm_controller/follow_joint_trajectory");
    
    RCLCPP_INFO(get_logger(), "节点已启动，等待 URDF 和 关节状态数据...");

}

void IkPickNode::robot_description_callback(const std_msgs::msg::String::SharedPtr message)
{
    // 如果已经初始化过了，直接返回，防止重复创建
    if (kinematics_) {
        return;
    }

    try {
        // 实例化你写的运动学库
        kinematics_ = std::make_unique<robot_arm_node::motion::UrdfKinematics>(
            message->data, base_link_, tip_link_);
        
        RCLCPP_INFO(get_logger(), "✅ URDF 解析成功！运动学引擎已就绪。");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "❌ URDF 解析失败: %s", e.what());
    }
}

void IkPickNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
    // 如果运动学引擎还没建好，无法知道关节名字，直接跳过
    if (!kinematics_) {
        return;
    }

    // 拿到你的机械臂所要求的关节名字顺序（比如 joint1, joint2...）
    const auto &required_names = kinematics_->joint_names();
    std::vector<double> joints(required_names.size());

    // 按照名字去消息里找对应的角度
    for (std::size_t i = 0; i < required_names.size(); ++i) {
        auto it = std::find(message->name.begin(), message->name.end(), required_names[i]);
        
        if (it == message->name.end()) {
            return; // 如果缺了某个关节，这一帧数据丢弃
        }
        
        auto index = std::distance(message->name.begin(), it);
        joints[i] = message->position[index];
    }

    // 加锁，更新数据
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = joints;
    have_joint_state_ = true;
}

rclcpp_action::GoalResponse IkPickNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickPoint::Goal> goal)
{
    (void)uuid;
    const auto &point = goal->target.point;
    if (goal->target.header.frame_id != base_link_) {
        RCLCPP_WARN(
            get_logger(), "拒绝目标：frame_id 必须是 %s，收到 %s",
            base_link_.c_str(), goal->target.header.frame_id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(goal->approach_height_m) ||
        goal->approach_height_m < 0.0 ||
        !std::isfinite(goal->grasp_offset_x_m) ||
        !std::isfinite(goal->grasp_offset_y_m) ||
        !std::isfinite(goal->grasp_offset_z_m) ||
        !std::isfinite(goal->target_tool_tilt) ||
        !std::isfinite(goal->target_joint5)) {
        RCLCPP_WARN(get_logger(), "拒绝目标：目标坐标/偏移/接近高度/姿态角无效");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
        get_logger(), "收到目标请求 (%s): X=%.3f, Y=%.3f, Z=%.3f",
        goal->target.header.frame_id.c_str(), point.x, point.y, point.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IkPickNode::handle_cancel(
    const std::shared_ptr<PickGoalHandle> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void IkPickNode::handle_accepted(const std::shared_ptr<PickGoalHandle> goal_handle)
{
    RCLCPP_INFO(get_logger(), "目标已接收，准备处理...");
    // 起新线程执行，防止卡死主节点
    std::thread{
        &IkPickNode::execute_pick, this, goal_handle
    }.detach();
}

void IkPickNode::execute_pick(const std::shared_ptr<PickGoalHandle> goal_handle)
{
    if (!kinematics_ || !have_joint_state_) {
        RCLCPP_ERROR(get_logger(), "缺少 URDF 或 关节状态，无法执行！");
        goal_handle->abort(std::make_shared<PickPoint::Result>());
        return;
    }

    auto goal = goal_handle->get_goal();
    
    // 获取当前关节角作为 seed
    std::vector<double> seed;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        seed = current_joints_;
    }

    // 你的 Action 里的目标点
    const auto &point = goal->target.point;
    double target_x = point.x + goal->grasp_offset_x_m;
    double target_y = point.y + goal->grasp_offset_y_m;
    double target_z = point.z + goal->grasp_offset_z_m;
    double approach_h = goal->approach_height_m;
    double target_pitch = goal->target_tool_tilt; 
    double target_roll = goal->target_joint5; 
    // double target_roll = 0.0; 


    RCLCPP_INFO(
        get_logger(), "开始 IK 解算，目标点: (%.3f, %.3f, %.3f)，接近高度: %.3f, Pitch: %.3f, roll: %.3f",
        target_x, target_y, target_z, approach_h, target_pitch, target_roll);

    const char *ik_stage = "pregrasp";
    try {
        // Solve approach, grasp, and lift poses with continuous seeds.
        ik_stage = "pregrasp";
        auto q_pregrasp = kinematics_->inverse_xyz_tilt_joint5(
            seed, target_x, target_y, target_z + approach_h, target_pitch, target_roll);
        ik_stage = "grasp";
        auto q_grasp = kinematics_->inverse_xyz_tilt_joint5(
            q_pregrasp, target_x, target_y, target_z, target_pitch, target_roll);

        const auto log_solution = [this](const char * stage, const std::vector<double> & joints) {
            const auto frame = kinematics_->forward(joints);
            double actual_roll = 0.0;
            double actual_pitch = 0.0;
            double actual_yaw = 0.0;
            frame.M.GetRPY(actual_roll, actual_pitch, actual_yaw);
            RCLCPP_INFO(
                get_logger(),
                "%s q=[%.4f, %.4f, %.4f, %.4f, %.4f], FK xyz=(%.4f, %.4f, %.4f), RPY=(roll %.4f, pitch %.4f, yaw %.4f)",
                stage, joints[0], joints[1], joints[2], joints[3], joints[4],
                frame.p.x(), frame.p.y(), frame.p.z(), actual_roll, actual_pitch, actual_yaw);

            const auto & lower = kinematics_->lower_limits();
            const auto & upper = kinematics_->upper_limits();
            for (std::size_t i = 0; i < joints.size(); ++i) {
                if (std::abs(joints[i] - lower[i]) < 1e-3 ||
                    std::abs(joints[i] - upper[i]) < 1e-3) {
                    RCLCPP_WARN(
                        get_logger(), "IK %s: %s 接近关节限位 [%.3f, %.3f]，当前 %.3f",
                        stage, kinematics_->joint_names()[i].c_str(), lower[i], upper[i], joints[i]);
                }
            }
        };

        log_solution("grasp", q_grasp);
        
        // Lift back above the target after reaching the grasp pose.
        ik_stage = "lift";
        auto q_lift = kinematics_->inverse_xyz_tilt_joint5(
            q_grasp, target_x, target_y, target_z + approach_h, target_pitch, target_roll);

        log_solution("lift", q_lift);

        // Execute approach, grasp, and lift; each point takes two seconds.
        ik_stage = "trajectory execution";
        std::vector<std::vector<double>> trajectory_points{q_pregrasp, q_grasp, q_lift};
        if (goal->return_home) {
            trajectory_points.emplace_back(kinematics_->joint_count(), 0.0);
        }
        send_trajectory(trajectory_points);

        // 告诉客户端成功
        auto result = std::make_shared<PickPoint::Result>();
        result->success = true;
        result->message = "trajectory execution succeeded";
        result->final_position = goal->target;
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(), "轨迹执行完成");

    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "阶段 %s 失败: %s", ik_stage, e.what());
        auto result = std::make_shared<PickPoint::Result>();
        result->success = false;
        result->message = std::string(ik_stage) + ": " + e.what();
        result->final_position = goal->target;
        goal_handle->abort(result);
    }
}

void IkPickNode::send_trajectory(const std::vector<std::vector<double>> &target_points)
{
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(2))) {
        throw std::runtime_error("底层控制器 /arm_controller/follow_joint_trajectory 未就绪！");
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = kinematics_->joint_names();

    for (std::size_t i = 0; i < target_points.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_points[i];
        point.time_from_start = rclcpp::Duration::from_seconds(2.0 * (i + 1));
        goal.trajectory.points.push_back(point);
    }

    auto goal_future = trajectory_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        throw std::runtime_error("等待轨迹控制器接受 goal 超时");
    }

    const auto controller_goal_handle = goal_future.get();
    if (!controller_goal_handle) {
        throw std::runtime_error("轨迹控制器拒绝了 goal");
    }

    auto result_future = trajectory_client_->async_get_result(controller_goal_handle);
    if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
        trajectory_client_->async_cancel_goal(controller_goal_handle);
        throw std::runtime_error("等待轨迹执行结果超时，已请求取消轨迹");
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        throw std::runtime_error("轨迹 Action 未以 SUCCEEDED 状态结束");
    }
    if (!wrapped_result.result ||
        wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
        const auto error_code = wrapped_result.result ?
            wrapped_result.result->error_code : FollowJointTrajectory::Result::INVALID_GOAL;
        throw std::runtime_error(
            "轨迹控制器执行失败，FollowJointTrajectory error_code=" +
            std::to_string(error_code));
    }
}

}

int main(int argc,char **argv)
{
    rclcpp::init(argc, argv);
    // 在这里，你可以随时改名字！比如改成 "my_super_arm"
    std::string my_node_name = "ik_pick_node";
    auto node = std::make_shared<robot_arm_node::control::IkPickNode>(my_node_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
