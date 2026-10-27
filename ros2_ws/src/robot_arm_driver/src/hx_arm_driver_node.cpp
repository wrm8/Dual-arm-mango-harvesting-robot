#include "robot_arm_driver/hx_arm_driver_node.hpp"

#include "robot_arm_driver/coordinate_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <thread>
#include <unordered_set>

using namespace std::chrono_literals;

namespace robot_arm_driver
{
HxArmDriverNode::HxArmDriverNode() : Node("robot_arm_driver")
{
  declare_parameter("serial_device", "/dev/ttyUSB0");
  declare_parameter("baud_rate", 115200);
  declare_parameter("joint_names", std::vector<std::string>{
      "joint1", "joint2", "joint3", "joint4", "joint5", "gripper"});
  declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
  declare_parameter("servo_protocols", std::vector<std::string>{"hx", "hx", "hx", "hx", "v3", "v3"});
  declare_parameter("zero_ticks", std::vector<double>{0, 0, 0, 0, 0, 0});
  declare_parameter("ticks_per_radian", std::vector<double>{
      651.8986, 651.8986, 651.8986, 651.8986, 651.8986, 651.8986});
  declare_parameter("directions", std::vector<double>{1, 1, 1, 1, 1, 1});
  declare_parameter("min_ticks", std::vector<int64_t>{
      -30719, -30719, -30719, -30719, -30719, -30719});
  declare_parameter("max_ticks", std::vector<int64_t>{
      30719, 30719, 30719, 30719, 30719, 30719});
  declare_parameter("goal_speed", 100);
  declare_parameter("goal_acceleration", 0);
  declare_parameter("position_tolerance_ticks", 20);
  declare_parameter("feedback_poll_period_ms", 200);
  declare_parameter("feedback_timeout_ms", 50);
  declare_parameter("allow_motion", false);
  declare_parameter("enable_torque_on_start", false);

  joint_names_ = get_parameter("joint_names").as_string_array();
  servo_ids_ = get_parameter("servo_ids").as_integer_array();
  servo_protocols_ = get_parameter("servo_protocols").as_string_array();
  zero_ticks_ = get_parameter("zero_ticks").as_double_array();
  ticks_per_radian_ = get_parameter("ticks_per_radian").as_double_array();
  directions_ = get_parameter("directions").as_double_array();
  min_ticks_ = get_parameter("min_ticks").as_integer_array();
  max_ticks_ = get_parameter("max_ticks").as_integer_array();
  const auto count = joint_names_.size();

  if (count == 0U || servo_ids_.size() != count || servo_protocols_.size() != count || zero_ticks_.size() != count ||
      ticks_per_radian_.size() != count || directions_.size() != count ||
      min_ticks_.size() != count || max_ticks_.size() != count)
  {
    throw std::invalid_argument("all joint calibration arrays must have the same non-zero size");
  }
  const auto configured_speed = get_parameter("goal_speed").as_int();
  const auto configured_acceleration = get_parameter("goal_acceleration").as_int();
  const auto configured_tolerance = get_parameter("position_tolerance_ticks").as_int();
  const auto feedback_poll_period_ms = get_parameter("feedback_poll_period_ms").as_int();
  feedback_timeout_ms_ = static_cast<int>(get_parameter("feedback_timeout_ms").as_int());
  allow_motion_ = get_parameter("allow_motion").as_bool();
  enable_torque_on_start_ = get_parameter("enable_torque_on_start").as_bool();

  if (configured_speed < 0 || configured_speed > 3400 || configured_acceleration < 0 || configured_acceleration > 254 ||
      configured_tolerance < 0 || configured_tolerance > 1000 || feedback_poll_period_ms < 0 ||
      feedback_poll_period_ms > 60000 || feedback_timeout_ms_ < 1 || feedback_timeout_ms_ > 1000)
  {
    throw std::invalid_argument("invalid goal speed, acceleration, or position tolerance");
  }
  goal_speed_ = static_cast<uint16_t>(configured_speed);
  goal_acceleration_ = static_cast<uint8_t>(configured_acceleration);
  position_tolerance_ticks_ = configured_tolerance;
  last_ticks_.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    if (!joint_index_.emplace(joint_names_[i], i).second || servo_ids_[i] < 0 || servo_ids_[i] > 253 ||
        (servo_protocols_[i] != "hx" && servo_protocols_[i] != "v3") ||
        ticks_per_radian_[i] <= 0.0 || std::abs(directions_[i]) != 1.0 || min_ticks_[i] < -30719 ||
        max_ticks_[i] > 30719 || min_ticks_[i] > max_ticks_[i])
    {
      throw std::invalid_argument("invalid HX calibration parameter at index " + std::to_string(i));
    }
    const auto zero = std::lround(zero_ticks_[i]);
    if (zero < min_ticks_[i] || zero > max_ticks_[i])
    {
      throw std::invalid_argument("zero_ticks is outside joint limits at index " + std::to_string(i));
    }
    last_ticks_.push_back(static_cast<int16_t>(zero));
  }

  const auto device = get_parameter("serial_device").as_string();

  bus_ = std::make_unique<CoordinateBus>(device, static_cast<int>(get_parameter("baud_rate").as_int()));
  bus_->open();
  online_.assign(servo_ids_.size(), false);
  for (size_t i = 0; i < servo_ids_.size(); ++i)
  {
    if (const auto position = read_position(i); position)
    {
      last_ticks_[i] = *position;
      online_[i] = true;
      RCLCPP_INFO(
        get_logger(), "HX servo id=%ld joint=%s raw_tick=%d", servo_ids_[i],
        joint_names_[i].c_str(), static_cast<int>(*position));
    }
    else
    {
      RCLCPP_WARN(
        get_logger(), "no valid HX position reply from ID %ld; verify ID, baud rate, and protocol",
        servo_ids_[i]);
    }
  }
  if (enable_torque_on_start_ && allow_motion_)
  {
    for (size_t i = 0; i < servo_ids_.size(); ++i)
    {
      if (!online_[i])
      {
        continue;
      }
      if (servo_protocols_[i] == "hx")
      {
        set_position_mode(static_cast<uint8_t>(servo_ids_[i]));
        set_torque(static_cast<uint8_t>(servo_ids_[i]), true);
      }
    }
  }
  else if (enable_torque_on_start_)
  {
    RCLCPP_WARN(
      get_logger(), "enable_torque_on_start ignored because allow_motion is false");
  }
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  action_server_ = rclcpp_action::create_server<MoveJoints>(
    this, "move_joints", std::bind(&HxArmDriverNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&HxArmDriverNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&HxArmDriverNode::handle_accepted, this, std::placeholders::_1));
  trajectory_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this, "arm_controller/follow_joint_trajectory",
    std::bind(&HxArmDriverNode::handle_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&HxArmDriverNode::handle_trajectory_cancel, this, std::placeholders::_1),
    std::bind(&HxArmDriverNode::handle_trajectory_accepted, this, std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Empty>("emergency_stop", rclcpp::QoS(1),
                                                                  [this](const std_msgs::msg::Empty &) {
      emergency_stop();
    });
  emergency_stop_service_ = create_service<std_srvs::srv::Trigger>("emergency_stop",
                                                                   [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      emergency_stop();
      response->success = true;
      response->message = "HX torque disabled";
    });
  publish_state(last_ticks_);
  if (!allow_motion_ && feedback_poll_period_ms > 0)
  {
    feedback_timer_ = create_wall_timer(
      std::chrono::milliseconds(feedback_poll_period_ms),
      std::bind(&HxArmDriverNode::poll_feedback, this));
    RCLCPP_INFO(
      get_logger(), "read-only HX feedback polling enabled every %ld ms; no motion commands will be sent",
      feedback_poll_period_ms);
  }
  RCLCPP_INFO(
    get_logger(), "opened %s at %d baud for %zu HX servos; motion=%s",
    device.c_str(), static_cast<int>(get_parameter("baud_rate").as_int()), count,
    allow_motion_ ? "enabled" : "disabled");
}

void HxArmDriverNode::poll_feedback()
{
  bool received_any = false;
  for (std::size_t i = 0; i < servo_ids_.size(); ++i)
  {
    const auto id = static_cast<uint8_t>(servo_ids_[i]);
    if (const auto position = read_position(i); position)
    {
      last_ticks_[i] = *position;
      online_[i] = true;
      received_any = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "HX servo id=%u joint=%s raw_tick=%d",
        static_cast<unsigned>(id), joint_names_[i].c_str(), static_cast<int>(*position));
    }
    else
    {
      online_[i] = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no position reply from HX servo id=%u joint=%s",
        static_cast<unsigned>(id), joint_names_[i].c_str());
    }
  }
  if (received_any)
  {
    publish_state(last_ticks_);
  }
}

rclcpp_action::GoalResponse HxArmDriverNode::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoints::Goal> goal)
{
  std::string reason;

  if (!validate_goal(*goal, reason))
  {
    RCLCPP_WARN(get_logger(), "rejecting MoveJoints goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;

  if (!moving_.compare_exchange_strong(expected, true))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HxArmDriverNode::handle_cancel(const std::shared_ptr<GoalHandleMoveJoints> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse HxArmDriverNode::handle_trajectory_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  std::string reason;
  if (!validate_trajectory(*goal, reason))
  {
    RCLCPP_WARN(get_logger(), "rejecting FollowJointTrajectory goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!moving_.compare_exchange_strong(expected, true))
  {
    RCLCPP_WARN(get_logger(), "rejecting trajectory: another motion goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HxArmDriverNode::handle_trajectory_cancel(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HxArmDriverNode::handle_trajectory_accepted(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> &goal_handle)
{
  std::thread{[this, goal_handle] {
    execute_trajectory(goal_handle);
  }}.detach();
}

bool HxArmDriverNode::validate_trajectory(
  const FollowJointTrajectory::Goal &goal, std::string &reason) const
{
  if (!allow_motion_)
  {
    reason = "motion is disabled; launch with allow_motion:=true after calibration";
    return false;
  }
  const auto &trajectory = goal.trajectory;
  if (trajectory.joint_names.empty() || trajectory.points.empty())
  {
    reason = "trajectory must contain joint_names and points";
    return false;
  }
  std::unordered_set<std::string> seen;
  for (const auto &name : trajectory.joint_names)
  {
    const auto it = joint_index_.find(name);
    if (it == joint_index_.end() || !seen.insert(name).second)
    {
      reason = "trajectory contains an unknown or duplicate joint name: " + name;
      return false;
    }
    if (!online_[it->second])
    {
      reason = "trajectory requests an offline servo: " + name;
      return false;
    }
  }

  int64_t previous_ms = 0;
  for (const auto &point : trajectory.points)
  {
    if (point.positions.size() != trajectory.joint_names.size())
    {
      reason = "each trajectory point must have one position per joint";
      return false;
    }
    if (!point.velocities.empty() || !point.accelerations.empty() || !point.effort.empty())
    {
      reason = "this servo driver accepts position-only trajectory points";
      return false;
    }
    const int64_t point_ms = static_cast<int64_t>(point.time_from_start.sec) * 1000 +
      point.time_from_start.nanosec / 1000000;
    if (point_ms <= previous_ms || point_ms < 20 || point_ms > 60000)
    {
      reason = "time_from_start must strictly increase and remain within 60 seconds";
      return false;
    }
    for (size_t i = 0; i < point.positions.size(); ++i)
    {
      if (!std::isfinite(point.positions[i]))
      {
        reason = "trajectory contains a non-finite position";
        return false;
      }
      const auto index = joint_index_.at(trajectory.joint_names[i]);
      const auto raw = std::lround(
        zero_ticks_[index] + directions_[index] * point.positions[i] * ticks_per_radian_[index]);
      if (raw < min_ticks_[index] || raw > max_ticks_[index])
      {
        reason = "trajectory exceeds calibrated limit for " + trajectory.joint_names[i];
        return false;
      }
    }
    previous_ms = point_ms;
  }
  return true;
}

void HxArmDriverNode::execute_trajectory(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> &goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto &trajectory = goal->trajectory;
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  const auto fail = [&result](const int32_t code, const std::string &message) {
      result->error_code = code;
      result->error_string = message;
    };

  try
  {
    const auto start = std::chrono::steady_clock::now();
    int64_t previous_ms = 0;
    for (const auto &point : trajectory.points)
    {
      if (goal_handle->is_canceling())
      {
        emergency_stop();
        fail(FollowJointTrajectory::Result::INVALID_GOAL, "trajectory cancelled; stop sent");
        goal_handle->canceled(result);
        moving_.store(false);
        return;
      }

      const int64_t point_ms = static_cast<int64_t>(point.time_from_start.sec) * 1000 +
        point.time_from_start.nanosec / 1000000;
      const auto segment_ms = static_cast<uint16_t>(std::clamp<int64_t>(point_ms - previous_ms, 20, 30000));
      std::vector<HxPositionTarget> hx_targets;
      hx_targets.reserve(trajectory.joint_names.size());

      for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
      {
        const auto index = joint_index_.at(trajectory.joint_names[i]);
        const auto target = std::lround(
          zero_ticks_[index] + directions_[index] * point.positions[i] * ticks_per_radian_[index]);
        if (target < min_ticks_[index] || target > max_ticks_[index])
        {
          throw std::out_of_range("trajectory target exceeds calibrated limit for " + trajectory.joint_names[i]);
        }
        const auto id = static_cast<uint8_t>(servo_ids_[index]);
        if (servo_protocols_[index] == "v3")
        {
          if (target < 0 || target > 1000)
          {
            throw std::out_of_range("V3 target must be in 0..1000 for " + trajectory.joint_names[i]);
          }
          bus_->send(encode_v3_position_command(id, static_cast<uint16_t>(target), segment_ms));
        }
        else
        {
          hx_targets.push_back({id, static_cast<int16_t>(target), goal_speed_, goal_acceleration_});
        }
      }
      for (const auto &target : hx_targets)
      {
        write_checked(
          target.id,
          encode_hx_reg_position_command(target.id, target.position, target.speed, target.acceleration),
          "staging trajectory target for");
      }
      if (!hx_targets.empty())
      {
        bus_->send(encode_hx_action_command());
      }

      const auto target_time = start + std::chrono::milliseconds(point_ms);
      const auto requested_goal_tolerance_ms =
        static_cast<int64_t>(goal->goal_time_tolerance.sec) * 1000 +
        goal->goal_time_tolerance.nanosec / 1000000;
      const auto grace_ms = requested_goal_tolerance_ms > 0 ? requested_goal_tolerance_ms : 1000;
      const auto deadline = target_time + std::chrono::milliseconds(grace_ms);
      bool reached = false;
      while (std::chrono::steady_clock::now() < deadline)
      {
        if (goal_handle->is_canceling())
        {
          emergency_stop();
          fail(FollowJointTrajectory::Result::INVALID_GOAL, "trajectory cancelled; stop sent");
          goal_handle->canceled(result);
          moving_.store(false);
          return;
        }
        if (std::chrono::steady_clock::now() < target_time)
        {
          std::this_thread::sleep_for(10ms);
          continue;
        }

        bool within_tolerance = true;
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.stamp = now();
        feedback->joint_names = trajectory.joint_names;
        feedback->desired.positions = point.positions;
        feedback->actual.positions.reserve(point.positions.size());
        feedback->error.positions.resize(point.positions.size());
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
        {
          const auto index = joint_index_.at(trajectory.joint_names[i]);
          if (const auto observed = read_position(index); observed)
          {
            last_ticks_[index] = *observed;
            online_[index] = true;
          }
          else
          {
            online_[index] = false;
            within_tolerance = false;
          }
          const auto actual =
            (static_cast<double>(last_ticks_[index]) - zero_ticks_[index]) /
            (directions_[index] * ticks_per_radian_[index]);
          feedback->actual.positions.push_back(actual);
          feedback->error.positions[i] = point.positions[i] - actual;
          const auto target_ticks = std::lround(
            zero_ticks_[index] + directions_[index] * point.positions[i] * ticks_per_radian_[index]);
          if (std::abs(static_cast<int>(last_ticks_[index]) - static_cast<int>(target_ticks)) > position_tolerance_ticks_)
          {
            within_tolerance = false;
          }
        }
        publish_state(last_ticks_);
        goal_handle->publish_feedback(feedback);
        if (within_tolerance)
        {
          reached = true;
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
      if (!reached)
      {
        fail(FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
             "servos did not reach trajectory point within the allowed time");
        goal_handle->abort(result);
        moving_.store(false);
        return;
      }
      previous_ms = point_ms;
    }

    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    result->error_string = "trajectory completed";
    goal_handle->succeed(result);
  }
  catch (const std::exception &error)
  {
    fail(FollowJointTrajectory::Result::INVALID_GOAL, error.what());
    goal_handle->abort(result);
  }
  moving_.store(false);
}

void HxArmDriverNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> &goal_handle)
{
  std::thread{[this, goal_handle] {
                execute(goal_handle);
              }}.detach();
}

bool HxArmDriverNode::validate_goal(const MoveJoints::Goal &goal, std::string &reason) const
{
  if (!allow_motion_)
  {
    reason = "motion is disabled; set allow_motion=true only after calibration and safety checks";
    return false;
  }
  if (goal.joint_names.empty() || goal.joint_names.size() != goal.positions.size())
  {
    reason = "joint_names and positions must be non-empty and same length";
    return false;
  }
  if (!goal.speeds.empty() && goal.speeds.size() != goal.joint_names.size())
  {
    reason = "speeds must be empty or have one value per joint";
    return false;
  }
  const auto ms = static_cast<int64_t>(goal.duration.sec) * 1000 + goal.duration.nanosec / 1000000;

  if (ms < 20 || ms > 30000)
  {
    reason = "duration must be between 20 and 30000 ms";
    return false;
  }
  std::unordered_set<std::string> seen;

  for (size_t i = 0; i < goal.joint_names.size(); ++i)
  {
    if (!std::isfinite(goal.positions[i]) || !joint_index_.count(goal.joint_names[i]) || !seen.insert(goal.joint_names[i]).second)
    {
      reason = "unknown, duplicate, or non-finite joint target";
      return false;
    }
    if (!goal.speeds.empty() && goal.speeds[i] > 3400U)
    {
      reason = "each speed must be 0..3400";
      return false;
    }
    if (!online_[joint_index_.at(goal.joint_names[i])])
    {
      reason = "requested servo is offline";
      return false;
    }
  }
  return true;
}

std::vector<int16_t> HxArmDriverNode::goal_to_ticks(const MoveJoints::Goal &goal) const
{
  auto ticks = last_ticks_;

  for (size_t i = 0; i < goal.joint_names.size(); ++i)
  {
    const auto index = joint_index_.at(goal.joint_names[i]);
    const auto target = std::lround(zero_ticks_[index] + directions_[index] * goal.positions[i] * ticks_per_radian_[index]);
    if (target < min_ticks_[index] || target > max_ticks_[index])
    {
      throw std::out_of_range("target exceeds calibrated HX limit for " + goal.joint_names[i]);
    }
    ticks[index] = static_cast<int16_t>(target);
  }
  return ticks;
}

sensor_msgs::msg::JointState HxArmDriverNode::state_from_ticks(const std::vector<int16_t> &ticks) const
{
  sensor_msgs::msg::JointState state;

  state.header.stamp = now();
  state.name = joint_names_;
  state.position.resize(ticks.size());

  for (size_t i = 0; i < ticks.size(); ++i)
  {
    state.position[i] = (static_cast<double>(ticks[i]) - zero_ticks_[i]) / (directions_[i] * ticks_per_radian_[i]);
  }
  return state;
}

void HxArmDriverNode::publish_state(const std::vector<int16_t> &ticks)
{
  joint_state_pub_->publish(state_from_ticks(ticks));
}
std::optional<int16_t> HxArmDriverNode::read_position(const size_t index)
{
  const auto id = static_cast<uint8_t>(servo_ids_[index]);
  if (servo_protocols_[index] == "v3")
  {
    const auto reply = bus_->request_v3(
      encode_v3_frame(id, kV3PositionReadCommand, {}), feedback_timeout_ms_);
    if (!reply || reply->size() < 8U || (*reply)[2] != id || (*reply)[4] != kV3PositionReadCommand)
    {
      return std::nullopt;
    }
    return decode_v3_position((*reply)[5], (*reply)[6]);
  }

  const auto reply = bus_->request(
    encode_at32_frame(id, kHxRead, {kPresentPositionL, kPresentPositionSize}), feedback_timeout_ms_);

  if (!reply || reply->size() < 8U || (*reply)[2] != id || (*reply)[4] != 0)
  {
    return std::nullopt;
  }
  return decode_hx_position((*reply)[5], (*reply)[6]);
}
void HxArmDriverNode::write_checked(const uint8_t id, const std::vector<uint8_t> &frame, const std::string &operation)
{
  const auto reply = bus_->request(frame);

  if (!reply || reply->size() < 6U || (*reply)[2] != id)
  {
    throw std::runtime_error("no status reply while " + operation + " servo ID " + std::to_string(id));
  }
  if ((*reply)[4] != 0U)
  {
    throw std::runtime_error("HX error " + std::to_string((*reply)[4]) + " while " + operation + " servo ID " + std::to_string(id));
  }
}
void HxArmDriverNode::set_position_mode(const uint8_t id)
{
  write_checked(id, encode_hx_mode_command(id, 0), "selecting position mode for");
}
void HxArmDriverNode::set_torque(const uint8_t id, const bool enabled)
{
  write_checked(id, encode_hx_torque_command(id, enabled), enabled ? "enabling torque for" : "disabling torque for");
}

void HxArmDriverNode::execute(const std::shared_ptr<GoalHandleMoveJoints> &goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoints::Result>();

  try {
    const auto ticks = goal_to_ticks(*goal);
    const auto duration_ms = static_cast<uint16_t>(
      std::clamp<int64_t>(static_cast<int64_t>(goal->duration.sec) * 1000 + goal->duration.nanosec / 1000000, 1, 30000));
    if (goal->joint_names.size() == 1U)
    {
      const auto index = joint_index_.at(goal->joint_names.front());
      const auto id = static_cast<uint8_t>(servo_ids_[index]);
      const auto speed = goal->speeds.empty() ? goal_speed_ : goal->speeds.front();
      if (servo_protocols_[index] == "v3")
      {
        if (ticks[index] < 0 || ticks[index] > 1000)
        {
          throw std::out_of_range("V3 target position must be 0..1000");
        }
        bus_->send(encode_v3_position_command(id, static_cast<uint16_t>(ticks[index]), duration_ms));
      }
      else
      {
        write_checked(id, encode_hx_position_command(id, ticks[index], speed, goal_acceleration_), "writing target position to");
      }
    }
    else
    {
      std::vector<HxPositionTarget> targets;
      std::vector<size_t> target_indices;
      targets.reserve(goal->joint_names.size());
      target_indices.reserve(goal->joint_names.size());
      bool hx_targets_staged = false;
      for (size_t i = 0; i < goal->joint_names.size(); ++i)
      {
        const auto &name = goal->joint_names[i];
        const auto index = joint_index_.at(name);
        const auto speed = goal->speeds.empty() ? goal_speed_ : goal->speeds[i];
        targets.push_back({static_cast<uint8_t>(servo_ids_[index]), ticks[index], speed, goal_acceleration_});
        target_indices.push_back(index);
      }
      for (size_t target_index = 0; target_index < targets.size(); ++target_index)
      {
        const auto &target = targets[target_index];
        if (servo_protocols_[target_indices[target_index]] == "v3")
        {
          if (target.position < 0 || target.position > 1000)
          {
            throw std::out_of_range("V3 target position must be 0..1000");
          }
          bus_->send(encode_v3_position_command(target.id, static_cast<uint16_t>(target.position), duration_ms));
        }
        else
        {
          write_checked(target.id,
                        encode_hx_reg_position_command(target.id, target.position, target.speed, target.acceleration),
                        "staging synchronized target for");
          hx_targets_staged = true;
        }
      }
      if (hx_targets_staged)
      {
        bus_->send(encode_hx_action_command());
      }
      RCLCPP_INFO(get_logger(), "mixed HX/V3 position commands sent to %zu servos", targets.size());
    }
    const auto duration = std::chrono::milliseconds(goal->duration.sec * 1000 + goal->duration.nanosec / 1000000);
    const auto start = std::chrono::steady_clock::now();
    bool reached_target = !goal->wait_for_completion;
    while (goal->wait_for_completion && std::chrono::steady_clock::now() - start < duration)
    {
      if (goal_handle->is_canceling())
      {
        emergency_stop();
        result->success = false;
        result->message = "motion cancelled and torque disabled";
        result->final_state = state_from_ticks(last_ticks_);
        goal_handle->canceled(result);
        moving_.store(false);
        return;
      }
      bool all_within_tolerance = true;
      for (const auto &name : goal->joint_names)
      {
        const auto index = joint_index_.at(name);
        const auto observed = read_position(index);
        if (!observed)
        {
          all_within_tolerance = false;
          continue;
        }
        last_ticks_[index] = *observed;
        if (std::abs(static_cast<int>(*observed) - static_cast<int>(ticks[index])) > position_tolerance_ticks_)
        {
          all_within_tolerance = false;
        }
      }
      auto feedback = std::make_shared<MoveJoints::Feedback>();
      feedback->progress = std::min(1.0F, static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()) / duration.count());
      feedback->current_state = state_from_ticks(last_ticks_);
      goal_handle->publish_feedback(feedback);
      if (all_within_tolerance)
      {
        reached_target = true;
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
    if (goal->wait_for_completion && !reached_target)
    {
      publish_state(last_ticks_);
      result->success = false;
      result->message = "motion timeout: one or more joints did not reach target";
      result->final_state = state_from_ticks(last_ticks_);
      goal_handle->abort(result);
      moving_.store(false);
      return;
    }
    if (!goal->wait_for_completion)
    {
      last_ticks_ = ticks;
    }
    publish_state(last_ticks_);
    result->success = true;
    result->message = goal->wait_for_completion ? "HX target reached" : "HX motion command sent";
    result->final_state = state_from_ticks(last_ticks_);
    goal_handle->succeed(result);
  }
  catch (const std::exception &error) {result->success = false;
                                       result->message = error.what();
                                       result->final_state = state_from_ticks(last_ticks_);
                                       goal_handle->abort(result);
  }
  moving_.store(false);
}

void HxArmDriverNode::emergency_stop()
{
  for (size_t i = 0; i < servo_ids_.size(); ++i)
  {
    if (!online_[i])
    {
      continue;
    }
    try {
      if (servo_protocols_[i] == "v3")
      {
        bus_->send(encode_v3_stop_command(static_cast<uint8_t>(servo_ids_[i])));
      }
      else
      {
        bus_->send(encode_hx_torque_command(static_cast<uint8_t>(servo_ids_[i]), false));
      }
    }
    catch (const std::exception &error) {RCLCPP_ERROR(get_logger(), "failed to disable torque: %s", error.what());
    }
  }
}
}  // namespace robot_arm_driver
