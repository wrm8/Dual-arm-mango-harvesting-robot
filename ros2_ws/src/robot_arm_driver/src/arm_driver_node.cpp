#include "robot_arm_driver/arm_driver_node.hpp"

#include "robot_arm_driver/servo_protocol.hpp"

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
ArmDriverNode::ArmDriverNode() : Node("robot_arm_driver")
{
  declare_parameter("serial_device", "/dev/ttyACM1");
  declare_parameter("baud_rate", 115200);
  declare_parameter("use_direction_gpio", false);
  declare_parameter("tx_enable_gpio", -1);
  declare_parameter("rx_enable_gpio", -1);
  declare_parameter("read_feedback_on_completion", false);
  declare_parameter("read_feedback_on_start", false);
  declare_parameter("feedback_poll_period_ms", 200);
  declare_parameter("feedback_timeout_ms", 50);
  declare_parameter("allow_motion", false);
  declare_parameter("joint_names", std::vector<std::string>{
      "joint_1", "joint_2", "joint_3", "joint_4", "wrist_roll", "gripper"});
  declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
  declare_parameter("zero_ticks", std::vector<double>{500, 500, 500, 500, 500, 500});
  declare_parameter("ticks_per_radian", std::vector<double>{238.7324, 238.7324, 238.7324, 238.7324, 238.7324, 238.7324});
  declare_parameter("directions", std::vector<double>{1, 1, 1, 1, 1, 1});
  declare_parameter("min_ticks", std::vector<int64_t>{0, 0, 0, 0, 0, 0});
  declare_parameter("max_ticks", std::vector<int64_t>{1000, 1000, 1000, 1000, 1000, 1000});

  joint_names_ = get_parameter("joint_names").as_string_array();
  servo_ids_ = get_parameter("servo_ids").as_integer_array();
  zero_ticks_ = get_parameter("zero_ticks").as_double_array();
  ticks_per_radian_ = get_parameter("ticks_per_radian").as_double_array();
  directions_ = get_parameter("directions").as_double_array();
  min_ticks_ = get_parameter("min_ticks").as_integer_array();
  max_ticks_ = get_parameter("max_ticks").as_integer_array();
  read_feedback_on_completion_ = get_parameter("read_feedback_on_completion").as_bool();
  const auto read_feedback_on_start = get_parameter("read_feedback_on_start").as_bool();
  const auto feedback_poll_period_ms = get_parameter("feedback_poll_period_ms").as_int();
  feedback_timeout_ms_ = static_cast<int>(get_parameter("feedback_timeout_ms").as_int());
  allow_motion_ = get_parameter("allow_motion").as_bool();
  if (feedback_poll_period_ms < 0 || feedback_poll_period_ms > 60000 ||
      feedback_timeout_ms_ < 1 || feedback_timeout_ms_ > 1000)
  {
    throw std::invalid_argument("invalid feedback polling period or timeout");
  }
  const auto count = joint_names_.size();

  if (count == 0U || servo_ids_.size() != count || zero_ticks_.size() != count ||
      ticks_per_radian_.size() != count || directions_.size() != count ||
      min_ticks_.size() != count || max_ticks_.size() != count)
  {
    throw std::invalid_argument("all joint calibration arrays must have the same non-zero size");
  }
  for (size_t i = 0; i < count; ++i)
  {
    if (!joint_index_.emplace(joint_names_[i], i).second || servo_ids_[i] < 0 || servo_ids_[i] > 253 ||
        ticks_per_radian_[i] <= 0.0 || std::abs(directions_[i]) != 1.0 || min_ticks_[i] < 0 ||
        max_ticks_[i] > 1000 || min_ticks_[i] > max_ticks_[i])
    {
      throw std::invalid_argument("invalid joint calibration parameter at index " + std::to_string(i));
    }
  }
  last_ticks_.resize(count);
  std::transform(zero_ticks_.begin(), zero_ticks_.end(), last_ticks_.begin(), [](const double tick) {
      return static_cast<uint16_t>(std::clamp(std::lround(tick), 0L, 1000L));
    });

  BusConfig bus_config;

  bus_config.device = get_parameter("serial_device").as_string();
  bus_config.baud_rate = static_cast<int>(get_parameter("baud_rate").as_int());
  bus_config.use_direction_gpio = get_parameter("use_direction_gpio").as_bool();
  bus_config.tx_enable_gpio = static_cast<int>(get_parameter("tx_enable_gpio").as_int());
  bus_config.rx_enable_gpio = static_cast<int>(get_parameter("rx_enable_gpio").as_int());
  bus_ = std::make_unique<ServoBus>(bus_config);
  bus_->open();

  if (read_feedback_on_start)
  {
    for (std::size_t i = 0; i < count; ++i)
    {
      const auto id = static_cast<uint8_t>(servo_ids_[i]);
      if (const auto position = bus_->read_position(id); position)
      {
        last_ticks_[i] = *position;
        RCLCPP_INFO(
          get_logger(), "V3 servo id=%u joint=%s raw_tick=%u",
          static_cast<unsigned>(id), joint_names_[i].c_str(),
          static_cast<unsigned>(*position));
      }
      else
      {
        RCLCPP_WARN(
          get_logger(), "no position reply from V3 servo id=%u joint=%s",
          static_cast<unsigned>(id), joint_names_[i].c_str());
      }
    }
  }

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  action_server_ = rclcpp_action::create_server<MoveJoints>(
    this, "move_joints",
    std::bind(&ArmDriverNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ArmDriverNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&ArmDriverNode::handle_accepted, this, std::placeholders::_1));
  trajectory_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    "arm_controller/follow_joint_trajectory",
    std::bind(&ArmDriverNode::handle_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ArmDriverNode::handle_trajectory_cancel, this, std::placeholders::_1),
    std::bind(&ArmDriverNode::handle_trajectory_accepted, this, std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Empty>("emergency_stop", rclcpp::QoS(1),
                                                                  [this](const std_msgs::msg::Empty &) {
      emergency_stop();
    });
  emergency_stop_service_ = create_service<std_srvs::srv::Trigger>("emergency_stop",
                                                                   [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      emergency_stop();
      response->success = true;
      response->message = "stop command sent";
    });
  publish_state(last_ticks_);
  if (!allow_motion_ && feedback_poll_period_ms > 0)
  {
    feedback_timer_ = create_wall_timer(
      std::chrono::milliseconds(feedback_poll_period_ms),
      std::bind(&ArmDriverNode::poll_feedback, this));
    RCLCPP_INFO(
      get_logger(), "read-only feedback polling enabled every %ld ms; no motion commands will be sent",
      feedback_poll_period_ms);
  }
  RCLCPP_INFO(
    get_logger(), "opened %s for %zu bus servos; motion=%s",
    bus_config.device.c_str(), count, allow_motion_ ? "enabled" : "disabled");
}

void ArmDriverNode::poll_feedback()
{
  bool received_any = false;
  for (std::size_t i = 0; i < servo_ids_.size(); ++i)
  {
    const auto id = static_cast<uint8_t>(servo_ids_[i]);
    if (const auto position = bus_->read_position(id, feedback_timeout_ms_); position)
    {
      last_ticks_[i] = *position;
      received_any = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "V3 servo id=%u joint=%s raw_tick=%u",
        static_cast<unsigned>(id), joint_names_[i].c_str(),
        static_cast<unsigned>(*position));
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no position reply from V3 servo id=%u joint=%s",
        static_cast<unsigned>(id), joint_names_[i].c_str());
    }
  }
  if (received_any)
  {
    publish_state(last_ticks_);
  }
}

rclcpp_action::GoalResponse ArmDriverNode::handle_goal(
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
    RCLCPP_WARN(get_logger(), "rejecting MoveJoints goal: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmDriverNode::handle_cancel(const std::shared_ptr<GoalHandleMoveJoints>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmDriverNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  std::thread{[this, goal_handle] {
                execute(goal_handle);
              }}.detach();
}

bool ArmDriverNode::validate_goal(const MoveJoints::Goal &goal, std::string &reason) const
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
  }
  return true;
}

rclcpp_action::GoalResponse ArmDriverNode::handle_trajectory_goal(
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
    RCLCPP_WARN(get_logger(), "rejecting FollowJointTrajectory goal: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmDriverNode::handle_trajectory_cancel(
  const std::shared_ptr<GoalHandleFollowJointTrajectory>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmDriverNode::handle_trajectory_accepted(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  std::thread{[this, goal_handle] {
    execute_trajectory(goal_handle);
  }}.detach();
}

bool ArmDriverNode::validate_trajectory(
  const FollowJointTrajectory::Goal &goal, std::string &reason) const
{
  if (!allow_motion_)
  {
    reason = "motion is disabled; set allow_motion=true only after calibration and safety checks";
    return false;
  }
  const auto &trajectory = goal.trajectory;
  if (trajectory.joint_names.empty() || trajectory.points.empty())
  {
    reason = "trajectory must contain joint_names and at least one point";
    return false;
  }

  std::unordered_set<std::string> seen;
  for (const auto &name : trajectory.joint_names)
  {
    if (!joint_index_.count(name) || !seen.insert(name).second)
    {
      reason = "trajectory contains an unknown or duplicate joint name";
      return false;
    }
  }

  int64_t previous_ms = 0;
  for (const auto &point : trajectory.points)
  {
    if (point.positions.size() != trajectory.joint_names.size())
    {
      reason = "each trajectory point must contain one position per joint";
      return false;
    }
    const int64_t point_ms = static_cast<int64_t>(point.time_from_start.sec) * 1000 +
      point.time_from_start.nanosec / 1000000;
    if (point_ms <= previous_ms || point_ms < 20 || point_ms > 60000)
    {
      reason = "time_from_start must be strictly increasing and within 20..60000 ms";
      return false;
    }
    for (std::size_t i = 0; i < point.positions.size(); ++i)
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
        reason = "trajectory position exceeds the calibrated servo limit";
        return false;
      }
    }
    previous_ms = point_ms;
  }
  return true;
}

void ArmDriverNode::execute_trajectory(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  const auto goal = goal_handle->get_goal();

  try
  {
    const auto &trajectory = goal->trajectory;
    const auto start = std::chrono::steady_clock::now();
    int64_t previous_ms = 0;

    for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index)
    {
      const auto &point = trajectory.points[point_index];
      const int64_t point_ms = static_cast<int64_t>(point.time_from_start.sec) * 1000 +
        point.time_from_start.nanosec / 1000000;
      const auto segment_ms = static_cast<uint16_t>(std::max<int64_t>(20, point_ms - previous_ms));

      std::vector<uint16_t> ticks = last_ticks_;
      for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
      {
        const auto index = joint_index_.at(trajectory.joint_names[i]);
        const auto raw = std::lround(
          zero_ticks_[index] + directions_[index] * point.positions[i] * ticks_per_radian_[index]);
        ticks[index] = static_cast<uint16_t>(raw);
        bus_->move(static_cast<uint8_t>(servo_ids_[index]), ticks[index], segment_ms);
      }

      const auto target_time = start + std::chrono::milliseconds(point_ms);
      while (std::chrono::steady_clock::now() < target_time)
      {
        if (goal_handle->is_canceling())
        {
          emergency_stop();
          result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
          result->error_string = "trajectory cancelled; stop command sent";
          goal_handle->canceled(result);
          moving_.store(false);
          return;
        }
        std::this_thread::sleep_for(20ms);
      }

      last_ticks_ = ticks;
      if (read_feedback_on_completion_)
      {
        for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
        {
          const auto index = joint_index_.at(trajectory.joint_names[i]);
          if (const auto observed = bus_->read_position(static_cast<uint8_t>(servo_ids_[index])); observed)
          {
            last_ticks_[index] = *observed;
          }
        }
      }
      publish_state(last_ticks_);

      auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
      feedback->header.stamp = now();
      feedback->joint_names = trajectory.joint_names;
      feedback->actual.positions.reserve(trajectory.joint_names.size());
      feedback->desired.positions = point.positions;
      feedback->error.positions.resize(point.positions.size());
      for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
      {
        const auto index = joint_index_.at(trajectory.joint_names[i]);
        feedback->actual.positions.push_back(
          (static_cast<double>(last_ticks_[index]) - zero_ticks_[index]) /
          (directions_[index] * ticks_per_radian_[index]));
        feedback->error.positions[i] =
          point.positions[i] - feedback->actual.positions[i];
      }
      goal_handle->publish_feedback(feedback);
      previous_ms = point_ms;
    }

    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    result->error_string = "trajectory completed";
    goal_handle->succeed(result);
  }
  catch (const std::exception &error)
  {
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = error.what();
    goal_handle->abort(result);
  }
  moving_.store(false);
}

std::vector<uint16_t> ArmDriverNode::goal_to_ticks(const MoveJoints::Goal &goal) const
{
  auto ticks = last_ticks_;

  for (size_t i = 0; i < goal.joint_names.size(); ++i)
  {
    const auto index = joint_index_.at(goal.joint_names[i]);
    const auto raw = zero_ticks_[index] + directions_[index] * goal.positions[i] * ticks_per_radian_[index];
    const auto rounded = std::lround(raw);
    if (rounded < min_ticks_[index] || rounded > max_ticks_[index])
    {
      throw std::out_of_range("target exceeds calibrated servo limit for " + goal.joint_names[i]);
    }
    ticks[index] = static_cast<uint16_t>(rounded);
  }
  return ticks;
}

sensor_msgs::msg::JointState ArmDriverNode::state_from_ticks(const std::vector<uint16_t> &ticks) const
{
  sensor_msgs::msg::JointState state;

  state.header.stamp = now();
  state.name = joint_names_;
  state.position.resize(ticks.size());
  for (size_t i = 0; i < ticks.size(); ++i)
  {
    state.position[i] = (static_cast<double>(ticks[i]) - zero_ticks_[i]) /
                        (directions_[i] * ticks_per_radian_[i]);
  }
  return state;
}

void ArmDriverNode::publish_state(const std::vector<uint16_t> &ticks)
{
  joint_state_pub_->publish(state_from_ticks(ticks));
}

void ArmDriverNode::execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoints::Result>();

  try {
    const auto ticks = goal_to_ticks(*goal);
    const auto duration_ms = static_cast<uint16_t>(goal->duration.sec * 1000 + goal->duration.nanosec / 1000000);
    for (const auto &joint_name : goal->joint_names)
    {
      const auto index = joint_index_.at(joint_name);
      bus_->move(static_cast<uint8_t>(servo_ids_[index]), ticks[index], duration_ms);
    }
    const auto start = std::chrono::steady_clock::now();
    const auto duration = std::chrono::milliseconds(duration_ms);
    do
    {
      if (goal_handle->is_canceling())
      {
        emergency_stop();
        result->success = false;
        result->message = "motion cancelled and stop command sent";
        result->final_state = state_from_ticks(last_ticks_);
        goal_handle->canceled(result);
        moving_.store(false);
        return;
      }
      const auto elapsed = std::chrono::steady_clock::now() - start;
      auto feedback = std::make_shared<MoveJoints::Feedback>();
      feedback->progress = std::min(1.0F, static_cast<float>(
                                      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()) / duration_ms);
      feedback->current_state = state_from_ticks(last_ticks_);
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(20ms);
    } while (goal->wait_for_completion && std::chrono::steady_clock::now() - start < duration);

    last_ticks_ = ticks;
    if (read_feedback_on_completion_)
    {
      for (size_t i = 0; i < last_ticks_.size(); ++i)
      {
        if (const auto observed = bus_->read_position(static_cast<uint8_t>(servo_ids_[i])); observed)
        {
          last_ticks_[i] = *observed;
        }
      }
    }
    publish_state(last_ticks_);
    result->success = true;
    result->message = goal->wait_for_completion ? "motion completed" : "motion command sent";
    result->final_state = state_from_ticks(last_ticks_);
    goal_handle->succeed(result);
  }
  catch (const std::exception &error) {
    result->success = false;
    result->message = error.what();
    result->final_state = state_from_ticks(last_ticks_);
    goal_handle->abort(result);
  }
  moving_.store(false);
}

void ArmDriverNode::emergency_stop()
{
  try {
    bus_->stop(kBroadcastId);
    RCLCPP_WARN(get_logger(), "broadcast servo stop sent");
  }
  catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "emergency stop failed: %s", error.what());
  }
}
}  // namespace robot_arm_driver
