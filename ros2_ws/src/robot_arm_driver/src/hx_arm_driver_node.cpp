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
  declare_parameter("serial_device", "/dev/ttyACM0");
  declare_parameter("baud_rate", 1000000);
  declare_parameter("joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4"});
  declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4});
  declare_parameter("zero_ticks", std::vector<double>{0, 0, 0, 0});
  declare_parameter("ticks_per_radian", std::vector<double>{651.8986, 651.8986, 651.8986, 651.8986});
  declare_parameter("directions", std::vector<double>{1, 1, 1, 1});
  declare_parameter("min_ticks", std::vector<int64_t>{-30719, -30719, -30719, -30719});
  declare_parameter("max_ticks", std::vector<int64_t>{30719, 30719, 30719, 30719});
  declare_parameter("goal_speed", 100);
  declare_parameter("goal_acceleration", 0);
  declare_parameter("position_tolerance_ticks", 20);
  declare_parameter("enable_torque_on_start", true);

  joint_names_ = get_parameter("joint_names").as_string_array();
  servo_ids_ = get_parameter("servo_ids").as_integer_array();
  zero_ticks_ = get_parameter("zero_ticks").as_double_array();
  ticks_per_radian_ = get_parameter("ticks_per_radian").as_double_array();
  directions_ = get_parameter("directions").as_double_array();
  min_ticks_ = get_parameter("min_ticks").as_integer_array();
  max_ticks_ = get_parameter("max_ticks").as_integer_array();
  const auto count = joint_names_.size();

  if (count == 0U || servo_ids_.size() != count || zero_ticks_.size() != count ||
      ticks_per_radian_.size() != count || directions_.size() != count ||
      min_ticks_.size() != count || max_ticks_.size() != count)
  {
    throw std::invalid_argument("all joint calibration arrays must have the same non-zero size");
  }
  const auto configured_speed = get_parameter("goal_speed").as_int();
  const auto configured_acceleration = get_parameter("goal_acceleration").as_int();
  const auto configured_tolerance = get_parameter("position_tolerance_ticks").as_int();

  if (configured_speed < 0 || configured_speed > 3400 || configured_acceleration < 0 || configured_acceleration > 254 ||
      configured_tolerance < 0 || configured_tolerance > 1000)
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
        ticks_per_radian_[i] <= 0.0 || std::abs(directions_[i]) != 1.0 || min_ticks_[i] < -30719 ||
        max_ticks_[i] > 30719 || min_ticks_[i] > max_ticks_[i])
    {
      throw std::invalid_argument("invalid HX-30HM calibration parameter at index " + std::to_string(i));
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
    if (const auto position = read_position(static_cast<uint8_t>(servo_ids_[i])); position)
    {
      last_ticks_[i] = *position;
      online_[i] = true;
      RCLCPP_INFO(get_logger(), "HX-30HM ID %ld present position: %d", servo_ids_[i], *position);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "no valid HX-30HM position reply from ID %ld; verify ID, 1 Mbps bus, and protocol", servo_ids_[i]);
    }
  }
  if (get_parameter("enable_torque_on_start").as_bool())
  {
    for (size_t i = 0; i < servo_ids_.size(); ++i)
    {
      if (!online_[i])
      {
        continue;
      }
      set_position_mode(static_cast<uint8_t>(servo_ids_[i]));
      set_torque(static_cast<uint8_t>(servo_ids_[i]), true);
    }
  }
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  action_server_ = rclcpp_action::create_server<MoveJoints>(
    this, "move_joints", std::bind(&HxArmDriverNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&HxArmDriverNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&HxArmDriverNode::handle_accepted, this, std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Empty>("emergency_stop", rclcpp::QoS(1),
                                                                  [this](const std_msgs::msg::Empty &) {
      emergency_stop();
    });
  emergency_stop_service_ = create_service<std_srvs::srv::Trigger>("emergency_stop",
                                                                   [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      emergency_stop();
      response->success = true;
      response->message = "HX-30HM torque disabled";
    });
  publish_state(last_ticks_);
  RCLCPP_INFO(get_logger(), "opened %s at 1 Mbps for %zu HX-30HM servos", device.c_str(), count);
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
void HxArmDriverNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> &goal_handle)
{
  std::thread{[this, goal_handle] {
                execute(goal_handle);
              }}.detach();
}

bool HxArmDriverNode::validate_goal(const MoveJoints::Goal &goal, std::string &reason) const
{
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
      throw std::out_of_range("target exceeds calibrated HX-30HM limit for " + goal.joint_names[i]);
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
std::optional<int16_t> HxArmDriverNode::read_position(const uint8_t id)
{
  const auto reply = bus_->request(encode_at32_frame(id, 2, {56, 2}));

  if (!reply || reply->size() < 8U || (*reply)[2] != id || (*reply)[4] != 0)
  {
    return std::nullopt;
  }
  const auto raw = static_cast<uint16_t>((*reply)[5] | (static_cast<uint16_t>((*reply)[6]) << 8U));

  return static_cast<int16_t>(raw);
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
    throw std::runtime_error("HX-30HM error " + std::to_string((*reply)[4]) + " while " + operation + " servo ID " + std::to_string(id));
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
    if (goal->joint_names.size() == 1U)
    {
      const auto index = joint_index_.at(goal->joint_names.front());
      const auto id = static_cast<uint8_t>(servo_ids_[index]);
      const auto speed = goal->speeds.empty() ? goal_speed_ : goal->speeds.front();
      write_checked(id, encode_hx_position_command(id, ticks[index], speed, goal_acceleration_), "writing target position to");
    }
    else
    {
      std::vector<HxPositionTarget> targets;
      targets.reserve(goal->joint_names.size());
      for (size_t i = 0; i < goal->joint_names.size(); ++i)
      {
        const auto &name = goal->joint_names[i];
        const auto index = joint_index_.at(name);
        const auto speed = goal->speeds.empty() ? goal_speed_ : goal->speeds[i];
        targets.push_back({static_cast<uint8_t>(servo_ids_[index]), ticks[index], speed, goal_acceleration_});
      }
      for (const auto &target : targets)
      {
        write_checked(target.id,
                      encode_hx_reg_position_command(target.id, target.position, target.speed, target.acceleration),
                      "staging synchronized target for");
      }
      bus_->send(encode_hx_action_command());
      RCLCPP_INFO(get_logger(), "HX-30HM REG_WRITE + broadcast ACTION sent to %zu servos", targets.size());
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
        const auto observed = read_position(static_cast<uint8_t>(servo_ids_[index]));
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
    result->message = goal->wait_for_completion ? "HX-30HM target reached" : "HX-30HM motion command sent";
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
    try {bus_->send(encode_hx_torque_command(static_cast<uint8_t>(servo_ids_[i]), false));
    }
    catch (const std::exception &error) {RCLCPP_ERROR(get_logger(), "failed to disable torque: %s", error.what());
    }
  }
}
}  // namespace robot_arm_driver
