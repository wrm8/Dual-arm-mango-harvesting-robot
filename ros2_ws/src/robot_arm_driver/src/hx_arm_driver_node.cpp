#include "robot_arm_driver/hx_arm_driver_node.hpp"

#include "robot_arm_driver/coordinate_protocol.hpp"
#include "robot_arm_driver/joint_stationary_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
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
  declare_parameter("urdf_zero_ticks", std::vector<double>{});
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
  declare_parameter("stationary_tolerance_ticks", 3);
  declare_parameter("stationary_duration_ms", 500);
  declare_parameter("stationary_min_wait_ms", 1000);
  declare_parameter("feedback_poll_period_ms", 200);
  declare_parameter("feedback_timeout_ms", 50);
  declare_parameter("allow_motion", false);
  declare_parameter("require_all_feedback_for_joint_states", false);
  declare_parameter("enable_torque_on_start", false);

  joint_names_ = get_parameter("joint_names").as_string_array();
  servo_ids_ = get_parameter("servo_ids").as_integer_array();
  servo_protocols_ = get_parameter("servo_protocols").as_string_array();
  zero_ticks_ = get_parameter("zero_ticks").as_double_array();
  urdf_zero_ticks_ = get_parameter("urdf_zero_ticks").as_double_array();
  if (urdf_zero_ticks_.empty())
  {
    urdf_zero_ticks_ = zero_ticks_;
  }
  ticks_per_radian_ = get_parameter("ticks_per_radian").as_double_array();
  directions_ = get_parameter("directions").as_double_array();
  min_ticks_ = get_parameter("min_ticks").as_integer_array();
  max_ticks_ = get_parameter("max_ticks").as_integer_array();
  const auto count = joint_names_.size();

  if (count == 0U || servo_ids_.size() != count || servo_protocols_.size() != count || zero_ticks_.size() != count ||
      urdf_zero_ticks_.size() != count || ticks_per_radian_.size() != count || directions_.size() != count ||
      min_ticks_.size() != count || max_ticks_.size() != count)
  {
    throw std::invalid_argument("all joint calibration arrays must have the same non-zero size");
  }
  const auto configured_speed = get_parameter("goal_speed").as_int();
  const auto configured_acceleration = get_parameter("goal_acceleration").as_int();
  const auto configured_tolerance = get_parameter("position_tolerance_ticks").as_int();
  const auto stationary_tolerance = get_parameter("stationary_tolerance_ticks").as_int();
  const auto stationary_duration = get_parameter("stationary_duration_ms").as_int();
  const auto stationary_min_wait = get_parameter("stationary_min_wait_ms").as_int();
  const auto feedback_poll_period_ms = get_parameter("feedback_poll_period_ms").as_int();
  feedback_timeout_ms_ = static_cast<int>(get_parameter("feedback_timeout_ms").as_int());
  allow_motion_ = get_parameter("allow_motion").as_bool();
  require_all_feedback_for_joint_states_ = get_parameter("require_all_feedback_for_joint_states").as_bool();
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
  if (stationary_tolerance < 0 || stationary_tolerance > 100 ||
      stationary_duration < 100 || stationary_duration > 10000 ||
      stationary_min_wait < 100 || stationary_min_wait > 30000)
  {
    throw std::invalid_argument("invalid stationary tolerance, duration, or minimum wait");
  }
  stationary_tolerance_ticks_ = static_cast<int>(stationary_tolerance);
  stationary_duration_ms_ = static_cast<int>(stationary_duration);
  stationary_min_wait_ms_ = static_cast<int>(stationary_min_wait);
  last_ticks_.reserve(count);
  joint_calibrations_.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    if (!joint_index_.emplace(joint_names_[i], i).second || servo_ids_[i] < 0 || servo_ids_[i] > 253 ||
        (servo_protocols_[i] != "hx" && servo_protocols_[i] != "v3") ||
        ticks_per_radian_[i] <= 0.0 || std::abs(directions_[i]) != 1.0 || min_ticks_[i] < -30719 ||
        max_ticks_[i] > 30719 || min_ticks_[i] > max_ticks_[i])
    {
      throw std::invalid_argument("invalid HX calibration parameter at index " + std::to_string(i));
    }
    joint_calibrations_.emplace_back(
      zero_ticks_[i], urdf_zero_ticks_[i], ticks_per_radian_[i], directions_[i]);
    if (urdf_zero_ticks_[i] < min_ticks_[i] || urdf_zero_ticks_[i] > max_ticks_[i])
    {
      throw std::invalid_argument("urdf_zero_ticks is outside joint limits at index " + std::to_string(i));
    }
    const auto zero = std::lround(zero_ticks_[i]);
    if (zero < min_ticks_[i] || zero > max_ticks_[i])
    {
      throw std::invalid_argument("zero_ticks is outside joint limits at index " + std::to_string(i));
    }
    last_ticks_.push_back(static_cast<int16_t>(zero));
    RCLCPP_INFO(
      get_logger(), "%s calibration: native zero=%.1f ticks, URDF zero=%.1f ticks, direction=%.0f",
      joint_names_[i].c_str(), zero_ticks_[i], urdf_zero_ticks_[i], directions_[i]);
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
  if (feedback_poll_period_ms > 0)
  {
    feedback_timer_ = create_wall_timer(
      std::chrono::milliseconds(feedback_poll_period_ms),
      std::bind(&HxArmDriverNode::poll_feedback, this));
    RCLCPP_INFO(
      get_logger(), "HX feedback polling enabled every %ld ms while idle",
      feedback_poll_period_ms);
  }
  RCLCPP_INFO(
    get_logger(), "opened %s at %d baud for %zu HX servos; motion=%s",
    device.c_str(), static_cast<int>(get_parameter("baud_rate").as_int()), count,
    allow_motion_ ? "enabled" : "disabled");
}

void HxArmDriverNode::poll_feedback()
{
  std::unique_lock<std::mutex> activity_lock(activity_mutex_, std::try_to_lock);
  if (!activity_lock.owns_lock() || moving_.load())
  {
    return;
  }

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
  std::lock_guard<std::mutex> activity_lock(activity_mutex_);
  if (moving_.load())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

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
  std::lock_guard<std::mutex> activity_lock(activity_mutex_);
  if (moving_.load())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

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

  int64_t previous_ns = 0;
  size_t point_index = 0;
  for (const auto &point : trajectory.points)
  {
    if (point.positions.size() != trajectory.joint_names.size())
    {
      reason = "each trajectory point must have one position per joint";
      return false;
    }
    const auto joint_count = trajectory.joint_names.size();
    if ((!point.velocities.empty() && point.velocities.size() != joint_count) ||
      (!point.accelerations.empty() && point.accelerations.size() != joint_count) ||
      (!point.effort.empty() && point.effort.size() != joint_count))
    {
      reason = "non-empty trajectory fields must match joint_names";
      return false;
    }
    if (!point.effort.empty())
    {
      reason = "effort trajectories are unsupported by the servo driver";
      return false;
    }
    // Servo speed remains controlled by goal_speed_; MoveIt derivatives are not tracked.
    for (const auto value : point.velocities)
    {
      if (!std::isfinite(value))
      {
        reason = "trajectory contains a non-finite velocity";
        return false;
      }
    }
    for (const auto value : point.accelerations)
    {
      if (!std::isfinite(value))
      {
        reason = "trajectory contains a non-finite acceleration";
        return false;
      }
    }
    const int64_t point_ns = static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
      point.time_from_start.nanosec;
    const bool zero_time_start = point_index == 0 && point_ns == 0;
    if (point_ns < 0 || point_ns > 60000000000LL ||
      (!zero_time_start && point_ns <= previous_ns))
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
        joint_calibrations_[index].urdf_to_ticks(point.positions[i]));
      if (raw < min_ticks_[index] || raw > max_ticks_[index])
      {
        reason = "trajectory exceeds calibrated limit for " + trajectory.joint_names[i];
        return false;
      }
    }
    previous_ns = point_ns;
    ++point_index;
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
    const auto requested_goal_tolerance_ms =
      static_cast<int64_t>(goal->goal_time_tolerance.sec) * 1000 +
      goal->goal_time_tolerance.nanosec / 1000000;
    const auto grace_ms = requested_goal_tolerance_ms > 0 ? requested_goal_tolerance_ms : 5000;
    int64_t previous_ns = 0;
    bool first_point = true;
    const auto cancelled = [&]() {
        if (!goal_handle->is_canceling()) {
          return false;
        }
        emergency_stop();
        fail(FollowJointTrajectory::Result::INVALID_GOAL, "trajectory cancelled; stop sent");
        goal_handle->canceled(result);
        moving_.store(false);
        return true;
      };
    const auto publish_feedback = [&](const auto &point, const bool enforce_tolerance) {
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
          const auto actual = joint_calibrations_[index].ticks_to_urdf(last_ticks_[index]);
          feedback->actual.positions.push_back(actual);
          feedback->error.positions[i] = point.positions[i] - actual;
          if (enforce_tolerance)
          {
            const auto target_ticks = std::lround(
              joint_calibrations_[index].urdf_to_ticks(point.positions[i]));
            // if (std::abs(static_cast<int>(last_ticks_[index]) - static_cast<int>(target_ticks)) >
            //   position_tolerance_ticks_)
            // {
            //   within_tolerance = false;
            // }
            // joint3 允许 30 tick，其余关节使用 YAML 中的公共容差。
            const int tolerance_ticks =
              trajectory.joint_names[i] == "joint3"
              ? 30
              : position_tolerance_ticks_;

            if (std::abs(static_cast<int>(last_ticks_[index]) - static_cast<int>(target_ticks)) >
              tolerance_ticks)
            {
              within_tolerance = false;
            }
          }
        }
        publish_state(last_ticks_);
        goal_handle->publish_feedback(feedback);
        return within_tolerance;
      };

    auto next_feedback_time = std::chrono::steady_clock::now();
    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index)
    {
      const auto &point = trajectory.points[point_index];
      if (cancelled())
      {
        return;
      }

      const int64_t point_ns = static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
        point.time_from_start.nanosec;
      const bool final_point = point_index + 1 == trajectory.points.size();
      if (first_point && point_ns == 0 && !final_point)
      {
        previous_ns = point_ns;
        first_point = false;
        continue;
      }
      first_point = false;
      const auto segment_ms = static_cast<uint16_t>(std::clamp<int64_t>(
          (point_ns - previous_ns + 999999) / 1000000, 20, 30000));
      std::vector<HxPositionTarget> hx_targets;
      hx_targets.reserve(trajectory.joint_names.size());

      for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
      {
        const auto index = joint_index_.at(trajectory.joint_names[i]);
        const auto target = std::lround(
          joint_calibrations_[index].urdf_to_ticks(point.positions[i]));
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
      if (!hx_targets.empty())
      {
        bus_->send(encode_hx_sync_position_command(hx_targets));
        bus_->send(encode_hx_action_command());
      }

      const auto target_time = start + std::chrono::nanoseconds(point_ns);
      if (!final_point)
      {
        while (std::chrono::steady_clock::now() < target_time)
        {
          if (cancelled())
          {
            return;
          }
          const auto current_time = std::chrono::steady_clock::now();
          if (current_time >= next_feedback_time)
          {
            // Intermediate points are streamed by time; feedback is published
            // without requiring the servos to settle at every sample.
            publish_feedback(point, false);
            next_feedback_time = current_time + 50ms;
          }
          std::this_thread::sleep_for(5ms);
        }
        previous_ns = point_ns;
        continue;
      }

      const auto final_wait_start = std::max(target_time, std::chrono::steady_clock::now());
      const auto deadline = final_wait_start + std::chrono::milliseconds(grace_ms);
      while (std::chrono::steady_clock::now() < final_wait_start)
      {
        if (cancelled())
        {
          return;
        }
        const auto current_time = std::chrono::steady_clock::now();
        if (current_time >= next_feedback_time)
        {
          publish_feedback(point, false);
          next_feedback_time = current_time + 50ms;
        }
        std::this_thread::sleep_for(5ms);
      }

      bool reached = false;
      while (std::chrono::steady_clock::now() < deadline)
      {
        if (cancelled())
        {
          return;
        }
        reached = publish_feedback(point, true);
        if (reached)
        {
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
      if (!reached)
      {
        std::ostringstream error_message;
        error_message << "final servos did not reach the goal within " << grace_ms << " ms;";
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
        {
          const auto index = joint_index_.at(trajectory.joint_names[i]);
          const auto target_ticks = std::lround(
            joint_calibrations_[index].urdf_to_ticks(point.positions[i]));
          const auto error_ticks = static_cast<int>(target_ticks) - static_cast<int>(last_ticks_[index]);
          error_message << ' ' << trajectory.joint_names[i]
                        << " actual=" << last_ticks_[index]
                        << " target=" << target_ticks
                        << " error=" << error_ticks << " ticks;";
        }
        fail(FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED, error_message.str());
        goal_handle->abort(result);
        moving_.store(false);
        return;
      }
      previous_ns = point_ns;
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

bool HxArmDriverNode::validate_goal(const MoveJoints::Goal &goal, std::string &reason)
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
  if ((!goal.start_delays_ms.empty() && goal.start_delays_ms.size() != goal.joint_names.size()) ||
    (!goal.durations_ms.empty() && goal.durations_ms.size() != goal.joint_names.size()))
  {
    reason = "start_delays_ms and durations_ms must be empty or match joint_names";
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
    const auto index = joint_index_.at(goal.joint_names[i]);
    // An idle poll can lose one reply immediately before the home request.
    // Re-read only requested offline joints before rejecting; never command
    // motion for a joint that still has no valid position response.
    if (!online_[index] && !refresh_position(index))
    {
      reason = "requested servo is offline after 3 position reads: " + goal.joint_names[i] +
        " (ID " + std::to_string(servo_ids_[index]) + ")";
      return false;
    }
  }
  for (size_t i = 0; i < goal.joint_names.size(); ++i)
  {
    const auto index = joint_index_.at(goal.joint_names[i]);
    const auto delay_ms = goal.start_delays_ms.empty() ? 0U : goal.start_delays_ms[i];
    const auto travel_ms = goal.durations_ms.empty() ? static_cast<uint32_t>(ms) :
      static_cast<uint32_t>(goal.durations_ms[i]);
    if (delay_ms >= static_cast<uint32_t>(ms) || travel_ms < 20U || travel_ms > 30000U ||
      (goal.wait_for_completion && servo_protocols_[index] == "v3" &&
      delay_ms + travel_ms > static_cast<uint32_t>(ms)))
    {
      reason = "per-joint delay/duration does not fit within the goal duration";
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
    // /move_joints retains native radians so an all-zero goal still means folded home.
    const auto target = std::lround(joint_calibrations_[index].native_to_ticks(goal.positions[i]));
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
    state.position[i] = joint_calibrations_[i].ticks_to_urdf(ticks[i]);
  }
  return state;
}

void HxArmDriverNode::publish_state(const std::vector<int16_t> &ticks)
{
  joint_state_pub_->publish(state_from_ticks(ticks));
}

bool HxArmDriverNode::refresh_position(const size_t index)
{
  for (unsigned attempt = 0; attempt < 3; ++attempt)
  {
    if (const auto observed = read_position(index); observed)
    {
      last_ticks_[index] = *observed;
      online_[index] = true;
      return true;
    }
  }
  online_[index] = false;
  return false;
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
    throw std::runtime_error(
      "no status reply while " + operation +
      " servo ID " + std::to_string(id));
  }

  if ((*reply)[4] != 0U)
  {
    throw std::runtime_error(
      "HX error " + std::to_string((*reply)[4]) +
      " while " + operation +
      " servo ID " + std::to_string(id));
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
  const auto cancel_requested = [&]() {
      if (!goal_handle->is_canceling()) {return false;}
      if (goal->hold_on_cancel) {
        try {
          hold_requested_joints(*goal);
        } catch (...) {
          emergency_stop();
          throw;
        }
      } else {
        emergency_stop();
      }
      result->success = false;
      result->message = goal->hold_on_cancel ?
        "motion cancelled; requested joints held for recovery" :
        "motion cancelled and torque disabled";
      result->final_state = state_from_ticks(last_ticks_);
      // Release the busy flag before reporting terminal cancellation: a
      // recovery client may submit its home goal immediately on that result.
      moving_.store(false);
      goal_handle->canceled(result);
      return true;
    };

  try {
    const auto ticks = goal_to_ticks(*goal);
    const auto duration_ms = static_cast<uint16_t>(
      std::clamp<int64_t>(static_cast<int64_t>(goal->duration.sec) * 1000 + goal->duration.nanosec / 1000000, 1, 30000));
    // Validate every V3 target before sending any joint command.
    for (const auto &name : goal->joint_names)
    {
      const auto index = joint_index_.at(name);
      if (servo_protocols_[index] == "v3" && (ticks[index] < 0 || ticks[index] > 1000))
      {
        throw std::out_of_range("V3 target position must be 0..1000 for " + name);
      }
    }
    // Use the same immediate HX WRITE as single-joint control, without
    // depending on REG_WRITE/broadcast ACTION firmware support.
    auto stationary_not_before = std::chrono::steady_clock::now();
    {
      const auto dispatch_start = std::chrono::steady_clock::now();
      std::vector<uint16_t> delays(goal->joint_names.size(), 0U);
      std::vector<uint16_t> travel_times(goal->joint_names.size(), duration_ms);
      if (!goal->start_delays_ms.empty())
      {
        delays = goal->start_delays_ms;
      }
      if (!goal->durations_ms.empty())
      {
        travel_times = goal->durations_ms;
      }
      auto dispatch_times = delays;
      std::sort(dispatch_times.begin(), dispatch_times.end());
      dispatch_times.erase(std::unique(dispatch_times.begin(), dispatch_times.end()), dispatch_times.end());
      for (const auto delay_ms : dispatch_times)
      {
        const auto dispatch_at = dispatch_start + std::chrono::milliseconds(delay_ms);
        while (std::chrono::steady_clock::now() < dispatch_at)
        {
          if (cancel_requested()) {return;}
          std::this_thread::sleep_for(5ms);
        }
        for (size_t i = 0; i < goal->joint_names.size(); ++i)
        {
          if (cancel_requested()) {return;}
          if (delays[i] != delay_ms)
          {
            continue;
          }
          const auto index = joint_index_.at(goal->joint_names[i]);
          const auto id = static_cast<uint8_t>(servo_ids_[index]);
          if (servo_protocols_[index] == "v3")
          {
            bus_->send(encode_v3_position_command(
              id, static_cast<uint16_t>(ticks[index]), travel_times[i]));
            // V3 uses a timed move. Quantized readings during a very slow move
            // can look stationary; wait out its commanded travel time first.
            if (std::abs(static_cast<int>(last_ticks_[index]) - ticks[index]) >
                stationary_tolerance_ticks_)
            {
              stationary_not_before = std::max(stationary_not_before,
                std::chrono::steady_clock::now() + std::chrono::milliseconds(travel_times[i]));
            }
          }
          else
          {
            const auto speed = goal->speeds.empty() ? goal_speed_ : goal->speeds[i];
            write_checked(
              id, encode_hx_position_command(id, ticks[index], speed, goal_acceleration_),
              "writing target position to");
          }
        }
      }
      RCLCPP_INFO(get_logger(), "position commands dispatched directly to %zu servos",
        goal->joint_names.size());
    }
    const auto duration = std::chrono::milliseconds(goal->duration.sec * 1000 + goal->duration.nanosec / 1000000);
    const auto start = std::chrono::steady_clock::now();
    stationary_not_before = std::max(stationary_not_before,
      start + std::chrono::milliseconds(stationary_min_wait_ms_));
    JointStationaryMonitor stationary_monitor(
      stationary_tolerance_ticks_, std::chrono::milliseconds(stationary_duration_ms_));
    bool completed_stationary = false;
    bool reached_target = !goal->wait_for_completion;
    std::vector<bool> feedback_received(servo_ids_.size(), false);
    while (goal->wait_for_completion && std::chrono::steady_clock::now() - start < duration)
    {
      if (cancel_requested()) {return;}
      bool all_within_tolerance = true;
      bool all_fresh = true;
      std::vector<int16_t> observed_positions;
      for (const auto &name : goal->joint_names)
      {
        const auto index = joint_index_.at(name);
        const auto observed = read_position(index);
        feedback_received[index] = observed.has_value();
        observed_positions.push_back(observed.value_or(last_ticks_[index]));
        if (!observed)
        {
          all_fresh = false;
          online_[index] = false;
          all_within_tolerance = false;
          continue;
        }
        last_ticks_[index] = *observed;
        online_[index] = true;
        if (std::abs(static_cast<int>(*observed) - static_cast<int>(ticks[index])) > position_tolerance_ticks_)
        {
          all_within_tolerance = false;
        }
      }
      auto feedback = std::make_shared<MoveJoints::Feedback>();
      feedback->progress = std::min(1.0F, static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()) / duration.count());
      feedback->current_state = state_from_ticks(last_ticks_);
      goal_handle->publish_feedback(feedback);
      publish_state(last_ticks_);
      const auto observed_at = std::chrono::steady_clock::now();
      const bool stationary = stationary_monitor.update(observed_positions, all_fresh, observed_at);
      if (cancel_requested()) {return;}
      const bool stationary_fallback = goal->complete_when_stationary && stationary &&
        observed_at >= stationary_not_before;
      if (all_within_tolerance || stationary_fallback)
      {
        completed_stationary = !all_within_tolerance;
        reached_target = true;
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
    if (goal->wait_for_completion && !reached_target)
    {
      publish_state(last_ticks_);
      result->success = false;
      std::ostringstream detail;
      detail << "motion timeout: ";
      if (goal->complete_when_stationary)
      {
        detail << "stationary range<=" << stationary_tolerance_ticks_ << " ticks for "
               << stationary_duration_ms_ << " ms not confirmed; ";
      }
      detail << "position tolerance=" << position_tolerance_ticks_ << " ticks;";
      for (const auto &name : goal->joint_names)
      {
        const auto index = joint_index_.at(name);
        const auto error_ticks = static_cast<int>(last_ticks_[index]) - static_cast<int>(ticks[index]);
        if (!feedback_received[index])
        {
          detail << " " << name << " no fresh feedback, last=" << last_ticks_[index]
                 << " target=" << ticks[index] << " ticks;";
        }
        else if (goal->complete_when_stationary || std::abs(error_ticks) > position_tolerance_ticks_)
        {
          detail << " " << name << " actual=" << last_ticks_[index]
                 << " target=" << ticks[index] << " error=" << error_ticks << " ticks;";
        }
      }
      result->message = detail.str();
      result->final_state = state_from_ticks(last_ticks_);
      moving_.store(false);
      goal_handle->abort(result);
      return;
    }
    if (!goal->wait_for_completion)
    {
      for (const auto &name : goal->joint_names)
      {
        const auto index = joint_index_.at(name);
        if (const auto observed = read_position(index); observed)
        {
          last_ticks_[index] = *observed;
          online_[index] = true;
        }
      }
    }
    publish_state(last_ticks_);
    result->success = true;
    result->message = goal->wait_for_completion ? "HX target reached" : "HX motion command sent";
    if (completed_stationary)
    {
      std::ostringstream detail;
      detail << "HX motion completed: requested joints stationary; range<="
             << stationary_tolerance_ticks_ << " ticks for " << stationary_duration_ms_
             << " ms; target errors:";
      for (const auto &name : goal->joint_names)
      {
        const auto index = joint_index_.at(name);
        detail << " " << name << "=" << static_cast<int>(last_ticks_[index]) - ticks[index]
               << " ticks;";
      }
      result->message = detail.str();
      RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    }
    result->final_state = state_from_ticks(last_ticks_);
    moving_.store(false);
    goal_handle->succeed(result);
  }
  catch (const std::exception &error) {result->success = false;
                                       result->message = error.what();
                                       result->final_state = state_from_ticks(last_ticks_);
                                       moving_.store(false);
                                       goal_handle->abort(result);
  }
}

void HxArmDriverNode::hold_requested_joints(const MoveJoints::Goal &goal)
{
  for (const auto &name : goal.joint_names)
  {
    const auto index = joint_index_.at(name);
    const auto id = static_cast<uint8_t>(servo_ids_[index]);
    if (servo_protocols_[index] == "v3")
    {
      bus_->send(encode_v3_stop_command(id));
      continue;
    }
    if (!refresh_position(index))
    {
      throw std::runtime_error("cannot hold " + name + ": no fresh position feedback");
    }
    write_checked(id, encode_hx_position_command(id, last_ticks_[index], goal_speed_, goal_acceleration_),
      "holding cancelled motion for");
  }
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
