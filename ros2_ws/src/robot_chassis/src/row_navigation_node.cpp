#include "robot_chassis/row_navigation_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace robot_chassis
{
namespace
{
constexpr uint8_t kReachedEnd = 0;
constexpr uint8_t kSensorFault = 1;
constexpr uint8_t kCanceled = 2;
constexpr uint8_t kSafetyLimit = 3;
constexpr uint8_t kTimeout = 4;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

RowNavigationNode::RowNavigationNode() : Node("row_navigation_node")
{
  declare_parameter("loop_hz", 20.0);
  declare_parameter("sensor_timeout_s", 0.3);
  declare_parameter("side_kp", 1.2);
  declare_parameter("yaw_kp", 1.0);
  declare_parameter("max_lateral_speed_mps", 0.15);
  declare_parameter("max_yaw_rate_rad_s", 0.4);
  declare_parameter("stop_speed_epsilon_mps", 0.03);
  declare_parameter("stop_angular_epsilon_rad_s", 0.08);
  declare_parameter("stop_settle_s", 0.2);
  declare_parameter("default_front_slow_m", 0.8);
  declare_parameter("default_front_stop_m", 0.35);
  declare_parameter("default_safety_distance_m", 0.12);

  loop_hz_ = get_parameter("loop_hz").as_double();
  sensor_timeout_s_ = get_parameter("sensor_timeout_s").as_double();
  side_kp_ = get_parameter("side_kp").as_double();
  yaw_kp_ = get_parameter("yaw_kp").as_double();
  max_lateral_speed_mps_ = get_parameter("max_lateral_speed_mps").as_double();
  max_yaw_rate_rad_s_ = get_parameter("max_yaw_rate_rad_s").as_double();
  stop_speed_epsilon_mps_ = get_parameter("stop_speed_epsilon_mps").as_double();
  stop_angular_epsilon_rad_s_ = get_parameter("stop_angular_epsilon_rad_s").as_double();
  stop_settle_s_ = get_parameter("stop_settle_s").as_double();
  default_front_slow_m_ = get_parameter("default_front_slow_m").as_double();
  default_front_stop_m_ = get_parameter("default_front_stop_m").as_double();
  default_safety_distance_m_ = get_parameter("default_safety_distance_m").as_double();

  if (loop_hz_ <= 0.0 || sensor_timeout_s_ <= 0.0 || max_lateral_speed_mps_ <= 0.0 ||
      max_yaw_rate_rad_s_ <= 0.0 || stop_settle_s_ < 0.0)
  {
    throw std::invalid_argument("invalid row navigation configuration");
  }

  ultrasonic_sub_ = create_subscription<robot_arm_interfaces::msg::UltrasonicArray>(
    "/base/ultrasonic", rclcpp::SensorDataQoS(),
    [this](const robot_arm_interfaces::msg::UltrasonicArray::SharedPtr message) {
      ultrasonic_callback(message);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/base/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) { odom_callback(message); });
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/base/cmd_vel", rclcpp::QoS(10));

  follow_server_ = rclcpp_action::create_server<FollowRow>(
    this, "follow_row",
    std::bind(&RowNavigationNode::follow_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RowNavigationNode::follow_cancel, this, std::placeholders::_1),
    std::bind(&RowNavigationNode::follow_accepted, this, std::placeholders::_1));
  shift_server_ = rclcpp_action::create_server<ShiftLane>(
    this, "shift_lane",
    std::bind(&RowNavigationNode::shift_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RowNavigationNode::shift_cancel, this, std::placeholders::_1),
    std::bind(&RowNavigationNode::shift_accepted, this, std::placeholders::_1));
  stop_server_ = rclcpp_action::create_server<StopBase>(
    this, "stop_base",
    std::bind(&RowNavigationNode::stop_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RowNavigationNode::stop_cancel, this, std::placeholders::_1),
    std::bind(&RowNavigationNode::stop_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "row navigation ready: follow_row, shift_lane, stop_base");
}

void RowNavigationNode::ultrasonic_callback(
  const robot_arm_interfaces::msg::UltrasonicArray::SharedPtr message)
{
  std::scoped_lock lock(sensor_mutex_);
  sensors_.ultrasonic = *message;
  sensors_.have_ultrasonic = true;
  sensors_.ultrasonic_time = std::chrono::steady_clock::now();
}

void RowNavigationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  std::scoped_lock lock(sensor_mutex_);
  sensors_.odom = *message;
  sensors_.have_odom = true;
  sensors_.odom_time = std::chrono::steady_clock::now();
}

RowNavigationNode::Sensors RowNavigationNode::sensors() const
{
  std::scoped_lock lock(sensor_mutex_);
  return sensors_;
}

bool RowNavigationNode::fresh(const Sensors &snapshot, const bool need_odom) const
{
  const auto now = std::chrono::steady_clock::now();
  if (!snapshot.have_ultrasonic ||
      std::chrono::duration<double>(now - snapshot.ultrasonic_time).count() > sensor_timeout_s_)
  {
    return false;
  }
  if (need_odom && (!snapshot.have_odom ||
      std::chrono::duration<double>(now - snapshot.odom_time).count() > sensor_timeout_s_))
  {
    return false;
  }
  return true;
}

void RowNavigationNode::publish_command(const geometry_msgs::msg::Twist &command)
{
  command_pub_->publish(command);
}

void RowNavigationNode::publish_zero()
{
  command_pub_->publish(geometry_msgs::msg::Twist());
}

void RowNavigationNode::set_phase(const RoutePhase phase)
{
  std::scoped_lock lock(phase_mutex_);
  phase_ = phase;
}

double RowNavigationNode::clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

double RowNavigationNode::speed_scale(
  const double distance, const double slow_distance, const double stop_distance)
{
  if (distance <= stop_distance)
  {
    return 0.0;
  }
  if (distance >= slow_distance)
  {
    return 1.0;
  }
  return clamp((distance - stop_distance) / (slow_distance - stop_distance), 0.0, 1.0);
}

double RowNavigationNode::normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double RowNavigationNode::planar_lateral_displacement(
  const nav_msgs::msg::Odometry &start, const nav_msgs::msg::Odometry &current)
{
  const auto start_yaw = yaw_from_quaternion(start.pose.pose.orientation);
  const auto dx = current.pose.pose.position.x - start.pose.pose.position.x;
  const auto dy = current.pose.pose.position.y - start.pose.pose.position.y;
  return -std::sin(start_yaw) * dx + std::cos(start_yaw) * dy;
}

rclcpp_action::GoalResponse RowNavigationNode::follow_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowRow::Goal> goal)
{
  if (goal->direction != 1 && goal->direction != -1 || goal->max_speed_mps <= 0.0 ||
      goal->max_speed_mps > 1.0 || goal->front_slow_distance_m <= goal->front_stop_distance_m ||
      goal->front_stop_distance_m <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "rejecting invalid FollowRow goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true))
  {
    RCLCPP_WARN(get_logger(), "rejecting FollowRow: another base action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  active_row_id_ = goal->row_id;
  active_direction_ = goal->direction;
  set_phase(RoutePhase::FollowRow);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RowNavigationNode::follow_cancel(
  const std::shared_ptr<FollowHandle> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RowNavigationNode::follow_accepted(const std::shared_ptr<FollowHandle> &handle)
{
  std::thread{[this, handle]() { execute_follow(handle); }}.detach();
}

void RowNavigationNode::execute_follow(const std::shared_ptr<FollowHandle> &handle)
{
  auto result = std::make_shared<FollowRow::Result>();
  const auto goal = handle->get_goal();
  const auto initial = sensors();
  const auto target_yaw = initial.have_odom ? yaw_from_quaternion(initial.odom.pose.pose.orientation) : 0.0;
  const auto slow_distance = goal->front_slow_distance_m > 0.0 ?
    goal->front_slow_distance_m : default_front_slow_m_;
  const auto stop_distance = goal->front_stop_distance_m > 0.0 ?
    goal->front_stop_distance_m : default_front_stop_m_;
  const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
  auto start = std::chrono::steady_clock::now();
  const auto start_odom = initial.odom;

  try
  {
    while (rclcpp::ok())
    {
      if (handle->is_canceling())
      {
        set_phase(RoutePhase::Braking);
        publish_zero();
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_settle_s_));
        set_phase(RoutePhase::Paused);
        result->success = false;
        result->reason = kCanceled;
        result->message = "row motion canceled and safely stopped; route state retained";
        if (initial.have_odom && sensors().have_odom)
        {
          result->traveled_m = std::hypot(
            sensors().odom.pose.pose.position.x - start_odom.pose.pose.position.x,
            sensors().odom.pose.pose.position.y - start_odom.pose.pose.position.y);
        }
        handle->canceled(result);
        active_.store(false);
        return;
      }

      const auto snapshot = sensors();
      const bool use_front = goal->direction > 0;
      const bool distance_valid = use_front ? snapshot.ultrasonic.front_valid : snapshot.ultrasonic.rear_valid;
      const double distance = use_front ? snapshot.ultrasonic.front_m : snapshot.ultrasonic.rear_m;
      if (!fresh(snapshot, false) || !distance_valid || !std::isfinite(distance))
      {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = "front/rear ultrasonic data is stale or invalid";
        handle->abort(result);
        active_.store(false);
        return;
      }
      if (goal->use_side_centering && !snapshot.ultrasonic.left_valid &&
          !snapshot.ultrasonic.right_valid)
      {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = "side ultrasonic data is unavailable";
        handle->abort(result);
        active_.store(false);
        return;
      }

      const auto scale = speed_scale(distance, slow_distance, stop_distance);
      if (scale <= 0.0)
      {
        set_phase(RoutePhase::Braking);
        publish_zero();
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_settle_s_));
        set_phase(RoutePhase::Stopped);
        result->success = true;
        result->reason = kReachedEnd;
        result->message = "row end reached and base stopped";
        if (initial.have_odom && sensors().have_odom)
        {
          result->traveled_m = std::hypot(
            sensors().odom.pose.pose.position.x - start_odom.pose.pose.position.x,
            sensors().odom.pose.pose.position.y - start_odom.pose.pose.position.y);
        }
        handle->succeed(result);
        active_.store(false);
        return;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = static_cast<double>(goal->direction) * goal->max_speed_mps * scale;
      if (goal->use_side_centering && scale > 0.85)
      {
        double side_error = 0.0;
        if (snapshot.ultrasonic.left_valid && snapshot.ultrasonic.right_valid)
        {
          side_error = (snapshot.ultrasonic.left_m - snapshot.ultrasonic.right_m) * 0.5;
        }
        else if (snapshot.ultrasonic.left_valid)
        {
          side_error = snapshot.ultrasonic.left_m - goal->side_target_m;
        }
        else
        {
          side_error = goal->side_target_m - snapshot.ultrasonic.right_m;
        }
        command.linear.y = clamp(side_kp_ * side_error, -max_lateral_speed_mps_, max_lateral_speed_mps_);
      }
      if (snapshot.have_odom)
      {
        const auto yaw = yaw_from_quaternion(snapshot.odom.pose.pose.orientation);
        command.angular.z = clamp(
          yaw_kp_ * normalize_angle(target_yaw - yaw), -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);
      }
      publish_command(command);
      set_phase(scale < 1.0 ? RoutePhase::Braking : RoutePhase::FollowRow);

      auto feedback = std::make_shared<FollowRow::Feedback>();
      feedback->progress = 0.0F;
      feedback->phase = scale < 1.0 ? "braking" : "following_row";
      feedback->front_distance_m = snapshot.ultrasonic.front_m;
      feedback->left_distance_m = snapshot.ultrasonic.left_m;
      feedback->right_distance_m = snapshot.ultrasonic.right_m;
      feedback->command = command;
      handle->publish_feedback(feedback);
      start += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(start);
    }
  }
  catch (const std::exception &error)
  {
    publish_zero();
    set_phase(RoutePhase::Fault);
    result->success = false;
    result->reason = kSensorFault;
    result->message = error.what();
    handle->abort(result);
    active_.store(false);
  }
}

rclcpp_action::GoalResponse RowNavigationNode::shift_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ShiftLane::Goal> goal)
{
  if ((goal->direction != 1 && goal->direction != -1) || goal->distance_m <= 0.0 ||
      goal->distance_m > 5.0 || goal->max_speed_mps <= 0.0 || goal->max_speed_mps > 0.5)
  {
    RCLCPP_WARN(get_logger(), "rejecting invalid ShiftLane goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true))
  {
    RCLCPP_WARN(get_logger(), "rejecting ShiftLane: another base action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  set_phase(RoutePhase::ShiftLane);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RowNavigationNode::shift_cancel(
  const std::shared_ptr<ShiftHandle> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RowNavigationNode::shift_accepted(const std::shared_ptr<ShiftHandle> &handle)
{
  std::thread{[this, handle]() { execute_shift(handle); }}.detach();
}

void RowNavigationNode::execute_shift(const std::shared_ptr<ShiftHandle> &handle)
{
  auto result = std::make_shared<ShiftLane::Result>();
  const auto goal = handle->get_goal();
  const auto initial = sensors();
  const auto start_odom = initial.odom;
  const auto safety = goal->safety_distance_m > 0.0 ?
    goal->safety_distance_m : default_safety_distance_m_;
  const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
  auto next = std::chrono::steady_clock::now();

  try
  {
    while (rclcpp::ok())
    {
      if (handle->is_canceling())
      {
        set_phase(RoutePhase::ShiftBraking);
        publish_zero();
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_settle_s_));
        set_phase(RoutePhase::Paused);
        result->success = false;
        result->reason = kCanceled;
        result->message = "lane shift canceled and safely stopped";
        handle->canceled(result);
        active_.store(false);
        return;
      }

      const auto snapshot = sensors();
      if (!fresh(snapshot, true))
      {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = "odom or ultrasonic data is stale during lane shift";
        handle->abort(result);
        active_.store(false);
        return;
      }

      const bool boundary_valid = goal->direction > 0 ? snapshot.ultrasonic.left_valid :
        snapshot.ultrasonic.right_valid;
      if (!boundary_valid)
      {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = "the lateral boundary ultrasonic sensor is unavailable";
        handle->abort(result);
        active_.store(false);
        return;
      }

      if (goal->direction > 0 && snapshot.ultrasonic.left_valid &&
          snapshot.ultrasonic.left_m <= safety)
      {
        publish_zero();
        result->success = false;
        result->reason = kSafetyLimit;
        result->message = "left ultrasonic safety limit reached";
        set_phase(RoutePhase::Fault);
        handle->abort(result);
        active_.store(false);
        return;
      }
      if (goal->direction < 0 && snapshot.ultrasonic.right_valid &&
          snapshot.ultrasonic.right_m <= safety)
      {
        publish_zero();
        result->success = false;
        result->reason = kSafetyLimit;
        result->message = "right ultrasonic safety limit reached";
        set_phase(RoutePhase::Fault);
        handle->abort(result);
        active_.store(false);
        return;
      }

      const auto signed_displacement = planar_lateral_displacement(start_odom, snapshot.odom);
      const auto traveled = std::max(0.0, static_cast<double>(goal->direction) * signed_displacement);
      const auto remaining = goal->distance_m - traveled;
      if (remaining <= 0.02)
      {
        set_phase(RoutePhase::ShiftBraking);
        publish_zero();
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_settle_s_));
        set_phase(RoutePhase::Stopped);
        result->success = true;
        result->reason = kReachedEnd;
        result->message = "lane shift completed";
        result->traveled_m = traveled;
        handle->succeed(result);
        active_.store(false);
        return;
      }

      const auto speed_scale_value = clamp(remaining / 0.25, 0.2, 1.0);
      geometry_msgs::msg::Twist command;
      command.linear.y = static_cast<double>(goal->direction) * goal->max_speed_mps * speed_scale_value;
      publish_command(command);
      set_phase(speed_scale_value < 1.0 ? RoutePhase::ShiftBraking : RoutePhase::ShiftLane);

      auto feedback = std::make_shared<ShiftLane::Feedback>();
      feedback->progress = static_cast<float>(clamp(traveled / goal->distance_m, 0.0, 1.0));
      feedback->phase = speed_scale_value < 1.0 ? "braking" : "shifting_lane";
      feedback->left_distance_m = snapshot.ultrasonic.left_m;
      feedback->right_distance_m = snapshot.ultrasonic.right_m;
      feedback->command = command;
      handle->publish_feedback(feedback);
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }
  catch (const std::exception &error)
  {
    publish_zero();
    set_phase(RoutePhase::Fault);
    result->success = false;
    result->reason = kSensorFault;
    result->message = error.what();
    handle->abort(result);
    active_.store(false);
  }
}

rclcpp_action::GoalResponse RowNavigationNode::stop_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const StopBase::Goal>)
{
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  set_phase(RoutePhase::Braking);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RowNavigationNode::stop_cancel(
  const std::shared_ptr<StopHandle> &)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RowNavigationNode::stop_accepted(const std::shared_ptr<StopHandle> &handle)
{
  std::thread{[this, handle]() { execute_stop(handle); }}.detach();
}

void RowNavigationNode::execute_stop(const std::shared_ptr<StopHandle> &handle)
{
  auto result = std::make_shared<StopBase::Result>();
  const auto goal = handle->get_goal();
  const auto timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : 2.0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  const auto period = 50ms;
  auto stopped_since = std::chrono::steady_clock::time_point{};

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (handle->is_canceling())
    {
      publish_zero();
      result->success = false;
      result->message = "stop request canceled";
      handle->canceled(result);
      active_.store(false);
      return;
    }
    publish_zero();
    const auto snapshot = sensors();
    const bool stopped = snapshot.have_odom &&
      std::abs(snapshot.odom.twist.twist.linear.x) < stop_speed_epsilon_mps_ &&
      std::abs(snapshot.odom.twist.twist.linear.y) < stop_speed_epsilon_mps_ &&
      std::abs(snapshot.odom.twist.twist.angular.z) < stop_angular_epsilon_rad_s_;
    if (stopped)
    {
      if (stopped_since == std::chrono::steady_clock::time_point{})
      {
        stopped_since = std::chrono::steady_clock::now();
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - stopped_since).count() >= stop_settle_s_)
      {
        set_phase(RoutePhase::Stopped);
        result->success = true;
        result->message = "base stopped";
        handle->succeed(result);
        active_.store(false);
        return;
      }
    }
    else
    {
      stopped_since = std::chrono::steady_clock::time_point{};
    }
    auto feedback = std::make_shared<StopBase::Feedback>();
    feedback->progress = static_cast<float>(clamp(
      1.0 - std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count() / timeout_s,
      0.0, 1.0));
    feedback->stopped = stopped;
    handle->publish_feedback(feedback);
    std::this_thread::sleep_for(period);
  }

  publish_zero();
  set_phase(RoutePhase::Fault);
  result->success = false;
  result->message = "base did not report zero velocity before timeout";
  handle->abort(result);
  active_.store(false);
}
}  // namespace robot_chassis

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_chassis::RowNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
