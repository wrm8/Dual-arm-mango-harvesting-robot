#include "robot_chassis/row_navigation_node.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <stdexcept>
#include <thread>
#include <vector>
#include <array>

using namespace std::chrono_literals;

namespace robot_chassis
{
namespace
{
constexpr uint8_t kReachedEnd = 0;
constexpr uint8_t kSensorFault = 1;
constexpr uint8_t kCanceled = 2;
constexpr uint8_t kSafetyLimit = 3;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double median_filter_update(
  std::deque<double> &history, const double raw, const int requested_window)
{
  const auto window = static_cast<std::size_t>(
    std::max(1, std::min(requested_window, 15)));
  if (std::isfinite(raw)) {
    history.push_back(raw);
  }
  while (history.size() > window) {
    history.pop_front();
  }
  if (history.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> sorted(history.begin(), history.end());
  std::sort(sorted.begin(), sorted.end());
  return sorted[sorted.size() / 2];
}

double slew_velocity(
  const double target, const double current, const double max_acceleration,
  const double max_deceleration, const double dt)
{
  const double delta = target - current;
  const bool same_direction = target * current >= 0.0;
  const bool increasing_magnitude = std::abs(target) > std::abs(current);
  const double rate =
    same_direction && increasing_magnitude ? max_acceleration : max_deceleration;
  const double max_delta = std::max(0.0, rate) * std::max(0.0, dt);
  return current + std::max(-max_delta, std::min(delta, max_delta));
}
}  // namespace

RowNavigationNode::RowNavigationNode() : Node("row_navigation_node")
{
  declare_parameter("loop_hz", 20.0);
  declare_parameter("sensor_timeout_s", 0.3);
  declare_parameter("side_kp", 1.2);
  declare_parameter("yaw_kp", 1.0);
  declare_parameter("yaw_ki", 0.05);
  declare_parameter("yaw_integral_limit", 0.5);
  declare_parameter("max_acceleration_mps2", 0.8);
  declare_parameter("max_deceleration_mps2", 0.8);
  declare_parameter("ultrasonic_filter_window", 3);
  declare_parameter("shift_wall_enable_tolerance_m", 0.10);
  declare_parameter("shift_wall_cancel_side_distance_m", 0.25);
  declare_parameter("shift_wall_cancel_side_tolerance_m", 0.02);
  declare_parameter("max_lateral_speed_mps", 0.15);
  declare_parameter("max_yaw_rate_rad_s", 0.4);
  declare_parameter("stop_speed_epsilon_mps", 0.03);
  declare_parameter("stop_angular_epsilon_rad_s", 0.08);
  declare_parameter("stop_settle_s", 0.2);
  declare_parameter("default_front_slow_m", 0.30);
  declare_parameter("default_front_stop_m", 0.05);
  declare_parameter("default_safety_distance_m", 0.05);
  declare_parameter("reverse_speed_mps", 0.20);
  declare_parameter("reverse_distance_kp", 2.0);
  declare_parameter("reverse_distance_m", 0.70);
  declare_parameter("reverse_distance_tolerance_m", 0.05);
  declare_parameter("reverse_slow_distance_m", 0.25);
  declare_parameter("shift_side_target_m", 0.05);
  declare_parameter("shift_side_slow_distance_m", 0.20);
  declare_parameter("shift_front_target_m", 0.05);
  declare_parameter("shift_front_kp", 1.0);
  declare_parameter("max_front_correction_mps", 0.08);
  declare_parameter("mango_brake_time_s", 0.35);
  declare_parameter("photoelectric_stop_timeout_s", 3.0);
  declare_parameter("vision_tree_kp", 1.0);
  declare_parameter("vision_tree_ki", 0.05);
  declare_parameter("vision_tree_integral_limit", 0.2);
  declare_parameter("vision_tree_max_speed_mps", 0.12);
  declare_parameter("vision_tree_center_tolerance_ratio", 0.02);
  declare_parameter("vision_stop_settle_s", 0.30);
  declare_parameter("vision_observation_timeout_s", 0.60);
  declare_parameter("vision_tree_search_distance_m", 0.35);
  declare_parameter("vision_tree_alignment_timeout_s", 15.0);
  declare_parameter("vision_tree_control_sign", 1.0);
  (void)vision_stop_parameters();  // Also validate YAML values at startup.

  loop_hz_ = get_parameter("loop_hz").as_double();
  sensor_timeout_s_ = get_parameter("sensor_timeout_s").as_double();
  side_kp_ = get_parameter("side_kp").as_double();
  yaw_kp_ = get_parameter("yaw_kp").as_double();
  yaw_ki_ = get_parameter("yaw_ki").as_double();
  yaw_integral_limit_rad_s_ = get_parameter("yaw_integral_limit").as_double();
  max_acceleration_mps2_ = get_parameter("max_acceleration_mps2").as_double();
  max_deceleration_mps2_ = get_parameter("max_deceleration_mps2").as_double();
  ultrasonic_filter_window_ = get_parameter("ultrasonic_filter_window").as_int();
  shift_wall_enable_tolerance_m_ =
    get_parameter("shift_wall_enable_tolerance_m").as_double();
  shift_wall_cancel_side_distance_m_ =
    get_parameter("shift_wall_cancel_side_distance_m").as_double();
  shift_wall_cancel_side_tolerance_m_ =
    get_parameter("shift_wall_cancel_side_tolerance_m").as_double();
  max_lateral_speed_mps_ = get_parameter("max_lateral_speed_mps").as_double();
  max_yaw_rate_rad_s_ = get_parameter("max_yaw_rate_rad_s").as_double();
  stop_speed_epsilon_mps_ = get_parameter("stop_speed_epsilon_mps").as_double();
  stop_angular_epsilon_rad_s_ = get_parameter("stop_angular_epsilon_rad_s").as_double();
  stop_settle_s_ = get_parameter("stop_settle_s").as_double();
  default_front_slow_m_ = get_parameter("default_front_slow_m").as_double();
  default_front_stop_m_ = get_parameter("default_front_stop_m").as_double();
  default_safety_distance_m_ = get_parameter("default_safety_distance_m").as_double();
  reverse_speed_mps = get_parameter("reverse_speed_mps").as_double();
  reverse_distance_kp_ = get_parameter("reverse_distance_kp").as_double();
  reverse_distance_m_ = get_parameter("reverse_distance_m").as_double();
  reverse_distance_tolerance_m_ = get_parameter("reverse_distance_tolerance_m").as_double();
  reverse_slow_distance_m_ = get_parameter("reverse_slow_distance_m").as_double();
  shift_side_target_m_ = get_parameter("shift_side_target_m").as_double();
  shift_side_slow_distance_m_ = get_parameter("shift_side_slow_distance_m").as_double();
  shift_front_target_m_ = get_parameter("shift_front_target_m").as_double();
  shift_front_kp_ = get_parameter("shift_front_kp").as_double();
  max_front_correction_mps_ = get_parameter("max_front_correction_mps").as_double();
  mango_brake_time_s_ = get_parameter("mango_brake_time_s").as_double();
  photoelectric_stop_timeout_s_ = get_parameter("photoelectric_stop_timeout_s").as_double();

  if (loop_hz_ <= 0.0 || sensor_timeout_s_ <= 0.0 || max_lateral_speed_mps_ <= 0.0 ||
      max_yaw_rate_rad_s_ <= 0.0 || stop_settle_s_ < 0.0 ||
      reverse_speed_mps <= 0.0 || reverse_speed_mps > 2.0 ||
      !std::isfinite(reverse_distance_kp_) || reverse_distance_kp_ <= 0.0 ||reverse_distance_m_ <= 0.0 ||
      reverse_distance_tolerance_m_ < 0.0 || reverse_distance_tolerance_m_ >= reverse_slow_distance_m_ ||
      shift_side_target_m_ <= 0.0 || shift_side_slow_distance_m_ <= shift_side_target_m_ ||
      !std::isfinite(yaw_kp_) || !std::isfinite(yaw_ki_) ||
      !std::isfinite(yaw_integral_limit_rad_s_) || yaw_integral_limit_rad_s_ <= 0.0 ||
      !std::isfinite(max_acceleration_mps2_) || max_acceleration_mps2_ <= 0.0 ||
      !std::isfinite(max_deceleration_mps2_) || max_deceleration_mps2_ <= 0.0 ||
      ultrasonic_filter_window_ < 1 || ultrasonic_filter_window_ > 15 ||
      !std::isfinite(shift_wall_enable_tolerance_m_) ||
      shift_wall_enable_tolerance_m_ < 0.0 ||
      !std::isfinite(shift_wall_cancel_side_distance_m_) ||
      shift_wall_cancel_side_distance_m_ < 0.0 ||
      !std::isfinite(shift_wall_cancel_side_tolerance_m_) ||
      shift_wall_cancel_side_tolerance_m_ < 0.0 ||
      !std::isfinite(photoelectric_stop_timeout_s_) || photoelectric_stop_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("invalid row navigation configuration");
  }

  parameter_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "";

      auto parameter_as_double = [](const rclcpp::Parameter &parameter, double &value) {
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value = parameter.as_double();
            return true;
          }
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = static_cast<double>(parameter.as_int());
            return true;
          }
          return false;
        };
      auto parameter_as_integer = [](const rclcpp::Parameter &parameter, int &value) {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            return false;
          }
          value = static_cast<int>(parameter.as_int());
          return true;
        };

      double kp;
      double ki;
      double integral_limit;
      double max_rate;
      double max_acceleration;
      double max_deceleration;
      double wall_enable_tolerance;
      double wall_cancel_side_distance;
      double wall_cancel_side_tolerance;
      int filter_window;
      {
        std::scoped_lock lock(control_mutex_);
        kp = yaw_kp_;
        ki = yaw_ki_;
        integral_limit = yaw_integral_limit_rad_s_;
        max_rate = max_yaw_rate_rad_s_;
        max_acceleration = max_acceleration_mps2_;
        max_deceleration = max_deceleration_mps2_;
        wall_enable_tolerance = shift_wall_enable_tolerance_m_;
        wall_cancel_side_distance = shift_wall_cancel_side_distance_m_;
        wall_cancel_side_tolerance = shift_wall_cancel_side_tolerance_m_;
        filter_window = ultrasonic_filter_window_;
      }

      for (const auto &parameter : parameters) {
        double value = 0.0;
        if (parameter.get_name().rfind("vision_", 0) == 0) {
          const auto &name = parameter.get_name();
          const bool nonnegative = name == "vision_tree_ki" || name == "vision_stop_settle_s";
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) ||
              (name == "vision_tree_control_sign" ? std::abs(value) != 1.0 :
               (nonnegative ? value < 0.0 : value <= 0.0)) ||
              (name == "vision_tree_center_tolerance_ratio" && value >= 0.5)) {
            result.successful = false;
            result.reason = "invalid vision stop parameter: " + name;
            return result;
          }
        } else if (parameter.get_name() == "yaw_kp") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value)) {
            result.successful = false;
            result.reason = "yaw_kp must be a finite number";
            return result;
          }
          kp = value;
        } else if (parameter.get_name() == "yaw_ki") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value)) {
            result.successful = false;
            result.reason = "yaw_ki must be a finite number";
            return result;
          }
          ki = value;
        } else if (parameter.get_name() == "yaw_integral_limit") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value <= 0.0) {
            result.successful = false;
            result.reason = "yaw_integral_limit must be positive";
            return result;
          }
          integral_limit = value;
        } else if (parameter.get_name() == "max_yaw_rate_rad_s") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value <= 0.0) {
            result.successful = false;
            result.reason = "max_yaw_rate_rad_s must be positive";
            return result;
          }
          max_rate = value;
        } else if (parameter.get_name() == "max_acceleration_mps2") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value <= 0.0) {
            result.successful = false;
            result.reason = "max_acceleration_mps2 must be positive";
            return result;
          }
          max_acceleration = value;
        } else if (parameter.get_name() == "max_deceleration_mps2") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value <= 0.0) {
            result.successful = false;
            result.reason = "max_deceleration_mps2 must be positive";
            return result;
          }
          max_deceleration = value;
        } else if (parameter.get_name() == "ultrasonic_filter_window") {
          int integer_value = 0;
          if (!parameter_as_integer(parameter, integer_value) ||
              integer_value < 1 || integer_value > 15) {
            result.successful = false;
            result.reason = "ultrasonic_filter_window must be an integer from 1 to 15";
            return result;
          }
          filter_window = integer_value;
        } else if (parameter.get_name() == "shift_wall_enable_tolerance_m") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value < 0.0) {
            result.successful = false;
            result.reason = "shift_wall_enable_tolerance_m must be non-negative";
            return result;
          }
          wall_enable_tolerance = value;
        } else if (parameter.get_name() == "shift_wall_cancel_side_distance_m") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value < 0.0) {
            result.successful = false;
            result.reason = "shift_wall_cancel_side_distance_m must be non-negative";
            return result;
          }
          wall_cancel_side_distance = value;
        } else if (parameter.get_name() == "shift_wall_cancel_side_tolerance_m") {
          if (!parameter_as_double(parameter, value) || !std::isfinite(value) || value < 0.0) {
            result.successful = false;
            result.reason = "shift_wall_cancel_side_tolerance_m must be non-negative";
            return result;
          }
          wall_cancel_side_tolerance = value;
        }
      }

      {
        std::scoped_lock lock(control_mutex_);
        yaw_kp_ = kp;
        yaw_ki_ = ki;
        yaw_integral_limit_rad_s_ = integral_limit;
        max_yaw_rate_rad_s_ = max_rate;
        max_acceleration_mps2_ = max_acceleration;
        max_deceleration_mps2_ = max_deceleration;
        ultrasonic_filter_window_ = filter_window;
        shift_wall_enable_tolerance_m_ = wall_enable_tolerance;
        shift_wall_cancel_side_distance_m_ = wall_cancel_side_distance;
        shift_wall_cancel_side_tolerance_m_ = wall_cancel_side_tolerance;
      }
      RCLCPP_INFO(
        get_logger(),
        "navigation parameters updated: yaw_kp=%.4f yaw_ki=%.4f accel=%.3f decel=%.3f filter=%d",
        kp, ki, max_acceleration, max_deceleration, filter_window);
      return result;
    });

  ultrasonic_sub_ = create_subscription<robot_arm_interfaces::msg::UltrasonicArray>(
    "/base/ultrasonic", rclcpp::SensorDataQoS(),
    [this](const robot_arm_interfaces::msg::UltrasonicArray::SharedPtr message) {
      ultrasonic_callback(message);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/base/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) { odom_callback(message); });
  vision_sub_ = create_subscription<robot_arm_interfaces::msg::VisionRowObservation>(
    "/vision/row_observation", rclcpp::SensorDataQoS(),
    [this](const robot_arm_interfaces::msg::VisionRowObservation::SharedPtr message) {
      std::scoped_lock lock(vision_mutex_);
      vision_observation_ = *message;
      vision_observation_time_ = std::chrono::steady_clock::now();
      have_vision_observation_ = true;
    });
  vision_state_pub_ = create_publisher<robot_arm_interfaces::msg::VisionStopState>(
    "/base/vision_stop_state", rclcpp::QoS(10));
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/base/cmd_vel", rclcpp::QoS(10));
  route_command_pub_ = create_publisher<robot_arm_interfaces::msg::RouteCommand>(
    "/base/route_command", rclcpp::QoS(10));
  stage_feedback_sub_ = create_subscription<robot_arm_interfaces::msg::StageFeedback>(
    "/base/stage_feedback", rclcpp::QoS(10),
    [this](const robot_arm_interfaces::msg::StageFeedback::SharedPtr message) {
      stage_feedback_callback(message);
    });

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

  RCLCPP_INFO(get_logger(),
    "row navigation ready: forward/reverse, side-wall shift, and stop_base");
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
  return !need_odom || (snapshot.have_odom &&
    std::chrono::duration<double>(now - snapshot.odom_time).count() <= sensor_timeout_s_);
}

void RowNavigationNode::publish_command(const geometry_msgs::msg::Twist &command)
{
  // Hold the lock through publication: feedback cannot be followed by an older
  // nonzero command prepared in another route/StopBase thread.
  std::scoped_lock lock(command_mutex_);
  last_command_ = photoelectric_reached_ ? geometry_msgs::msg::Twist() : command;
  robot_arm_interfaces::msg::RouteCommand message;
  message.header.stamp = now();
  message.stage_id = active_stage_id_;
  message.velocity = last_command_;
  route_command_pub_->publish(message);
  command_pub_->publish(last_command_);
}

void RowNavigationNode::begin_stage(const uint8_t stage_id)
{
  if (stage_id == 1) {
    std::scoped_lock lock(control_mutex_);
    have_task_yaw_reference_ = false;
  }
  publish_vision_state(stage_id, VisionStopController::State::Idle, 0.0,
    "route stage started; waiting for mango in depth range");
  {
    std::scoped_lock lock(command_mutex_);
    if (active_stage_id_ != stage_id) {
      active_stage_id_ = stage_id;
      photoelectric_reached_ = false;
    }
  }
  // Announce the stage with zero speeds before the first movement command.
  publish_zero();
}

double RowNavigationNode::task_yaw_reference(const double observed_yaw)
{
  std::scoped_lock lock(control_mutex_);
  if (!have_task_yaw_reference_) {
    task_yaw_reference_rad_ = observed_yaw;
    have_task_yaw_reference_ = true;
  }
  return task_yaw_reference_rad_;
}

void RowNavigationNode::publish_vision_state(
  const uint8_t stage_id, const VisionStopController::State state,
  const double error, const std::string &message)
{
  robot_arm_interfaces::msg::VisionStopState status;
  status.header.stamp = now();
  status.header.frame_id = "camera_color_optical_frame";
  status.stage_id = stage_id;
  status.state = static_cast<uint8_t>(state);
  status.tree_center_error_ratio = std::isfinite(error) ? error : 0.0;
  status.message = message;
  vision_state_pub_->publish(status);
}

VisionStopController::Parameters RowNavigationNode::vision_stop_parameters() const
{
  VisionStopController::Parameters p;
  p.kp = get_parameter("vision_tree_kp").as_double();
  p.ki = get_parameter("vision_tree_ki").as_double();
  p.integral_limit = get_parameter("vision_tree_integral_limit").as_double();
  p.max_align_speed = get_parameter("vision_tree_max_speed_mps").as_double();
  p.center_tolerance = get_parameter("vision_tree_center_tolerance_ratio").as_double();
  p.settle_time = get_parameter("vision_stop_settle_s").as_double();
  p.observation_timeout = get_parameter("vision_observation_timeout_s").as_double();
  p.search_distance = get_parameter("vision_tree_search_distance_m").as_double();
  p.alignment_timeout = get_parameter("vision_tree_alignment_timeout_s").as_double();
  p.control_sign = get_parameter("vision_tree_control_sign").as_double();
  for (const auto value : {p.kp, p.integral_limit, p.max_align_speed,
      p.center_tolerance, p.observation_timeout, p.search_distance, p.alignment_timeout}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("vision stop parameters must be finite and positive");
    }
  }
  if (!std::isfinite(p.ki) || p.ki < 0.0 || !std::isfinite(p.settle_time) ||
      p.settle_time < 0.0 || p.center_tolerance >= 0.5 || std::abs(p.control_sign) != 1.0) {
    throw std::invalid_argument("invalid vision integral, settle time or control sign");
  }
  return p;
}

void RowNavigationNode::stage_feedback_callback(
  const robot_arm_interfaces::msg::StageFeedback::SharedPtr message)
{
  {
    std::scoped_lock lock(command_mutex_);
    if (message->stage_id == 0 || message->stage_id != active_stage_id_ || !message->reached) {
      return;
    }
    photoelectric_reached_ = true;
  }
  publish_zero();
}

RowNavigationNode::PhotoelectricStop RowNavigationNode::photoelectric_stop(
  const uint8_t stage_id, std::chrono::steady_clock::time_point &started,
  std::chrono::steady_clock::time_point &stopped_since)
{
  {
    std::scoped_lock lock(command_mutex_);
    if (stage_id == 0 || stage_id != active_stage_id_ || !photoelectric_reached_) {
      return PhotoelectricStop::NotTriggered;
    }
  }
  publish_zero();
  const auto current = std::chrono::steady_clock::now();
  if (started == std::chrono::steady_clock::time_point{}) {
    started = current;
    RCLCPP_INFO(get_logger(), "photoelectric arrival at stage %u; holding zero until stopped",
      static_cast<unsigned>(stage_id));
  }
  const auto snapshot = sensors();
  const auto &velocity = snapshot.odom.twist.twist;
  const bool stopped = snapshot.have_odom &&
    snapshot.odom_time >= started &&
    std::chrono::duration<double>(current - snapshot.odom_time).count() <= sensor_timeout_s_ &&
    std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
    std::isfinite(velocity.angular.z) &&
    std::abs(velocity.linear.x) < stop_speed_epsilon_mps_ &&
    std::abs(velocity.linear.y) < stop_speed_epsilon_mps_ &&
    std::abs(velocity.angular.z) < stop_angular_epsilon_rad_s_;
  if (stopped) {
    if (stopped_since == std::chrono::steady_clock::time_point{}) {
      stopped_since = current;
    }
    if (std::chrono::duration<double>(current - stopped_since).count() >= stop_settle_s_) {
      return PhotoelectricStop::Stopped;
    }
  } else {
    stopped_since = std::chrono::steady_clock::time_point{};
  }
  return std::chrono::duration<double>(current - started).count() >= photoelectric_stop_timeout_s_ ?
    PhotoelectricStop::TimedOut : PhotoelectricStop::Waiting;
}

void RowNavigationNode::publish_zero()
{
  publish_command(geometry_msgs::msg::Twist());
}

geometry_msgs::msg::Twist RowNavigationNode::last_motion_command() const
{
  std::scoped_lock lock(command_mutex_);
  return last_command_;
}

void RowNavigationNode::clear_last_motion_command()
{
  std::scoped_lock lock(command_mutex_);
  last_command_ = geometry_msgs::msg::Twist();
}

RowNavigationNode::YawControlParameters RowNavigationNode::yaw_control_parameters() const
{
  std::scoped_lock lock(control_mutex_);
  return YawControlParameters{
    yaw_kp_, yaw_ki_, yaw_integral_limit_rad_s_, max_yaw_rate_rad_s_};
}

RowNavigationNode::MotionControlParameters RowNavigationNode::motion_control_parameters() const
{
  std::scoped_lock lock(control_mutex_);
  return MotionControlParameters{
    max_acceleration_mps2_, max_deceleration_mps2_, ultrasonic_filter_window_};
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
  if (distance <= stop_distance) return 0.0;
  if (distance >= slow_distance) return 1.0;
  return clamp((distance - stop_distance) / (slow_distance - stop_distance), 0.0, 1.0);
}

double RowNavigationNode::normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double RowNavigationNode::planar_lateral_displacement(
  const nav_msgs::msg::Odometry &start, const nav_msgs::msg::Odometry &current)
{
  const auto yaw = yaw_from_quaternion(start.pose.pose.orientation);
  const auto dx = current.pose.pose.position.x - start.pose.pose.position.x;
  const auto dy = current.pose.pose.position.y - start.pose.pose.position.y;
  return -std::sin(yaw) * dx + std::cos(yaw) * dy;
}

double RowNavigationNode::planar_longitudinal_displacement(
  const nav_msgs::msg::Odometry &start, const nav_msgs::msg::Odometry &current)
{
  const auto yaw = yaw_from_quaternion(start.pose.pose.orientation);
  const auto dx = current.pose.pose.position.x - start.pose.pose.position.x;
  const auto dy = current.pose.pose.position.y - start.pose.pose.position.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}

rclcpp_action::GoalResponse RowNavigationNode::follow_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowRow::Goal> goal)
{
  if ((goal->direction != 1 && goal->direction != -1) || goal->max_speed_mps <= 0.0 ||
      goal->max_speed_mps > 2.0 || goal->front_slow_distance_m <= goal->front_stop_distance_m ||
      goal->front_stop_distance_m <= 0.0 || goal->stage_id > 16 || goal->side_sensor > 1)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
  active_direction_ = goal->direction;
  active_row_id_ = goal->row_id;
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
  begin_stage(handle->get_goal()->stage_id);
  std::thread{[this, handle]() { execute_follow(handle); }}.detach();
}

void RowNavigationNode::execute_follow(const std::shared_ptr<FollowHandle> &handle)
{
  auto result = std::make_shared<FollowRow::Result>();
  const auto goal = handle->get_goal();
  //const auto initial = sensors();

  auto initial = sensors();
  const auto ready_deadline =
    std::chrono::steady_clock::now() + 3s;

  while (rclcpp::ok() &&
        !handle->is_canceling() &&
        !fresh(initial, true) &&
        std::chrono::steady_clock::now() < ready_deadline)
  {
    publish_zero();
    std::this_thread::sleep_for(50ms);
    initial = sensors();
  }

  if (!rclcpp::ok()) {
    active_.store(false);
    return;
  }

  if (!handle->is_canceling() && !fresh(initial, true))
  {
    publish_zero();
    set_phase(RoutePhase::Fault);
    result->success = false;
    result->reason = kSensorFault;
    result->message = "ultrasonic or odometry not ready after 3 seconds";
    handle->abort(result);
    active_.store(false);
    return;
  }

  const bool encoder_reverse = goal->direction < 0;
  const bool need_odom = true;
  const auto start_odom = initial.odom;
  const auto target_yaw = task_yaw_reference(
    yaw_from_quaternion(initial.odom.pose.pose.orientation));
  const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
  auto next = std::chrono::steady_clock::now();
  double yaw_integral = 0.0;
  double limited_linear_x = 0.0;
  bool reverse_motion_started = false;
  auto reverse_zero_since = std::chrono::steady_clock::time_point{};
  auto photoelectric_started = std::chrono::steady_clock::time_point{};
  auto photoelectric_stopped_since = std::chrono::steady_clock::time_point{};
  std::deque<double> front_history;
  std::deque<double> left_history;
  std::deque<double> right_history;
  double filtered_front = std::numeric_limits<double>::quiet_NaN();

  while (rclcpp::ok())
  {
    if (handle->is_canceling())
    {
      publish_zero();
      set_phase(RoutePhase::Paused);
      result->success = false;
      result->reason = kCanceled;
      result->message = "row action canceled; the mission will restart this step";
      handle->canceled(result);
      active_.store(false);
      return;
    }

    const auto photoelectric = photoelectric_stop(
      goal->stage_id, photoelectric_started, photoelectric_stopped_since);
    if (photoelectric != PhotoelectricStop::NotTriggered) {
      set_phase(RoutePhase::Braking);
      if (photoelectric == PhotoelectricStop::Stopped || photoelectric == PhotoelectricStop::TimedOut) {
        result->success = photoelectric == PhotoelectricStop::Stopped;
        result->reason = result->success ? kReachedEnd : kSensorFault;
        result->message = result->success ?
          "photoelectric arrival at stage " + std::to_string(goal->stage_id) :
          "photoelectric stop: fresh zero-velocity feedback was not confirmed before timeout";
        const auto current = sensors();
        result->traveled_m = std::abs(planar_longitudinal_displacement(start_odom, current.odom));
        set_phase(result->success ? RoutePhase::Stopped : RoutePhase::Fault);
        active_.store(false);
        if (result->success) handle->succeed(result); else handle->abort(result);
        return;
      }
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
      continue;
    }

    const auto snapshot = sensors();
    if (!fresh(snapshot, need_odom))
    {
      publish_zero();
      set_phase(RoutePhase::Fault);
      result->success = false;
      result->reason = kSensorFault;
      result->message = "ultrasonic or odometry data is stale";
      handle->abort(result);
      active_.store(false);
      return;
    }

    const auto motion = motion_control_parameters();
    double scale = 1.0;
    double traveled = 0.0;
    bool reached = false;
    bool stopped_by_zero_velocity = false;

    if (encoder_reverse)
    {
      traveled = std::max(
        0.0, -planar_longitudinal_displacement(start_odom, snapshot.odom));
      const double remaining = reverse_distance_m_ - traveled;

      const double braking_distance =
      std::max(0.0, remaining - reverse_distance_tolerance_m_);
      const double proportional_speed =
      std::sqrt(2.0 * max_deceleration_mps2_ * braking_distance);

      scale = clamp(
      proportional_speed / goal->max_speed_mps,0.0, 1.0);
      reached = remaining <= reverse_distance_tolerance_m_;
      // scale = speed_scale(
      //   remaining, reverse_slow_distance_m_, reverse_distance_tolerance_m_);
      // reached = remaining <= reverse_distance_tolerance_m_;

      const double wheel_speed =
        std::abs(snapshot.odom.twist.twist.linear.x);
      if (traveled > 0.02 || wheel_speed > stop_speed_epsilon_mps_) {
        reverse_motion_started = true;
      }

      if (goal->reverse_stop_on_zero_velocity && reverse_motion_started &&
          wheel_speed <= stop_speed_epsilon_mps_)
      {
        if (reverse_zero_since == std::chrono::steady_clock::time_point{}) {
          reverse_zero_since = std::chrono::steady_clock::now();
        }
        if (std::chrono::duration<double>(
              std::chrono::steady_clock::now() - reverse_zero_since).count() >=
            stop_settle_s_)
        {
          reached = true;
          stopped_by_zero_velocity = true;
        }
      }
      else
      {
        reverse_zero_since = std::chrono::steady_clock::time_point{};
      }
    }
    else
    {
      if (!snapshot.ultrasonic.front_valid ||
          !std::isfinite(snapshot.ultrasonic.front_m))
      {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = "front ultrasonic data is invalid";
        handle->abort(result);
        active_.store(false);
        return;
      }

      filtered_front = median_filter_update(
        front_history, snapshot.ultrasonic.front_m,
        motion.ultrasonic_filter_window);
      if (!std::isfinite(filtered_front))
      {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = "front ultrasonic filter has no valid sample";
        handle->abort(result);
        active_.store(false);
        return;
      }

      scale = speed_scale(
        filtered_front, goal->front_slow_distance_m,
        goal->front_stop_distance_m);
      reached = scale <= 0.0;
    }

    const double desired_linear_x = reached ? 0.0 :
      static_cast<double>(goal->direction) * goal->max_speed_mps * scale;
    limited_linear_x = slew_velocity(
      desired_linear_x, limited_linear_x, motion.max_acceleration_mps2,
      motion.max_deceleration_mps2, period.count());

    geometry_msgs::msg::Twist command;
    command.linear.x = limited_linear_x;

    if (!reached && goal->use_side_centering)
    {
      const bool right_side = goal->side_sensor == 1;
      const bool side_valid = right_side ? snapshot.ultrasonic.right_valid :
        snapshot.ultrasonic.left_valid;
      const double side_distance = right_side ? snapshot.ultrasonic.right_m :
        snapshot.ultrasonic.left_m;
      const double raw_side = side_valid && std::isfinite(side_distance) ?
        side_distance : std::numeric_limits<double>::quiet_NaN();
      auto &side_history = right_side ? right_history : left_history;
      const double filtered_side = median_filter_update(
        side_history, raw_side, motion.ultrasonic_filter_window);
      if (std::isfinite(filtered_side))
      {
        const double side_error = filtered_side - goal->side_target_m;
        const double correction_sign = right_side ? -1.0 : 1.0;
        command.linear.y = clamp(
          correction_sign * side_kp_ * side_error,
          -max_lateral_speed_mps_, max_lateral_speed_mps_);
      }
    }

    if (!reached && snapshot.have_odom)
    {
      const auto control = yaw_control_parameters();
      const auto yaw = yaw_from_quaternion(snapshot.odom.pose.pose.orientation);
      const auto yaw_error = normalize_angle(target_yaw - yaw);
      yaw_integral = clamp(
        yaw_integral + yaw_error * period.count(),
        -control.integral_limit, control.integral_limit);
      command.angular.z = clamp(
        control.kp * yaw_error + control.ki * yaw_integral,
        -control.max_rate, control.max_rate);
    }
    else
    {
      command.linear.y = 0.0;
      command.angular.z = 0.0;
      yaw_integral = 0.0;
    }

    publish_command(command);

    if (reached && std::abs(limited_linear_x) <= stop_speed_epsilon_mps_)
    {
      publish_zero();
      std::this_thread::sleep_for(
        std::chrono::duration<double>(stop_settle_s_));
      set_phase(RoutePhase::Stopped);
      result->success = true;
      result->reason = kReachedEnd;
      result->message = stopped_by_zero_velocity ?
        "reverse wheel velocity reached zero" :
        (encoder_reverse ? "reverse distance reached" :
         "front wall distance reached");
      result->traveled_m = traveled;
      handle->succeed(result);
      active_.store(false);
      return;
    }

    set_phase(reached || scale < 1.0 ?
      RoutePhase::Braking : RoutePhase::FollowRow);

    auto feedback = std::make_shared<FollowRow::Feedback>();
    feedback->progress = encoder_reverse ?
      static_cast<float>(clamp(
        traveled / reverse_distance_m_, 0.0, 1.0)) :
      static_cast<float>(1.0 - scale);
    feedback->phase = reached || scale < 1.0 ? "braking" :
      (encoder_reverse ? "reverse" : "following_row");
    feedback->front_distance_m = std::isfinite(filtered_front) ?
      filtered_front : snapshot.ultrasonic.front_m;
    feedback->left_distance_m = snapshot.ultrasonic.left_m;
    feedback->right_distance_m = snapshot.ultrasonic.right_m;
    feedback->command = command;
    handle->publish_feedback(feedback);

    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
}

rclcpp_action::GoalResponse RowNavigationNode::shift_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ShiftLane::Goal> goal)
{
  if ((goal->direction != 1 && goal->direction != -1) || goal->distance_m <= 0.0 ||
      goal->distance_m > 5.0 || goal->max_speed_mps <= 0.0 || goal->max_speed_mps > 2.0 ||
      goal->wall_sensor > 2 || goal->wall_target_m < 0.0 ||
      goal->wall_enable_tolerance_m < 0.0 ||
      goal->wall_control_cancel_side_distance_m < 0.0 ||
      goal->wall_control_cancel_side_tolerance_m < 0.0 || goal->stage_id > 16)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
  active_direction_ = goal->direction;
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
  begin_stage(handle->get_goal()->stage_id);
  std::thread{[this, handle]() { execute_shift(handle); }}.detach();
}

void RowNavigationNode::execute_shift(const std::shared_ptr<ShiftHandle> &handle)
{
  auto result = std::make_shared<ShiftLane::Result>();
  const auto goal = handle->get_goal();
  const auto initial = sensors();
  if (!fresh(initial, true))
  {
    result->success = false;
    result->reason = kSensorFault;
    result->message = "odometry or ultrasonic data is not ready";
    handle->abort(result);
    active_.store(false);
    return;
  }

  const auto start_odom = initial.odom;
  const auto target_yaw = task_yaw_reference(
    yaw_from_quaternion(start_odom.pose.pose.orientation));
  const auto target_side = goal->direction > 0 ? 1 : -1;
  const auto safety_distance = goal->safety_distance_m > 0.0 ?
    goal->safety_distance_m : default_safety_distance_m_;
  const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
  auto next = std::chrono::steady_clock::now();
  double yaw_integral = 0.0;
  double limited_linear_x = 0.0;
  double limited_linear_y = 0.0;
  bool wall_control_enabled = false;
  auto photoelectric_started = std::chrono::steady_clock::time_point{};
  auto photoelectric_stopped_since = std::chrono::steady_clock::time_point{};
  std::deque<double> front_history;
  std::deque<double> rear_history;
  std::deque<double> left_history;
  std::deque<double> right_history;
  VisionStopController vision_stop;
  auto last_vision_state = VisionStopController::State::Idle;
  const auto vision_stage_started = now();

  while (rclcpp::ok())
  {
    if (handle->is_canceling())
    {
      publish_zero();
      set_phase(RoutePhase::Paused);
      result->success = false;
      result->reason = kCanceled;
      result->message = "lane shift canceled; the mission will restart this step";
      publish_vision_state(goal->stage_id, VisionStopController::State::Idle, 0.0,
        "visual stop canceled with lane shift");
      handle->canceled(result);
      active_.store(false);
      return;
    }

    const auto photoelectric = photoelectric_stop(
      goal->stage_id, photoelectric_started, photoelectric_stopped_since);
    if (photoelectric != PhotoelectricStop::NotTriggered) {
      set_phase(RoutePhase::ShiftBraking);
      if (vision_stop.active()) {
        publish_zero();
        result->success = false;
        result->reason = kSafetyLimit;
        result->message = "photoelectric route end interrupted tree alignment; holding zero";
        publish_vision_state(goal->stage_id, VisionStopController::State::Fault,
          0.0, result->message);
        set_phase(RoutePhase::Fault);
        handle->abort(result);
        active_.store(false);
        return;
      }
      if (photoelectric == PhotoelectricStop::Stopped || photoelectric == PhotoelectricStop::TimedOut) {
        result->success = photoelectric == PhotoelectricStop::Stopped;
        result->reason = result->success ? kReachedEnd : kSensorFault;
        result->message = result->success ?
          "photoelectric arrival at stage " + std::to_string(goal->stage_id) :
          "photoelectric stop: fresh zero-velocity feedback was not confirmed before timeout";
        const auto current = sensors();
        result->traveled_m = std::abs(planar_lateral_displacement(start_odom, current.odom));
        set_phase(result->success ? RoutePhase::Stopped : RoutePhase::Fault);
        active_.store(false);
        if (result->success) handle->succeed(result); else handle->abort(result);
        return;
      }
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
      continue;
    }

    const auto snapshot = sensors();
    if (!fresh(snapshot, true))
    {
      publish_zero();
      result->success = false;
      result->reason = kSensorFault;
      result->message = "odometry or ultrasonic data is stale during lane shift";
      publish_vision_state(goal->stage_id, VisionStopController::State::Fault,
        0.0, result->message);
      set_phase(RoutePhase::Fault);
      handle->abort(result);
      active_.store(false);
      return;
    }

    const auto motion = motion_control_parameters();
    const double raw_front = snapshot.ultrasonic.front_valid &&
      std::isfinite(snapshot.ultrasonic.front_m) ?
      snapshot.ultrasonic.front_m : std::numeric_limits<double>::quiet_NaN();
    const double raw_rear = snapshot.ultrasonic.rear_valid &&
      std::isfinite(snapshot.ultrasonic.rear_m) ?
      snapshot.ultrasonic.rear_m : std::numeric_limits<double>::quiet_NaN();
    const double raw_left = snapshot.ultrasonic.left_valid &&
      std::isfinite(snapshot.ultrasonic.left_m) ?
      snapshot.ultrasonic.left_m : std::numeric_limits<double>::quiet_NaN();
    const double raw_right = snapshot.ultrasonic.right_valid &&
      std::isfinite(snapshot.ultrasonic.right_m) ?
      snapshot.ultrasonic.right_m : std::numeric_limits<double>::quiet_NaN();

    const double filtered_front = median_filter_update(
      front_history, raw_front, motion.ultrasonic_filter_window);
    const double filtered_rear = median_filter_update(
      rear_history, raw_rear, motion.ultrasonic_filter_window);
    const double filtered_left = median_filter_update(
      left_history, raw_left, motion.ultrasonic_filter_window);
    const double filtered_right = median_filter_update(
      right_history, raw_right, motion.ultrasonic_filter_window);

    const bool side_raw_valid = target_side > 0 ?
      std::isfinite(raw_left) : std::isfinite(raw_right);
    const bool side_valid = side_raw_valid &&
      (target_side > 0 ? std::isfinite(filtered_left) : std::isfinite(filtered_right));
    const double side_distance = target_side > 0 ? filtered_left : filtered_right;

    const auto traveled = std::max(
      0.0, static_cast<double>(goal->direction) *
      planar_lateral_displacement(start_odom, snapshot.odom));

    const auto visual_parameters = vision_stop_parameters();
    VisionStopController::Input visual_input;
    const auto steady_now = std::chrono::steady_clock::now();
    visual_input.now = std::chrono::duration<double>(steady_now.time_since_epoch()).count();
    robot_arm_interfaces::msg::VisionRowObservation observation;
    {
      std::scoped_lock lock(vision_mutex_);
      observation = vision_observation_;
      const double acquisition_age = now().seconds() -
        rclcpp::Time(observation.header.stamp).seconds();
      const double reception_age = std::chrono::duration<double>(
        steady_now - vision_observation_time_).count();
      visual_input.have_observation = have_vision_observation_ && acquisition_age >= -0.1 &&
        rclcpp::Time(observation.header.stamp) >= vision_stage_started;
      visual_input.observation_time = visual_input.now -
        std::max({0.0, acquisition_age, reception_age});
    }
    visual_input.frame_sequence = observation.frame_sequence;
    visual_input.mango_in_range = observation.mango_in_range;
    visual_input.tree_valid = observation.tree_valid;
    // Image-width ratio has no metric unit: never subtract odometry in meters.
    // Positive error is image-right; vision_tree_control_sign maps it to chassis Y.
    visual_input.tree_error = observation.tree_center_error_ratio;
    visual_input.lateral_position = planar_lateral_displacement(start_odom, snapshot.odom);
    const auto &measured_velocity = snapshot.odom.twist.twist;
    visual_input.stopped = std::isfinite(measured_velocity.linear.x) &&
      std::isfinite(measured_velocity.linear.y) && std::isfinite(measured_velocity.angular.z) &&
      std::abs(measured_velocity.linear.x) < stop_speed_epsilon_mps_ &&
      std::abs(measured_velocity.linear.y) < stop_speed_epsilon_mps_ &&
      std::abs(measured_velocity.angular.z) < stop_angular_epsilon_rad_s_ &&
      std::abs(limited_linear_x) < stop_speed_epsilon_mps_ &&
      std::abs(limited_linear_y) < stop_speed_epsilon_mps_;
    const auto visual_output = vision_stop.update(visual_input, visual_parameters,
      goal->vision_stop_enabled, goal->max_speed_mps, goal->direction);
    if (vision_stop.active()) {
      publish_vision_state(goal->stage_id, visual_output.state,
        visual_input.tree_error, visual_output.message);
      if (last_vision_state != visual_output.state) {
        RCLCPP_INFO(get_logger(), "visual stop at route step %u: state=%u %s",
          static_cast<unsigned>(goal->stage_id), static_cast<unsigned>(visual_output.state),
          visual_output.message.c_str());
        last_vision_state = visual_output.state;
      }
      if (visual_output.state == VisionStopController::State::Fault) {
        publish_zero();
        set_phase(RoutePhase::Fault);
        result->success = false;
        result->reason = kSensorFault;
        result->message = visual_output.message;
        handle->abort(result);
        active_.store(false);
        return;
      }
    }

    bool reached = false;
    double scale = 1.0;
    if (side_valid)
    {
      if (side_distance <= shift_side_target_m_ + 0.01)
      {
        reached = true;
      }
      else if (side_distance < safety_distance &&
               safety_distance > shift_side_target_m_ + 0.01)
      {
        publish_zero();
        result->success = false;
        result->reason = kSafetyLimit;
        result->message = "target-side ultrasonic safety limit reached";
        publish_vision_state(goal->stage_id, VisionStopController::State::Fault,
          visual_input.tree_error, result->message);
        set_phase(RoutePhase::Fault);
        handle->abort(result);
        active_.store(false);
        return;
      }
      else
      {
        scale = clamp(
          (side_distance - shift_side_target_m_) /
          (shift_side_slow_distance_m_ - shift_side_target_m_),
          0.15, 1.0);
      }
    }
    else if (traveled >= goal->distance_m - 0.02)
    {
      reached = true;
    }

    if (vision_stop.active() && reached &&
        visual_output.state != VisionStopController::State::Holding) {
      publish_zero();
      result->success = false;
      result->reason = kSafetyLimit;
      result->message = "lane endpoint reached before tree centering; holding zero";
      publish_vision_state(goal->stage_id, VisionStopController::State::Fault,
        visual_input.tree_error, result->message);
      set_phase(RoutePhase::Fault);
      handle->abort(result);
      active_.store(false);
      return;
    }
    if (vision_stop.active()) {reached = false;}

    double wall_distance = 0.0;
    bool wall_valid = false;
    if (goal->wall_sensor == 1)
    {
      wall_distance = filtered_front;
      wall_valid = std::isfinite(raw_front) && std::isfinite(filtered_front);
    }
    else if (goal->wall_sensor == 2)
    {
      wall_distance = filtered_rear;
      wall_valid = std::isfinite(raw_rear) && std::isfinite(filtered_rear);
    }

    const double wall_target = goal->wall_target_m > 0.0 ?
      goal->wall_target_m : shift_front_target_m_;
    const double wall_enable_tolerance =
      goal->wall_enable_tolerance_m > 0.0 ?
      goal->wall_enable_tolerance_m : shift_wall_enable_tolerance_m_;

    if (!wall_control_enabled && goal->wall_sensor != 0 && wall_valid &&
        std::abs(wall_distance - wall_target) <= wall_enable_tolerance)
    {
      wall_control_enabled = true;
    }

    const double cancel_side_distance =
      goal->wall_control_cancel_side_distance_m > 0.0 ?
      goal->wall_control_cancel_side_distance_m :
      shift_wall_cancel_side_distance_m_;
    const double cancel_side_tolerance =
      goal->wall_control_cancel_side_tolerance_m > 0.0 ?
      goal->wall_control_cancel_side_tolerance_m :
      shift_wall_cancel_side_tolerance_m_;

    if (goal->disable_wall_control_on_side_distance &&
        side_valid && cancel_side_distance > 0.0 &&
        std::abs(side_distance - cancel_side_distance) <= cancel_side_tolerance)
    {
      wall_control_enabled = false;
    }

    const double desired_linear_x = (!vision_stop.active() && !reached && wall_control_enabled && wall_valid) ?
      clamp(
        (goal->wall_sensor == 2 ? -1.0 : 1.0) *
        shift_front_kp_ * (wall_distance - wall_target),
        -max_front_correction_mps_, max_front_correction_mps_) : 0.0;
    const double desired_linear_y = vision_stop.active() ? visual_output.velocity :
      (reached ? 0.0 : static_cast<double>(goal->direction) * goal->max_speed_mps * scale);
    // Centering may reverse y, so check the actual commanded side as well.
    if (vision_stop.active() &&
        ((desired_linear_y > 0.0 && std::isfinite(raw_left) && raw_left < safety_distance) ||
         (desired_linear_y < 0.0 && std::isfinite(raw_right) && raw_right < safety_distance))) {
      publish_zero();
      result->success = false;
      result->reason = kSafetyLimit;
      result->message = "side ultrasonic safety limit during tree centering";
      publish_vision_state(goal->stage_id, VisionStopController::State::Fault,
        visual_input.tree_error, result->message);
      set_phase(RoutePhase::Fault);
      handle->abort(result);
      active_.store(false);
      return;
    }
    if (vision_stop.active()) {
      limited_linear_x = desired_linear_x;
      limited_linear_y = desired_linear_y;
    } else {
      limited_linear_x = slew_velocity(
        desired_linear_x, limited_linear_x, motion.max_acceleration_mps2,
        motion.max_deceleration_mps2, period.count());
      limited_linear_y = slew_velocity(
        desired_linear_y, limited_linear_y, motion.max_acceleration_mps2,
        motion.max_deceleration_mps2, period.count());
    }
    if (visual_output.state == VisionStopController::State::Holding) {
      limited_linear_x = 0.0;
      limited_linear_y = 0.0;
      set_phase(RoutePhase::Stopped);
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = limited_linear_x;
    command.linear.y = limited_linear_y;

    if (!reached && visual_output.state != VisionStopController::State::Holding &&
        visual_output.state != VisionStopController::State::Settling)
    {
      const auto yaw = yaw_from_quaternion(snapshot.odom.pose.pose.orientation);
      const auto control = yaw_control_parameters();
      const auto yaw_error = normalize_angle(target_yaw - yaw);
      yaw_integral = clamp(
        yaw_integral + yaw_error * period.count(),
        -control.integral_limit, control.integral_limit);
      command.angular.z = clamp(
        control.kp * yaw_error + control.ki * yaw_integral,
        -control.max_rate, control.max_rate);
    }
    else
    {
      command.angular.z = 0.0;
      yaw_integral = 0.0;
    }

    publish_command(command);

    if (reached &&
        std::abs(limited_linear_x) <= stop_speed_epsilon_mps_ &&
        std::abs(limited_linear_y) <= stop_speed_epsilon_mps_)
    {
      publish_zero();
      std::this_thread::sleep_for(
        std::chrono::duration<double>(stop_settle_s_));
      set_phase(RoutePhase::Stopped);
      result->success = true;
      result->reason = kReachedEnd;
      result->message = side_valid ?
        "target-side wall distance reached" : "lane distance reached";
      result->traveled_m = traveled;
      handle->succeed(result);
      active_.store(false);
      return;
    }

    set_phase(visual_output.state == VisionStopController::State::Holding ?
      RoutePhase::Stopped : (vision_stop.active() || reached || scale < 1.0 ?
      RoutePhase::ShiftBraking : RoutePhase::ShiftLane));

    auto feedback = std::make_shared<ShiftLane::Feedback>();
    feedback->progress = static_cast<float>(
      clamp(traveled / goal->distance_m, 0.0, 1.0));
    feedback->phase = reached || scale < 1.0 ?
      "braking" : "shifting_lane";
    if (vision_stop.active()) {
      feedback->phase = visual_output.state == VisionStopController::State::Holding ?
        "vision_holding" : "vision_centering";
    }
    feedback->left_distance_m =
      std::isfinite(filtered_left) ? filtered_left : snapshot.ultrasonic.left_m;
    feedback->right_distance_m =
      std::isfinite(filtered_right) ? filtered_right : snapshot.ultrasonic.right_m;
    feedback->command = command;
    handle->publish_feedback(feedback);

    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
}

rclcpp_action::GoalResponse RowNavigationNode::stop_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const StopBase::Goal>)
{
  // Stop is an emergency/interrupt action and must be accepted while a route action is active.
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
  const auto timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : 3.0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  auto stopped_since = std::chrono::steady_clock::time_point{};
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (handle->is_canceling())
    {
      publish_zero();
      result->success = false;
      result->message = "stop request canceled";
      handle->canceled(result);
      return;
    }
    const auto command = last_motion_command();
    const auto elapsed = timeout_s - std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    const auto brake_scale = mango_brake_time_s_ > 0.0 ?
      clamp(1.0 - elapsed / mango_brake_time_s_, 0.0, 1.0) : 0.0;
    geometry_msgs::msg::Twist braking_command = command;
    braking_command.linear.x *= brake_scale;
    braking_command.linear.y *= brake_scale;
    braking_command.angular.z *= brake_scale;
    publish_command(braking_command);
    const auto snapshot = sensors();
    const bool stopped = snapshot.have_odom &&
      std::abs(snapshot.odom.twist.twist.linear.x) < stop_speed_epsilon_mps_ &&
      std::abs(snapshot.odom.twist.twist.linear.y) < stop_speed_epsilon_mps_ &&
      std::abs(snapshot.odom.twist.twist.angular.z) < stop_angular_epsilon_rad_s_;
    if (stopped)
    {
      if (stopped_since == std::chrono::steady_clock::time_point{}) stopped_since = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - stopped_since).count() >= stop_settle_s_)
      {
        set_phase(RoutePhase::Stopped);
        clear_last_motion_command();
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
      1.0 - std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count() /
      timeout_s, 0.0, 1.0));
    feedback->stopped = stopped;
    handle->publish_feedback(feedback);
    std::this_thread::sleep_for(50ms);
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
