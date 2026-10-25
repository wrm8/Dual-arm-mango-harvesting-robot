#include "robot_arm_driver/coordinate_arm_node.hpp"

#include "robot_arm_driver/coordinate_protocol.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace robot_arm_driver
{
CoordinateArmNode::CoordinateArmNode()
  : Node("robot_arm_coordinate_driver")
{
  declare_parameter("serial_device", "/dev/ttyACM1");
  declare_parameter("baud_rate", 1000000);
  declare_parameter("base_frame", "base_link");
  declare_parameter("require_base_frame", true);
  declare_parameter("x_min_m", 0.100);
  declare_parameter("x_max_m", 0.400);
  declare_parameter("y_min_m", -0.450);
  declare_parameter("y_max_m", 0.450);
  declare_parameter("z_min_m", 0.020);
  declare_parameter("z_max_m", 0.500);
  declare_parameter("default_motion_ms", 2000);
  declare_parameter("gripper_servo_id", 6);
  declare_parameter("gripper_open_position", 1086);
  declare_parameter("gripper_close_position", 2418);
  declare_parameter("gripper_speed", 2500);
  declare_parameter("pick_pitch_deg", -90.0);
  declare_parameter("pick_roll_deg", 0.0);
  declare_parameter("home_x_m", 0.200);
  declare_parameter("home_y_m", 0.0);
  declare_parameter("home_z_m", 0.200);

  base_frame_ = get_parameter("base_frame").as_string();
  require_base_frame_ = get_parameter("require_base_frame").as_bool();
  x_min_ = get_parameter("x_min_m").as_double();
  x_max_ = get_parameter("x_max_m").as_double();
  y_min_ = get_parameter("y_min_m").as_double();
  y_max_ = get_parameter("y_max_m").as_double();
  z_min_ = get_parameter("z_min_m").as_double();
  z_max_ = get_parameter("z_max_m").as_double();
  default_motion_ms_ = static_cast<uint16_t>(get_parameter("default_motion_ms").as_int());
  gripper_id_ = static_cast<uint8_t>(get_parameter("gripper_servo_id").as_int());
  gripper_open_ = static_cast<uint16_t>(get_parameter("gripper_open_position").as_int());
  gripper_close_ = static_cast<uint16_t>(get_parameter("gripper_close_position").as_int());
  gripper_speed_ = static_cast<uint16_t>(get_parameter("gripper_speed").as_int());
  pick_pitch_ = get_parameter("pick_pitch_deg").as_double();
  pick_roll_ = get_parameter("pick_roll_deg").as_double();
  home_.point.x = get_parameter("home_x_m").as_double();
  home_.point.y = get_parameter("home_y_m").as_double();
  home_.point.z = get_parameter("home_z_m").as_double();
  home_.header.frame_id = base_frame_;

  if (!valid_point(home_, nullptr) || default_motion_ms_ < 20U)
  {
    throw std::invalid_argument("invalid coordinate limits, home pose, or duration");
  }

  bus_ = std::make_unique<CoordinateBus>(
    get_parameter("serial_device").as_string(),
    static_cast<int>(get_parameter("baud_rate").as_int()));
  bus_->open();

  move_server_ = rclcpp_action::create_server<MoveCartesian>(
    this, "move_cartesian",
    [this](const auto &, const auto goal) {
      return accept_move(*goal);
    },
    [](const std::shared_ptr<MoveHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<MoveHandle> handle) {
      std::thread{[this, handle] {
                    execute_move(handle);
                  }}.detach();
    });
  pick_server_ = rclcpp_action::create_server<PickPoint>(
    this, "pick_point",
    [this](const auto &, const auto goal) {
      return accept_pick(*goal);
    },
    [](const std::shared_ptr<PickHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<PickHandle> handle) {
      std::thread{[this, handle] {
                    execute_pick(handle);
                  }}.detach();
    });
  RCLCPP_INFO(get_logger(), "coordinate backend opened; target frame is %s", base_frame_.c_str());
}

bool CoordinateArmNode::valid_point(
  const geometry_msgs::msg::PointStamped &point, std::string *reason) const
{
  const auto fail = [reason](const std::string &message) {
                      if (reason != nullptr)
                      {
                        *reason = message;
                      }
                      return false;
                    };

  if (!std::isfinite(point.point.x) || !std::isfinite(point.point.y) ||
      !std::isfinite(point.point.z))
  {
    return fail("target contains NaN or infinity");
  }
  if (require_base_frame_ && point.header.frame_id != base_frame_)
  {
    return fail("target must be transformed into " + base_frame_);
  }
  if (point.point.x < x_min_ || point.point.x > x_max_ || point.point.y < y_min_ ||
      point.point.y > y_max_ || point.point.z < z_min_ || point.point.z > z_max_)
  {
    return fail("target is outside configured Cartesian limits");
  }
  return true;
}

uint16_t CoordinateArmNode::duration_ms(
  const builtin_interfaces::msg::Duration &duration) const
{
  const auto milliseconds = static_cast<int64_t>(duration.sec) * 1000 +
                            duration.nanosec / 1000000;

  if (milliseconds == 0)
  {
    return default_motion_ms_;
  }
  if (milliseconds < 20 || milliseconds > 30000)
  {
    throw std::invalid_argument("duration must be 20 to 30000 ms");
  }
  return static_cast<uint16_t>(milliseconds);
}

void CoordinateArmNode::send_pose(
  const geometry_msgs::msg::PointStamped &point, const double pitch_deg,
  const double roll_deg, const double claw, const uint16_t duration)
{
  CartesianCommand command;

  command.x_mm = static_cast<int16_t>(std::lround(point.point.x * 1000.0));
  command.y_mm = static_cast<int16_t>(std::lround(point.point.y * 1000.0));
  command.z_mm = static_cast<int16_t>(std::lround(point.point.z * 1000.0));
  command.pitch_deci_degrees = static_cast<int16_t>(std::lround(pitch_deg * 10.0));
  command.roll_degrees = static_cast<int16_t>(std::lround(roll_deg));
  command.claw_position = static_cast<int16_t>(std::lround(claw));
  command.duration_ms = duration;
  bus_->send(encode_coordinate_command(command));
}

void CoordinateArmNode::send_gripper(const uint16_t position)
{
  bus_->send(encode_gripper_command(gripper_id_, position, gripper_speed_));
}

template<typename HandleT>
bool CoordinateArmNode::wait_or_cancel(
  const std::shared_ptr<HandleT> &handle, const uint16_t duration)
{
  for (uint16_t elapsed = 0; elapsed < duration; elapsed = static_cast<uint16_t>(elapsed + 20U))
  {
    if (handle->is_canceling())
    {
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
  return true;
}

rclcpp_action::GoalResponse CoordinateArmNode::claim_goal(std::string &reason)
{
  bool expected = false;

  if (!busy_.compare_exchange_strong(expected, true))
  {
    reason = "another motion is active";
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse CoordinateArmNode::accept_move(
  const MoveCartesian::Goal &goal)
{
  std::string reason;

  if (!valid_point(goal.target, &reason) || !std::isfinite(goal.pitch_deg) ||
      !std::isfinite(goal.roll_deg) || !std::isfinite(goal.claw_position))
  {
    RCLCPP_WARN(
      get_logger(), "reject MoveCartesian: %s",
      reason.empty() ? "invalid orientation or claw" : reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  try {
    static_cast<void>(duration_ms(goal.duration));
  }
  catch (const std::exception &error) {
    RCLCPP_WARN(get_logger(), "%s", error.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return claim_goal(reason);
}

rclcpp_action::GoalResponse CoordinateArmNode::accept_pick(
  const PickPoint::Goal &goal)
{
  std::string reason;
  auto grasp = goal.target;

  grasp.point.x += goal.grasp_offset_x_m;
  grasp.point.y += goal.grasp_offset_y_m;
  grasp.point.z += goal.grasp_offset_z_m;
  auto pregrasp = grasp;

  pregrasp.point.z += goal.approach_height_m;
  if (goal.approach_height_m < 0.0 || !valid_point(grasp, &reason) ||
      !valid_point(pregrasp, &reason))
  {
    RCLCPP_WARN(
      get_logger(), "reject PickPoint: %s",
      reason.empty() ? "approach height must be non-negative" : reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  try {
    static_cast<void>(duration_ms(goal.duration));
  }
  catch (const std::exception &error) {
    RCLCPP_WARN(get_logger(), "%s", error.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return claim_goal(reason);
}

void CoordinateArmNode::execute_move(const std::shared_ptr<MoveHandle> &handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<MoveCartesian::Result>();

  try {
    const auto duration = duration_ms(goal->duration);
    send_pose(goal->target, goal->pitch_deg, goal->roll_deg, goal->claw_position, duration);
    auto feedback = std::make_shared<MoveCartesian::Feedback>();
    feedback->stage = "moving";
    feedback->progress = 0.0F;
    handle->publish_feedback(feedback);
    if (goal->wait_for_completion && !wait_or_cancel(handle, duration))
    {
      result->success = false;
      result->message = "cancelled; controller trajectory may still be in progress";
      result->final_position = goal->target;
      handle->canceled(result);
    }
    else
    {
      result->success = true;
      result->message = "coordinate command completed";
      result->final_position = goal->target;
      handle->succeed(result);
    }
  }
  catch (const std::exception &error) {
    result->success = false;
    result->message = error.what();
    handle->abort(result);
  }
  busy_.store(false);
}

void CoordinateArmNode::execute_pick(const std::shared_ptr<PickHandle> &handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<PickPoint::Result>();
  auto grasp = goal->target;

  grasp.point.x += goal->grasp_offset_x_m;
  grasp.point.y += goal->grasp_offset_y_m;
  grasp.point.z += goal->grasp_offset_z_m;
  auto pregrasp = grasp;

  pregrasp.point.z += goal->approach_height_m;
  const auto report = [&handle](const char *stage, const float progress) {
                        auto feedback = std::make_shared<PickPoint::Feedback>();

                        feedback->stage = stage;
                        feedback->progress = progress;
                        handle->publish_feedback(feedback);
                      };

  try {
    const auto duration = duration_ms(goal->duration);
    report("opening_gripper", 0.0F);
    send_gripper(gripper_open_);
    if (!wait_or_cancel(handle, 500))
    {
      throw std::runtime_error("pick cancelled before approach");
    }
    report("moving_to_pregrasp", 0.15F);
    send_pose(pregrasp, pick_pitch_, pick_roll_, 0.0, duration);
    if (!wait_or_cancel(handle, duration))
    {
      throw std::runtime_error("pick cancelled during pregrasp");
    }
    report("descending", 0.45F);
    send_pose(grasp, pick_pitch_, pick_roll_, 0.0, duration);
    if (!wait_or_cancel(handle, duration))
    {
      throw std::runtime_error("pick cancelled during descent");
    }
    report("closing_gripper", 0.65F);
    send_gripper(gripper_close_);
    if (!wait_or_cancel(handle, 700))
    {
      throw std::runtime_error("pick cancelled while closing gripper");
    }
    report("lifting", 0.80F);
    send_pose(pregrasp, pick_pitch_, pick_roll_, 0.0, duration);
    if (!wait_or_cancel(handle, duration))
    {
      throw std::runtime_error("pick cancelled during lift");
    }
    if (goal->return_home)
    {
      report("returning_home", 0.90F);
      send_pose(home_, pick_pitch_, pick_roll_, 0.0, duration);
      if (!wait_or_cancel(handle, duration))
      {
        throw std::runtime_error("pick cancelled while returning home");
      }
    }
    result->success = true;
    result->message = "pick sequence completed";
    result->final_position = goal->return_home ? home_ : pregrasp;
    handle->succeed(result);
  }
  catch (const std::exception &error) {
    result->success = false;
    result->message = error.what();
    result->final_position = pregrasp;
    if (handle->is_canceling())
    {
      handle->canceled(result);
    }
    else
    {
      handle->abort(result);
    }
  }
  busy_.store(false);
}
}  // namespace robot_arm_driver
