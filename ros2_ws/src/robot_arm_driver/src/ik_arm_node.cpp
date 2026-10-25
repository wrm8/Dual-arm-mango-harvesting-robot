#include "robot_arm_driver/ik_arm_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace robot_arm_driver
{
IkArmNode::IkArmNode()
  : Node("robot_arm_ik_driver")
{
  declare_parameter("serial_device", "/dev/ttyACM1");
  declare_parameter("baud_rate", 115200);
  declare_parameter("use_direction_gpio", false);
  declare_parameter("tx_enable_gpio", -1);
  declare_parameter("rx_enable_gpio", -1);
  declare_parameter("base_frame", "base_link");
  declare_parameter("require_base_frame", true);
  declare_parameter("joint_names", std::vector<std::string>{
      "joint_1", "joint_2", "joint_3", "joint_4", "wrist_roll", "gripper"});
  declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
  declare_parameter("zero_ticks", std::vector<double>{500, 500, 500, 500, 500, 500});
  declare_parameter("ticks_per_radian", std::vector<double>{
      238.7324, 238.7324, 238.7324, 238.7324, 238.7324, 238.7324});
  declare_parameter("directions", std::vector<double>{1, 1, 1, 1, 1, 1});
  declare_parameter("min_ticks", std::vector<int64_t>{0, 0, 0, 0, 0, 0});
  declare_parameter("max_ticks", std::vector<int64_t>{1000, 1000, 1000, 1000, 1000, 1000});
  declare_parameter("base_height_m", 0.0);
  declare_parameter("shoulder_offset_m", 0.0);
  declare_parameter("upper_arm_length_m", 0.0);
  declare_parameter("forearm_length_m", 0.0);
  declare_parameter("tool_length_m", 0.0);
  declare_parameter("elbow_up", false);
  declare_parameter("default_motion_ms", 2000);
  declare_parameter("gripper_open_tick", 500);
  declare_parameter("gripper_close_tick", 500);
  declare_parameter("pick_pitch_deg", -90.0);
  declare_parameter("pick_roll_deg", 0.0);
  declare_parameter("x_min_m", 0.100);
  declare_parameter("x_max_m", 0.400);
  declare_parameter("y_min_m", -0.450);
  declare_parameter("y_max_m", 0.450);
  declare_parameter("z_min_m", 0.020);
  declare_parameter("z_max_m", 0.500);
  declare_parameter("home_x_m", 0.200);
  declare_parameter("home_y_m", 0.0);
  declare_parameter("home_z_m", 0.200);

  joint_names_ = get_parameter("joint_names").as_string_array();
  servo_ids_ = get_parameter("servo_ids").as_integer_array();
  zero_ticks_ = get_parameter("zero_ticks").as_double_array();
  ticks_per_radian_ = get_parameter("ticks_per_radian").as_double_array();
  directions_ = get_parameter("directions").as_double_array();
  min_ticks_ = get_parameter("min_ticks").as_integer_array();
  max_ticks_ = get_parameter("max_ticks").as_integer_array();

  const auto joint_count = joint_names_.size();

  if (joint_count != 6U || servo_ids_.size() != joint_count ||
      zero_ticks_.size() != joint_count || ticks_per_radian_.size() != joint_count ||
      directions_.size() != joint_count || min_ticks_.size() != joint_count ||
      max_ticks_.size() != joint_count)
  {
    throw std::invalid_argument(
            "IK driver needs six calibrated joints: 4 arm, wrist_roll, gripper");
  }

  for (std::size_t i = 0; i < joint_count; ++i)
  {
    if (servo_ids_[i] < 0 || servo_ids_[i] > 253 || ticks_per_radian_[i] <= 0.0 ||
        std::abs(directions_[i]) != 1.0 || min_ticks_[i] < 0 || max_ticks_[i] > 1000 ||
        min_ticks_[i] > max_ticks_[i])
    {
      throw std::invalid_argument(
              "invalid servo calibration at index " + std::to_string(i));
    }
  }

  const PlanarArmGeometry geometry{
    get_parameter("base_height_m").as_double(),
    get_parameter("shoulder_offset_m").as_double(),
    get_parameter("upper_arm_length_m").as_double(),
    get_parameter("forearm_length_m").as_double(),
    get_parameter("tool_length_m").as_double()};

  kinematics_ = std::make_unique<PlanarArmKinematics>(geometry);

  base_frame_ = get_parameter("base_frame").as_string();
  require_base_frame_ = get_parameter("require_base_frame").as_bool();
  elbow_up_ = get_parameter("elbow_up").as_bool();
  default_motion_ms_ = static_cast<uint16_t>(get_parameter("default_motion_ms").as_int());
  gripper_open_ = static_cast<uint16_t>(get_parameter("gripper_open_tick").as_int());
  gripper_close_ = static_cast<uint16_t>(get_parameter("gripper_close_tick").as_int());
  pick_pitch_ = get_parameter("pick_pitch_deg").as_double();
  pick_roll_ = get_parameter("pick_roll_deg").as_double();
  x_min_ = get_parameter("x_min_m").as_double();
  x_max_ = get_parameter("x_max_m").as_double();
  y_min_ = get_parameter("y_min_m").as_double();
  y_max_ = get_parameter("y_max_m").as_double();
  z_min_ = get_parameter("z_min_m").as_double();
  z_max_ = get_parameter("z_max_m").as_double();
  home_.header.frame_id = base_frame_;
  home_.point.x = get_parameter("home_x_m").as_double();
  home_.point.y = get_parameter("home_y_m").as_double();
  home_.point.z = get_parameter("home_z_m").as_double();

  BusConfig bus_config;

  bus_config.device = get_parameter("serial_device").as_string();
  bus_config.baud_rate = static_cast<int>(get_parameter("baud_rate").as_int());
  bus_config.use_direction_gpio = get_parameter("use_direction_gpio").as_bool();
  bus_config.tx_enable_gpio = static_cast<int>(get_parameter("tx_enable_gpio").as_int());
  bus_config.rx_enable_gpio = static_cast<int>(get_parameter("rx_enable_gpio").as_int());
  bus_ = std::make_unique<ServoBus>(bus_config);
  bus_->open();

  state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS());
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
}

bool IkArmNode::valid_point(
  const geometry_msgs::msg::PointStamped &point, std::string &reason) const
{
  if (!std::isfinite(point.point.x) || !std::isfinite(point.point.y) ||
      !std::isfinite(point.point.z))
  {
    reason = "target contains NaN or infinity";
    return false;
  }
  if (require_base_frame_ && point.header.frame_id != base_frame_)
  {
    reason = "target must be in " + base_frame_;
    return false;
  }
  if (point.point.x < x_min_ || point.point.x > x_max_ || point.point.y < y_min_ ||
      point.point.y > y_max_ || point.point.z < z_min_ || point.point.z > z_max_)
  {
    reason = "target is outside configured Cartesian limits";
    return false;
  }
  return true;
}

uint16_t IkArmNode::duration_ms(const builtin_interfaces::msg::Duration &duration) const
{
  const int64_t milliseconds = static_cast<int64_t>(duration.sec) * 1000 +
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

uint16_t IkArmNode::tick(const std::size_t index, const double radians) const
{
  const auto raw = std::lround(
    zero_ticks_[index] + directions_[index] * radians * ticks_per_radian_[index]);

  if (raw < min_ticks_[index] || raw > max_ticks_[index])
  {
    throw std::out_of_range("IK solution exceeds servo limit: " + joint_names_[index]);
  }
  return static_cast<uint16_t>(raw);
}

void IkArmNode::publish(const std::array<double, 5> &joints, const uint16_t gripper)
{
  sensor_msgs::msg::JointState state;

  state.header.stamp = now();
  state.name = joint_names_;
  state.position = {
    joints[0], joints[1], joints[2], joints[3], joints[4],
    (static_cast<double>(gripper) - zero_ticks_[5]) /
    (directions_[5] * ticks_per_radian_[5])};
  state_pub_->publish(state);
}

void IkArmNode::move(
  const geometry_msgs::msg::PointStamped &point, const double pitch_deg,
  const double roll_deg, const uint16_t gripper, const uint16_t duration)
{
  const auto arm = kinematics_->inverse(
    point.point.x, point.point.y, point.point.z, pitch_deg * kPi / 180.0, elbow_up_);
  const std::array<double, 5> joints{
    arm[0], arm[1], arm[2], arm[3], roll_deg *kPi / 180.0};

  for (std::size_t i = 0; i < 5U; ++i)
  {
    bus_->move(static_cast<uint8_t>(servo_ids_[i]), tick(i, joints[i]), duration);
  }
  if (gripper < min_ticks_[5] || gripper > max_ticks_[5])
  {
    throw std::out_of_range("gripper tick is outside calibrated limit");
  }
  bus_->move(static_cast<uint8_t>(servo_ids_[5]), gripper, duration);
  publish(joints, gripper);
}

template<typename HandleT>
bool IkArmNode::wait(const std::shared_ptr<HandleT> &handle, const uint16_t duration) const
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

rclcpp_action::GoalResponse IkArmNode::claim()
{
  bool expected = false;

  return busy_.compare_exchange_strong(expected, true) ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::GoalResponse IkArmNode::accept_move(const MoveCartesian::Goal &goal)
{
  std::string reason;

  if (!valid_point(goal.target, reason) || !std::isfinite(goal.pitch_deg) ||
      !std::isfinite(goal.roll_deg) || !std::isfinite(goal.claw_position))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  try {
    duration_ms(goal.duration);
    static_cast<void>(kinematics_->inverse(
                        goal.target.point.x, goal.target.point.y, goal.target.point.z,
                        goal.pitch_deg * kPi / 180.0, elbow_up_));
  }
  catch (const std::exception &) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return claim();
}

rclcpp_action::GoalResponse IkArmNode::accept_pick(const PickPoint::Goal &goal)
{
  std::string reason;
  auto grasp = goal.target;

  grasp.point.x += goal.grasp_offset_x_m;
  grasp.point.y += goal.grasp_offset_y_m;
  grasp.point.z += goal.grasp_offset_z_m;
  auto pregrasp = grasp;

  pregrasp.point.z += goal.approach_height_m;
  if (goal.approach_height_m < 0.0 || !valid_point(grasp, reason) ||
      !valid_point(pregrasp, reason))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  try {
    duration_ms(goal.duration);
    kinematics_->inverse(
      pregrasp.point.x, pregrasp.point.y, pregrasp.point.z,
      pick_pitch_ * kPi / 180.0, elbow_up_);
    kinematics_->inverse(
      grasp.point.x, grasp.point.y, grasp.point.z,
      pick_pitch_ * kPi / 180.0, elbow_up_);
  }
  catch (const std::exception &) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return claim();
}

void IkArmNode::execute_move(const std::shared_ptr<MoveHandle> &handle)
{
  auto result = std::make_shared<MoveCartesian::Result>();
  const auto goal = handle->get_goal();

  try {
    const auto duration = duration_ms(goal->duration);
    move(
      goal->target, goal->pitch_deg, goal->roll_deg,
      static_cast<uint16_t>(std::lround(goal->claw_position)), duration);
    if (goal->wait_for_completion && !wait(handle, duration))
    {
      result->success = false;
      result->message = "cancelled; servo motion may still be active";
      result->final_position = goal->target;
      handle->canceled(result);
    }
    else
    {
      result->success = true;
      result->message = "IK motion completed";
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

void IkArmNode::execute_pick(const std::shared_ptr<PickHandle> &handle)
{
  auto result = std::make_shared<PickPoint::Result>();
  const auto goal = handle->get_goal();
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
    move(home_, pick_pitch_, pick_roll_, gripper_open_, 500);
    if (!wait(handle, 500))
    {
      throw std::runtime_error("cancelled");
    }
    report("pregrasp", 0.2F);
    move(pregrasp, pick_pitch_, pick_roll_, gripper_open_, duration);
    if (!wait(handle, duration))
    {
      throw std::runtime_error("cancelled");
    }
    report("descending", 0.45F);
    move(grasp, pick_pitch_, pick_roll_, gripper_open_, duration);
    if (!wait(handle, duration))
    {
      throw std::runtime_error("cancelled");
    }
    report("closing_gripper", 0.65F);
    move(grasp, pick_pitch_, pick_roll_, gripper_close_, 500);
    if (!wait(handle, 500))
    {
      throw std::runtime_error("cancelled");
    }
    report("lifting", 0.8F);
    move(pregrasp, pick_pitch_, pick_roll_, gripper_close_, duration);
    if (!wait(handle, duration))
    {
      throw std::runtime_error("cancelled");
    }
    if (goal->return_home)
    {
      report("home", 0.9F);
      move(home_, pick_pitch_, pick_roll_, gripper_close_, duration);
      if (!wait(handle, duration))
      {
        throw std::runtime_error("cancelled");
      }
    }
    result->success = true;
    result->message = "IK pick sequence completed";
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
