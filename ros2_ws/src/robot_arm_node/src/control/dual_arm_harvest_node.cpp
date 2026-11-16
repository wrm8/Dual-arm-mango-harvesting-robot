#include "robot_arm_node/motion/urdf_kinematics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_arm_interfaces/action/move_joints.hpp>
#include <robot_arm_interfaces/action/trigger_strike.hpp>
#include <robot_arm_interfaces/msg/harvest_target_array.hpp>
#include <robot_arm_interfaces/msg/harvest_result.hpp>
#include <robot_arm_interfaces/msg/vision_stop_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

namespace robot_arm_node::control
{
class DualArmHarvestNode final : public rclcpp::Node
{
  using MoveJoints = robot_arm_interfaces::action::MoveJoints;
  using GoalHandleMoveJoints = rclcpp_action::ClientGoalHandle<MoveJoints>;
  using TriggerStrike = robot_arm_interfaces::action::TriggerStrike;
  using HarvestArray = robot_arm_interfaces::msg::HarvestTargetArray;
  using StopState = robot_arm_interfaces::msg::VisionStopState;
  struct HarvestTimeout : std::runtime_error
  {
    HarvestTimeout() : std::runtime_error("10-second harvest parking budget expired") {}
  };
  struct GoalRejected : std::runtime_error
  {
    using std::runtime_error::runtime_error;
  };

  struct MotionProfile
  {
    std::vector<int64_t> speeds;
    int64_t duration_ms{5000};  // V3 joint5 travel time; HX joints use speeds.
  };

  struct ArmSolution
  {
    std::vector<double> urdf;
    std::vector<double> native;
    double gripper_pregrasp{0.0};
    double gripper_close{0.0};
    bool upper_target{false};
  };

  struct GraspProfile
  {
    std::array<double, 3> offset_m{};
    double pregrasp{0.0};
    double close{0.0};
  };

public:
  DualArmHarvestNode() : Node("dual_arm_harvest_node")
  {
    const auto &overrides = get_node_parameters_interface()->get_parameter_overrides();
    if (overrides.count("arm_speeds") || overrides.count("move_duration_ms"))
    {
      throw std::invalid_argument(
        "shared arm_speeds/move_duration_ms removed; use top/bottom/home_arm_speeds and *_move_duration_ms");
    }
    dry_run_ = declare_parameter("dry_run", true);
    allow_motion_ = declare_parameter("allow_motion", false);
    stop_at_target_ = declare_parameter("stop_at_target", false);
    approach_complete_when_stationary_ = declare_parameter("approach_complete_when_stationary", false);
    use_motor_ = declare_parameter("use_motor", false);
    skip_strike_ = declare_parameter("skip_strike", false);
    motor_return_delay_s_ = declare_parameter("motor_return_delay_s", 1.0);
    left_transform_calibrated_ = declare_parameter("left_transform_calibrated", false);
    right_transform_calibrated_ = declare_parameter("right_transform_calibrated", false);
    left_ticks_calibrated_ = declare_parameter("left_ticks_calibrated", false);
    right_ticks_calibrated_ = declare_parameter("right_ticks_calibrated", false);
    left_transform_ = declare_parameter<std::vector<double>>("left_camera_to_base", {});
    right_transform_ = declare_parameter<std::vector<double>>("right_camera_to_base", {});
    joint_names_ = declare_parameter<std::vector<std::string>>("driver_joint_names", {});
    native_zero_ = declare_parameter<std::vector<double>>("zero_ticks", {});
    urdf_zero_ = declare_parameter<std::vector<double>>("urdf_zero_ticks", {});
    ticks_per_radian_ = declare_parameter<std::vector<double>>("ticks_per_radian", {});
    directions_ = declare_parameter<std::vector<double>>("directions", {});
    min_ticks_ = declare_parameter<std::vector<int64_t>>("min_ticks", {});
    max_ticks_ = declare_parameter<std::vector<int64_t>>("max_ticks", {});
    left_lower_ = declare_parameter<std::vector<double>>("left_ik_lower_limits", {});
    left_upper_ = declare_parameter<std::vector<double>>("left_ik_upper_limits", {});
    right_lower_ = declare_parameter<std::vector<double>>("right_ik_lower_limits", {});
    right_upper_ = declare_parameter<std::vector<double>>("right_ik_upper_limits", {});
    // 速度调节集中在这里：依次为 joint1~joint5，左右臂共用。
    // 上方/下方由 target.slot 的 top/bottom 决定，与果实大小和颜色无关。
    // 前四项为 HX 速度；V3 第五关节用对应的 duration_ms（毫秒）调节。
    top_motion_ = declare_motion_profile("top", {50, 300, 500, 500, 20}, 5000);
    bottom_motion_ = declare_motion_profile("bottom", {50, 300, 500, 500, 20}, 5000);
    home_motion_ = declare_motion_profile("home", {50, 300, 500, 500, 20}, 5000);
    gripper_speed_ = declare_parameter<int64_t>("gripper_speed", 20);
    target_tool_tilt_ = declare_parameter("target_tool_tilt", 0.0);
    target_joint5_ = declare_parameter("target_joint5", 0.0);
    close_native_position_ = declare_parameter("gripper_close_native_rad", -2.0);
    pregrasp_native_position_ = declare_parameter("gripper_pregrasp_native_rad", -0.2);
    open_native_position_ = declare_parameter("gripper_open_native_rad", 0.0);
    use_size_profiles_ = declare_parameter("use_size_profiles", true);
    for (const auto &side : {std::string("left"), std::string("right")})
    {
      for (const bool small : {false, true})
      {
        const auto key = side + (small ? "_small" : "_large");
        const auto offset = declare_parameter<std::vector<double>>(
          key + "_offset_m", {0.0, 0.0, 0.0});
        if (offset.size() != 3U || !std::all_of(
            offset.begin(), offset.end(), [](double value) {return std::isfinite(value);}))
        {
          throw std::invalid_argument(key + "_offset_m must contain three finite XYZ values");
        }
        GraspProfile profile;
        std::copy(offset.begin(), offset.end(), profile.offset_m.begin());
        profile.pregrasp = declare_parameter(
          key + "_gripper_pregrasp_native_rad", small ? -1.0 : 0.0);
        profile.close = declare_parameter(
          key + "_gripper_close_native_rad", small ? -2.0 : -1.0);
        if (!std::isfinite(profile.pregrasp) || !std::isfinite(profile.close))
        {
          throw std::invalid_argument(key + " gripper positions must be finite");
        }
        grasp_profiles_.emplace(key, profile);
      }
    }
    gripper_duration_ms_ = declare_parameter<int64_t>("gripper_duration_ms", 2000);
    move_timeout_ms_ = declare_parameter<int64_t>("move_timeout_ms", 15000);
    home_timeout_ms_ = declare_parameter<int64_t>("home_timeout_ms", 15000);
    open_delay_ms_ = declare_parameter<int64_t>("open_delay_ms", 500);
    state_timeout_s_ = declare_parameter("joint_state_timeout_s", 2.0);
    stop_timeout_s_ = declare_parameter("stop_state_timeout_s", 0.6);
    harvest_timeout_s_ = declare_parameter("harvest_timeout_s", 10.0);
    if (!std::isfinite(harvest_timeout_s_) || harvest_timeout_s_ <= 0.0 ||
      harvest_timeout_s_ > 10.0)
    {
      throw std::invalid_argument("harvest_timeout_s must be in (0, 10] seconds");
    }

    validate_configuration();
    action_client_ = rclcpp_action::create_client<MoveJoints>(this, "/move_joints");
    if (use_motor_ && !skip_strike_)
    {
      motor_client_ = rclcpp_action::create_client<TriggerStrike>(this, "/end_effector/trigger_strike");
    }
    description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_robot_description(msg);});
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {on_joint_state(msg);});
    stop_sub_ = create_subscription<StopState>(
      "/base/vision_stop_state", 10,
      [this](const StopState::SharedPtr msg) {on_stop_state(msg);});
    targets_sub_ = create_subscription<HarvestArray>(
      "/vision/harvest_targets", 10,
      [this](const HarvestArray::SharedPtr msg) {on_targets(msg);});
    result_pub_ = create_publisher<robot_arm_interfaces::msg::HarvestResult>(
      "/harvest/result", rclcpp::QoS(10).reliable());
    worker_ = std::thread([this] {worker_loop();});
    RCLCPP_WARN(
      get_logger(),
      "Dual-arm harvest node started: waiting for URDF and fresh arm state; dry_run=%s allow_motion=%s stop_at_target=%s use_motor=%s skip_strike=%s",
      dry_run_ ? "true" : "false", allow_motion_ ? "true" : "false",
      stop_at_target_ ? "true" : "false", use_motor_ ? "true" : "false",
      skip_strike_ ? "true" : "false");
  }

  ~DualArmHarvestNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

private:
  MotionProfile declare_motion_profile(
    const std::string &name, const std::vector<int64_t> &speeds, int64_t duration_ms)
  {
    MotionProfile profile;
    profile.speeds = declare_parameter<std::vector<int64_t>>(name + "_arm_speeds", speeds);
    profile.duration_ms = declare_parameter<int64_t>(name + "_move_duration_ms", duration_ms);
    if (profile.speeds.size() != 5U || std::any_of(
        profile.speeds.begin(), profile.speeds.end(),
        [](int64_t speed) {return speed < 0 || speed > 3400;}) ||
      profile.duration_ms < 20 || profile.duration_ms > 30000)
    {
      throw std::invalid_argument(name + " motion profile needs 5 speeds in 0..3400 and duration in 20..30000 ms");
    }
    return profile;
  }

  void validate_configuration()
  {
    if (use_motor_ && (!std::isfinite(motor_return_delay_s_) ||
      motor_return_delay_s_ < 0.0 || motor_return_delay_s_ > 30.0))
    {
      throw std::invalid_argument("motor_return_delay_s must be in [0, 30] seconds");
    }
    const auto count = joint_names_.size();
    if (count != 12U || native_zero_.size() != count || urdf_zero_.size() != count ||
      ticks_per_radian_.size() != count || directions_.size() != count ||
      min_ticks_.size() != count || max_ticks_.size() != count)
    {
      throw std::invalid_argument("dual-arm driver calibration arrays must contain 12 entries");
    }
    if (left_lower_.size() != 5U || left_upper_.size() != 5U ||
      right_lower_.size() != 5U || right_upper_.size() != 5U ||
      gripper_speed_ < 0 || gripper_speed_ > 3400 || gripper_duration_ms_ < 20 ||
      move_timeout_ms_ < std::max(top_motion_.duration_ms, bottom_motion_.duration_ms) ||
      move_timeout_ms_ < gripper_duration_ms_ || home_timeout_ms_ < home_motion_.duration_ms ||
      gripper_duration_ms_ > 30000 || move_timeout_ms_ > 30000 ||
      home_timeout_ms_ > 30000 ||
      home_timeout_ms_ < open_delay_ms_ + gripper_duration_ms_ ||
      open_delay_ms_ < 0 || open_delay_ms_ > 5000 ||
      !std::isfinite(target_tool_tilt_) || !std::isfinite(target_joint5_) ||
      !std::isfinite(pregrasp_native_position_) || !std::isfinite(close_native_position_) ||
      !std::isfinite(open_native_position_) ||
      !std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0 ||
      !std::isfinite(stop_timeout_s_) || stop_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid dual-arm IK, speed, duration or state-timeout configuration");
    }
    validate_transform(left_transform_);
    validate_transform(right_transform_);
    for (size_t i = 0; i < count; ++i)
    {
      if (joint_names_[i].empty() || ticks_per_radian_[i] <= 0.0 ||
        std::abs(directions_[i]) != 1.0 || !std::isfinite(native_zero_[i]) ||
        !std::isfinite(urdf_zero_[i]) || !std::isfinite(ticks_per_radian_[i]))
      {
        throw std::invalid_argument("invalid per-servo calibration in dual-arm driver parameters");
      }
      if (!driver_index_.emplace(joint_names_[i], i).second)
      {
        throw std::invalid_argument("driver_joint_names must be unique");
      }
    }
    for (const auto side : {std::string("left"), std::string("right")})
    {
      for (int joint = 1; joint <= 5; ++joint)
      {
        const auto name = side + "_joint" + std::to_string(joint);
        if (!driver_index_.count(name))
        {
          throw std::invalid_argument("missing configured driver joint " + name);
        }
      }
      if (!driver_index_.count(side + "_gripper"))
      {
        throw std::invalid_argument("missing configured gripper for " + side);
      }
    }
  }

  static void validate_transform(const std::vector<double> &m)
  {
    if (m.size() != 16U ||
      !std::all_of(m.begin(), m.end(), [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument("each camera_to_base transform must contain 16 finite values");
    }
    if (std::abs(m[12]) > 1e-6 || std::abs(m[13]) > 1e-6 ||
      std::abs(m[14]) > 1e-6 || std::abs(m[15] - 1.0) > 1e-6)
    {
      throw std::invalid_argument("camera transform has an invalid homogeneous last row");
    }
    for (size_t i = 0; i < 3U; ++i)
    {
      for (size_t j = 0; j < 3U; ++j)
      {
        double dot = 0.0;
        for (size_t k = 0; k < 3U; ++k)
        {
          dot += m[k * 4U + i] * m[k * 4U + j];
        }
        if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-4)
        {
          throw std::invalid_argument("camera transform rotation must be orthonormal");
        }
      }
    }
    const double det =
      m[0] * (m[5] * m[10] - m[6] * m[9]) -
      m[1] * (m[4] * m[10] - m[6] * m[8]) +
      m[2] * (m[4] * m[9] - m[5] * m[8]);
    if (std::abs(det - 1.0) > 1e-4)
    {
      throw std::invalid_argument("camera transform rotation must be proper");
    }
  }

  void on_robot_description(const std_msgs::msg::String::SharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    if (left_kinematics_)
    {
      return;
    }
    try
    {
      left_kinematics_ = std::make_unique<motion::UrdfKinematics>(
        msg->data, "left_base_link", "left_tcp_link", 200U, 1e-3, left_lower_, left_upper_);
      right_kinematics_ = std::make_unique<motion::UrdfKinematics>(
        msg->data, "right_base_link", "right_tcp_link", 200U, 1e-3, right_lower_, right_upper_);
      if (left_kinematics_->joint_count() != 5U || right_kinematics_->joint_count() != 5U)
      {
        left_kinematics_.reset();
        right_kinematics_.reset();
        throw std::runtime_error("the shared URDF chain must have exactly five movable arm joints");
      }
      RCLCPP_INFO(get_logger(),
        "Loaded dual-arm URDF: left_base_link -> left_tcp_link; right_base_link -> right_tcp_link");
    }
    catch (const std::exception &e)
    {
      left_kinematics_.reset();
      right_kinematics_.reset();
      RCLCPP_ERROR(get_logger(), "Failed to initialize dual-arm IK: %s", e.what());
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr &msg)
  {
    if (msg->name.size() != msg->position.size())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::unordered_set<std::string> received_names;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (std::isfinite(msg->position[i]))
      {
        current_state_[msg->name[i]] = msg->position[i];
        received_names.insert(msg->name[i]);
      }
    }
    const bool complete = std::all_of(
      joint_names_.begin(), joint_names_.end(), [this, &received_names](const auto &name) {
        return (use_motor_ && (name == "left_gripper" || name == "right_gripper")) ||
               received_names.count(name) != 0U;
      });
    if (complete)
    {
      state_received_ = std::chrono::steady_clock::now();
    }
  }

  bool startup_ready(std::string &reason)
  {
    {
      std::lock_guard<std::mutex> lock(model_mutex_);
      if (!left_kinematics_ || !right_kinematics_)
      {
        reason = "URDF kinematics has not loaded";
        return false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (state_received_ == std::chrono::steady_clock::time_point{} ||
        now - state_received_ > std::chrono::duration<double>(state_timeout_s_))
      {
        reason = use_motor_ ? "fresh /joint_states for all 10 arm joints are not available" :
          "fresh complete /joint_states for all 12 joints are not available";
        return false;
      }
      for (const auto &name : joint_names_)
      {
        if (use_motor_ && (name == "left_gripper" || name == "right_gripper"))
        {
          continue;
        }
        if (current_state_.find(name) == current_state_.end())
        {
          reason = "missing /joint_states entry for " + name;
          return false;
        }
      }
    }
    return true;
  }

  void on_stop_state(const StopState::SharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (msg->state == StopState::HOLDING &&
      (msg->stop_id != holding_stop_id_ || holding_started_ == std::chrono::steady_clock::time_point{}))
    {
      holding_started_ = std::chrono::steady_clock::now();
    }
    holding_ = msg->state == StopState::HOLDING;
    holding_stage_ = msg->stage_id;
    holding_stop_id_ = msg->stop_id;
    stop_received_ = std::chrono::steady_clock::now();
  }

  bool holding_for(uint8_t stage, uint64_t stop_id = 0) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return holding_ && holding_stage_ == stage &&
      (stop_id == 0 || holding_stop_id_ == stop_id) &&
      std::chrono::steady_clock::now() - stop_received_ <
      std::chrono::duration<double>(stop_timeout_s_);
  }

  void on_targets(const HarvestArray::SharedPtr &msg)
  {
    if (!msg->valid || (msg->targets.empty() && msg->stop_id == 0))
    {
      return;
    }
    if (msg->header.frame_id != "camera_color_optical_frame")
    {
      RCLCPP_WARN(
        get_logger(), "Ignoring harvest snapshot in frame '%s'; expected camera_color_optical_frame",
        msg->header.frame_id.c_str());
      return;
    }
    std::string readiness_reason;
    if (!startup_ready(readiness_reason))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring harvest snapshot until startup is ready: %s", readiness_reason.c_str());
      return;
    }
    if (!holding_for(msg->stage_id))
    {
      RCLCPP_WARN(get_logger(), "Ignoring harvest snapshot because matching base HOLDING is not fresh");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (msg->stop_id != holding_stop_id_) {return;}
    }
    const auto stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
      msg->header.stamp.nanosec;
    const auto key = std::to_string(msg->stage_id) + ":" + std::to_string(stamp_ns);
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // A SETTLING/HOLDING fluctuation may cause vision to freeze a new stamp.
    // Automatic operation still accepts only one batch per parking session.
    if (msg->stop_id != 0 && accepted_stop_ids_.count(msg->stop_id)) {return;}
    if (key == last_snapshot_key_)
    {
      return;
    }
    last_snapshot_key_ = key;
    if (queue_.size() >= 2U)
    {
      RCLCPP_WARN(get_logger(), "Harvest queue full; dropping a newer frozen target snapshot");
      return;
    }
    if (msg->stop_id != 0) {accepted_stop_ids_.insert(msg->stop_id);}
    queue_.push_back(*msg);
    queue_cv_.notify_one();
  }

  std::vector<double> seed_for(const std::vector<std::string> &model_names)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_state_.empty() ||
      std::chrono::steady_clock::now() - state_received_ >
      std::chrono::duration<double>(state_timeout_s_))
    {
      throw std::runtime_error("fresh /joint_states are required before IK");
    }
    std::vector<double> seed;
    seed.reserve(model_names.size());
    for (const auto &name : model_names)
    {
      const auto it = current_state_.find(name);
      if (it == current_state_.end())
      {
        throw std::runtime_error("missing measured joint state " + name);
      }
      seed.push_back(it->second);
    }
    return seed;
  }

  std::array<double, 3> to_base(
    const std::vector<double> &matrix, const geometry_msgs::msg::Point &point) const
  {
    const auto x = point.x;
    const auto y = point.y;
    const auto z = point.z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      throw std::runtime_error("camera target contains non-finite coordinates");
    }
    return {
      matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
      matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
      matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11]};
  }

  ArmSolution solve(
    const std::string &side, const robot_arm_interfaces::msg::HarvestTarget &target)
  {
    const auto &camera_point = target.surface_point;
    const auto profile_key = side + (target.is_small ? "_small" : "_large");
    const auto &profile = grasp_profiles_.at(profile_key);
    motion::UrdfKinematics *model = nullptr;
    {
      std::lock_guard<std::mutex> lock(model_mutex_);
      model = side == "left" ? left_kinematics_.get() : right_kinematics_.get();
    }
    if (!model)
    {
      throw std::runtime_error("URDF kinematics is not ready");
    }
    auto point = to_base(
      side == "left" ? left_transform_ : right_transform_, camera_point);
    // Apply the selected fruit offset in this arm's base frame, before IK.
    for (size_t i = 0; i < point.size(); ++i)
    {
      point[i] += profile.offset_m[i];
    }

    const auto seed = seed_for(model->joint_names());
    ArmSolution solution;
    if (target.slot != side + "_top" && target.slot != side + "_bottom")
    {
      throw std::runtime_error("cannot select top/bottom motion profile for slot: " + target.slot);
    }
    solution.upper_target = target.slot == side + "_top";
    solution.gripper_pregrasp = use_size_profiles_ ? profile.pregrasp : pregrasp_native_position_;
    solution.gripper_close = use_size_profiles_ ? profile.close : close_native_position_;
    RCLCPP_INFO(
      get_logger(),
      "Grasp profile=%s color=%s offset_base_m=[%.3f %.3f %.3f] pregrasp=%.3f close=%.3f rad duration=%lld ms size_profiles=%s",
      profile_key.c_str(), target.color.c_str(), profile.offset_m[0], profile.offset_m[1],
      profile.offset_m[2], solution.gripper_pregrasp, solution.gripper_close,
      static_cast<long long>(gripper_duration_ms_), use_size_profiles_ ? "true" : "false");
    solution.urdf = model->inverse_xyz_tilt_joint5_analytic(
      seed, point[0], point[1], point[2], target_tool_tilt_, target_joint5_);
    solution.native.reserve(solution.urdf.size());
    for (size_t i = 0; i < solution.urdf.size(); ++i)
    {
      const auto &driver_name = model->joint_names()[i];
      const auto index = driver_index_.at(driver_name);
      const auto scale = directions_[index] * ticks_per_radian_[index];
      const auto tick = urdf_zero_[index] + scale * solution.urdf[i];
      if (!std::isfinite(tick) || tick < static_cast<double>(min_ticks_[index]) ||
        tick > static_cast<double>(max_ticks_[index]))
      {
        throw std::runtime_error("IK solution exceeds calibrated ticks for " + driver_name);
      }
      solution.native.push_back(
        solution.urdf[i] + (urdf_zero_[index] - native_zero_[index]) / scale);
    }
    RCLCPP_INFO(
      get_logger(),
      "IK %s target camera=(%.3f, %.3f, %.3f) base=(%.3f, %.3f, %.3f), q_urdf=[%.3f %.3f %.3f %.3f %.3f]",
      side.c_str(), camera_point.x, camera_point.y, camera_point.z,
      point[0], point[1], point[2], solution.urdf[0], solution.urdf[1],
      solution.urdf[2], solution.urdf[3], solution.urdf[4]);
    return solution;
  }

  void append_joint(
    MoveJoints::Goal &goal, const std::string &name, double position,
    uint16_t speed, uint16_t delay_ms, uint16_t duration_ms)
  {
    goal.joint_names.push_back(name);
    goal.positions.push_back(position);
    goal.speeds.push_back(speed);
    goal.start_delays_ms.push_back(delay_ms);
    goal.durations_ms.push_back(duration_ms);
  }

  void set_goal_duration(MoveJoints::Goal &goal, int64_t timeout_ms)
  {
    goal.duration.sec = static_cast<int32_t>(timeout_ms / 1000);
    goal.duration.nanosec = static_cast<uint32_t>((timeout_ms % 1000) * 1000000);
    goal.wait_for_completion = true;
  }

  void execute_goal(MoveJoints::Goal goal, int64_t timeout_ms, const std::string &stage)
  {
    check_harvest_deadline();
    if (dry_run_ || !allow_motion_)
    {
      throw std::runtime_error("physical action requested while dry_run or allow_motion interlock is active");
    }
    if (!action_client_->wait_for_action_server(3s))
    {
      throw std::runtime_error("/move_joints action server is unavailable");
    }
    check_harvest_deadline();
    set_goal_duration(goal, timeout_ms);
    goal.hold_on_cancel = enforce_deadline_;
    for (size_t i = 0; i < goal.joint_names.size(); ++i)
    {
      RCLCPP_INFO(
        get_logger(), "%s: %s native=%.4f rad speed=%u duration=%u ms delay=%u ms",
        stage.c_str(), goal.joint_names[i].c_str(), goal.positions[i],
        static_cast<unsigned>(goal.speeds[i]), static_cast<unsigned>(goal.durations_ms[i]),
        static_cast<unsigned>(goal.start_delays_ms[i]));
    }
    arm_goal_unconfirmed_ = true;
    auto sent = action_client_->async_send_goal(goal);
    if (sent.wait_for(5s) != std::future_status::ready)
    {
      throw std::runtime_error(stage + ": timed out waiting for action acceptance");
    }
    const auto handle = sent.get();
    if (!handle)
    {
      arm_goal_unconfirmed_ = false;
      throw GoalRejected(stage + ": /move_joints rejected the goal");
    }
    auto result_future = action_client_->async_get_result(handle);
    const auto action_deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(timeout_ms + 5000);
    while (result_future.wait_for(20ms) != std::future_status::ready)
    {
      if ((enforce_deadline_ && std::chrono::steady_clock::now() >= harvest_deadline_) ||
        std::chrono::steady_clock::now() >= action_deadline)
      {
        action_client_->async_cancel_goal(handle);
        // Do not overlap the return-home action with an uncancelled arm goal.
        if (result_future.wait_for(2s) != std::future_status::ready)
        {
          throw std::runtime_error(stage + ": cancellation not confirmed; base remains stopped");
        }
        const auto cancelled = result_future.get();
        if (cancelled.code != rclcpp_action::ResultCode::CANCELED &&
          cancelled.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
          throw std::runtime_error(stage + ": safe cancellation failed; base remains stopped");
        }
        arm_goal_unconfirmed_ = false;
        check_harvest_deadline();
        throw std::runtime_error(stage + ": timed out waiting for motion completion");
      }
    }
    const auto wrapped = result_future.get();
    arm_goal_unconfirmed_ = false;
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      !wrapped.result->success)
    {
      const auto detail = wrapped.result ? wrapped.result->message : "no action result";
      throw std::runtime_error(stage + ": " + detail);
    }
    check_harvest_deadline();
    const auto &state = wrapped.result->final_state;
    if (state.name.size() == state.position.size())
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (size_t i = 0; i < state.name.size(); ++i)
      {
        current_state_[state.name[i]] = state.position[i];
      }
      state_received_ = std::chrono::steady_clock::now();
    }
  }

  void wait_without_strike(uint8_t stage)
  {
    RCLCPP_WARN(
      get_logger(), "Strike skipped: no serial trigger; waiting %.3f s before returning home",
      motor_return_delay_s_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(motor_return_delay_s_);
    do
    {
      check_harvest_deadline();
      if (!rclcpp::ok() || !holding_for(stage))
      {
        throw std::runtime_error("strike skipped: shutdown or matching base HOLDING lost during local wait");
      }
      if (std::chrono::steady_clock::now() >= deadline)
      {
        return;
      }
      std::this_thread::sleep_for(20ms);
    } while (true);
  }

  void execute_motors(const std::vector<std::string> &sides)
  {
    if (dry_run_ || !allow_motion_)
    {
      throw std::runtime_error("motor motion is disabled by dry_run/allow_motion");
    }
    TriggerStrike::Goal goal;
    goal.arms = sides;
    goal.wait_s = motor_return_delay_s_;
    RCLCPP_INFO(get_logger(), "Trigger STM32 strike for %zu arm(s); local return delay=%.3f s; no MCU feedback",
      sides.size(), goal.wait_s);
    auto sent = motor_client_->async_send_goal(goal);
    if (sent.wait_for(5s) != std::future_status::ready)
    {
      motor_client_->async_cancel_all_goals();
      throw std::runtime_error("timed out waiting for STM32 strike action acceptance; trigger not retried");
    }
    const auto handle = sent.get();
    if (!handle)
    {
      throw std::runtime_error("STM32 strike server rejected the goal; arm remains at target");
    }
    auto future = motor_client_->async_get_result(handle);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(goal.wait_s + 5.0);
    while (future.wait_for(20ms) != std::future_status::ready)
    {
      if (enforce_deadline_ && std::chrono::steady_clock::now() >= harvest_deadline_)
      {
        motor_client_->async_cancel_goal(handle);
        throw HarvestTimeout{};
      }
      if (std::chrono::steady_clock::now() >= deadline)
      {
        motor_client_->async_cancel_goal(handle);
        throw std::runtime_error("STM32 strike action timed out; MCU completion unverified");
      }
    }
    const auto wrapped = future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped.result || !wrapped.result->success)
    {
      throw std::runtime_error("STM32 strike failed: " +
        (wrapped.result ? wrapped.result->message : std::string("no result")));
    }
  }

  MoveJoints::Goal make_target_goal(const std::map<std::string, ArmSolution> &solutions)
  {
    MoveJoints::Goal goal;
    goal.complete_when_stationary = approach_complete_when_stationary_;
    for (const auto &[side, solution] : solutions)
    {
      const auto &motion = solution.upper_target ? top_motion_ : bottom_motion_;
      RCLCPP_INFO(get_logger(),
        "Motion profile=%s_%s speeds=[%lld %lld %lld %lld %lld] V3_duration=%lld ms",
        side.c_str(), solution.upper_target ? "top" : "bottom",
        static_cast<long long>(motion.speeds[0]), static_cast<long long>(motion.speeds[1]),
        static_cast<long long>(motion.speeds[2]), static_cast<long long>(motion.speeds[3]),
        static_cast<long long>(motion.speeds[4]), static_cast<long long>(motion.duration_ms));
      for (size_t i = 0; i < 5U; ++i)
      {
        const auto name = side + "_joint" + std::to_string(i + 1U);
        append_joint(
          goal, name, solution.native[i], static_cast<uint16_t>(motion.speeds[i]),
          0U, static_cast<uint16_t>(motion.duration_ms));
      }
      if (!stop_at_target_ && !use_motor_)
      {
        append_joint(
          goal, side + "_gripper", solution.gripper_pregrasp,
          static_cast<uint16_t>(gripper_speed_), 0U,
          static_cast<uint16_t>(gripper_duration_ms_));
      }
    }
    return goal;
  }

  MoveJoints::Goal make_gripper_goal(const std::map<std::string, ArmSolution> &solutions)
  {
    MoveJoints::Goal goal;
    for (const auto &[side, solution] : solutions)
    {
      append_joint(
        goal, side + "_gripper", solution.gripper_close, static_cast<uint16_t>(gripper_speed_),
        0U, static_cast<uint16_t>(gripper_duration_ms_));
    }
    return goal;
  }

  MoveJoints::Goal make_home_goal(const std::vector<std::string> &sides)
  {
    MoveJoints::Goal goal;
    for (const auto &side : sides)
    {
      for (size_t i = 0; i < 5U; ++i)
      {
        const auto name = side + "_joint" + std::to_string(i + 1U);
        const auto delay = 0U;
        append_joint(
          goal, name, 0.0, static_cast<uint16_t>(home_motion_.speeds[i]), delay,
          static_cast<uint16_t>(home_motion_.duration_ms));
      }
      if (!use_motor_)
      {
        append_joint(
          goal, side + "_gripper", open_native_position_,
          static_cast<uint16_t>(gripper_speed_), static_cast<uint16_t>(open_delay_ms_),
          static_cast<uint16_t>(gripper_duration_ms_));
      }
    }
    return goal;
  }

  void return_home(const HarvestArray &snapshot, const std::vector<std::string> &sides,
    const std::string &stage)
  {
    // The 10-second picking budget must not cancel a return already in progress.
    // Home has its own finite timeout and must succeed before releasing the base.
    const bool picking_deadline = enforce_deadline_;
    enforce_deadline_ = false;
    try
    {
      for (unsigned attempt = 0; ; ++attempt)
      {
        if (!holding_for(snapshot.stage_id, snapshot.stop_id))
        {
          throw std::runtime_error("matching parking confirmation lost before return home");
        }
        try
        {
          execute_goal(make_home_goal(sides), home_timeout_ms_, stage);
          break;
        }
        catch (const GoalRejected &)
        {
          // Rejection dispatched no movement. Allow idle polling to recover
          // from a lost reply or a just-finishing action before trying again.
          if (attempt >= 2) {throw;}
          RCLCPP_WARN(get_logger(), "%s: retrying rejected home goal (%u/3)",
            stage.c_str(), attempt + 2);
          std::this_thread::sleep_for(200ms);
        }
      }
    }
    catch (...)
    {
      enforce_deadline_ = picking_deadline;
      throw;
    }
    enforce_deadline_ = picking_deadline;
  }

  void process_snapshot(const HarvestArray &snapshot)
  {
    if (!holding_for(snapshot.stage_id, snapshot.stop_id))
    {
      throw std::runtime_error("base left HOLDING or stop-state became stale");
    }
    std::map<std::string, std::vector<robot_arm_interfaces::msg::HarvestTarget>> grouped;
    grouped["left"] = {};
    grouped["right"] = {};
    for (const auto &target : snapshot.targets)
    {
      std::string side;
      if (target.slot.rfind("left_", 0) == 0)
      {
        side = "left";
      }
      else if (target.slot.rfind("right_", 0) == 0)
      {
        side = "right";
      }
      else
      {
        throw std::runtime_error("unsupported vision slot: " + target.slot);
      }
      grouped[side].push_back(target);
    }
    const auto slot_order = [](const auto &target) {
        if (target.slot.find("_bottom") != std::string::npos) {return 0;}
        if (target.slot.find("_top") != std::string::npos) {return 1;}
        return 2;
      };
    std::sort(grouped["left"].begin(), grouped["left"].end(),
      [&](const auto &a, const auto &b) {return slot_order(a) < slot_order(b);});
    std::sort(grouped["right"].begin(), grouped["right"].end(),
      [&](const auto &a, const auto &b) {return slot_order(a) < slot_order(b);});
    const auto rounds = std::max(grouped["left"].size(), grouped["right"].size());
    RCLCPP_INFO(
      get_logger(), "Processing stage %u: left=%zu, right=%zu, rounds=%zu",
      static_cast<unsigned>(snapshot.stage_id), grouped["left"].size(), grouped["right"].size(), rounds);
    for (size_t round = 0; round < rounds; ++round)
    {
      check_harvest_deadline();
      if (!holding_for(snapshot.stage_id, snapshot.stop_id))
      {
        throw std::runtime_error("base no longer confirms HOLDING; stopping this harvest batch");
      }
      std::map<std::string, ArmSolution> solutions;
      if (round < grouped["left"].size())
      {
        solutions.emplace("left", solve("left", grouped["left"][round]));
      }
      if (round < grouped["right"].size())
      {
        solutions.emplace("right", solve("right", grouped["right"][round]));
      }
      if (dry_run_)
      {
        RCLCPP_WARN(
          get_logger(), "DRY RUN round %zu: no commands sent; stop_at_target=%s",
          round + 1U, stop_at_target_ ? "true" : "false");
        continue;
      }
      if (!allow_motion_)
      {
        throw std::runtime_error("allow_motion is false; refusing physical motion");
      }
      std::vector<std::string> sides;
      for (const auto &[side, solution] : solutions)
      {
        (void)solution;
        if ((side == "left" && (!left_transform_calibrated_ || !left_ticks_calibrated_)) ||
          (side == "right" && (!right_transform_calibrated_ || !right_ticks_calibrated_)))
        {
          throw std::runtime_error(
            side + " camera transform and tick zero/direction must both be marked calibrated");
        }
        sides.push_back(side);
      }
      if (use_motor_ && !skip_strike_ && !stop_at_target_ &&
        !motor_client_->wait_for_action_server(3s))
      {
        throw std::runtime_error("STM32 strike server unavailable; refusing to move arm to target");
      }
      execute_goal(make_target_goal(solutions), move_timeout_ms_, "move to IK target");
      if (stop_at_target_)
      {
        RCLCPP_INFO(
          get_logger(), "Round %zu target pose reached; holding position, grippers unchanged",
          round + 1U);
        continue;
      }
      if (use_motor_)
      {
        if (skip_strike_)
        {
          wait_without_strike(snapshot.stage_id);
          return_home(snapshot, sides, "return home after skipped strike delay");
          RCLCPP_INFO(get_logger(), "Round %zu complete; strike skipped; arm returned home", round + 1U);
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Target pose reached; sending STM32 strike trigger");
          execute_motors(sides);
          return_home(snapshot, sides, "return home after STM32 strike delay");
          RCLCPP_INFO(get_logger(), "Round %zu complete; local strike delay elapsed; MCU completion unverified",
            round + 1U);
        }
        continue;
      }
      RCLCPP_INFO(get_logger(), "Target pose reached; closing %zu gripper(s)", sides.size());
      execute_goal(make_gripper_goal(solutions), move_timeout_ms_, "close gripper");
      return_home(snapshot, sides, "return home and release");
      RCLCPP_INFO(get_logger(), "Round %zu complete; gripper opened during return", round + 1U);
    }
  }

  void check_harvest_deadline() const
  {
    if (enforce_deadline_ && std::chrono::steady_clock::now() >= harvest_deadline_)
    {
      throw HarvestTimeout{};
    }
  }

  void publish_result(const HarvestArray &snapshot, bool success, const std::string &detail)
  {
    if (snapshot.stop_id == 0) {return;}  // Legacy manual tests cannot release navigation.
    robot_arm_interfaces::msg::HarvestResult result;
    result.header.stamp = now();
    result.stage_id = snapshot.stage_id;
    result.stop_id = snapshot.stop_id;
    result.snapshot_stamp = snapshot.header.stamp;
    result.success = success;
    result.returned_home = success && !dry_run_ && !stop_at_target_;
    result.dry_run = dry_run_;
    result.message = detail;
    result_pub_->publish(result);
  }

  void recover_home(const HarvestArray &snapshot)
  {
    enforce_deadline_ = false;  // Recovery has its own finite home_timeout_ms.
    if (!holding_for(snapshot.stage_id, snapshot.stop_id))
    {
      throw std::runtime_error("matching parking confirmation lost; refusing recovery motion");
    }
    if (!left_ticks_calibrated_ || !right_ticks_calibrated_)
    {
      throw std::runtime_error("both arms need calibrated ticks for timeout recovery");
    }
    if (arm_goal_unconfirmed_)
    {
      throw std::runtime_error("previous arm action is not confirmed stopped; refusing overlapping home action");
    }
    return_home(snapshot, {"left", "right"},
      "harvest budget expired or target skipped; return both arms home");
    if (!holding_for(snapshot.stage_id, snapshot.stop_id))
    {
      throw std::runtime_error("home reached but fresh parking confirmation lost; refusing resume");
    }
  }

  void worker_loop()
  {
    for (;;)
    {
      HarvestArray snapshot;
      bool timed_out_without_targets = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait_for(lock, 50ms, [this] {return stopping_ || !queue_.empty();});
        if (stopping_)
        {
          return;
        }
        if (queue_.empty())
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          if (dry_run_ || !allow_motion_ || stop_at_target_ || !holding_ ||
            holding_stop_id_ == 0 || holding_started_ == std::chrono::steady_clock::time_point{} ||
            std::chrono::steady_clock::now() - holding_started_ <
            std::chrono::duration<double>(harvest_timeout_s_) ||
            !accepted_stop_ids_.insert(holding_stop_id_).second)
          {
            continue;
          }
          snapshot.stage_id = holding_stage_;
          snapshot.stop_id = holding_stop_id_;
          timed_out_without_targets = true;
        }
        else
        {
          snapshot = std::move(queue_.front());
          queue_.pop_front();
        }
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        harvest_deadline_ = holding_started_ +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(harvest_timeout_s_));
      }
      enforce_deadline_ = !dry_run_ && !stop_at_target_ && snapshot.stop_id != 0;
      try
      {
        if (timed_out_without_targets) {throw HarvestTimeout{};}
        process_snapshot(snapshot);
        if (!dry_run_ && allow_motion_ && !stop_at_target_)
        {
          // Empty valid selections also need a verified home before continuing.
          if (snapshot.targets.empty()) {recover_home(snapshot);}
          if (!holding_for(snapshot.stage_id, snapshot.stop_id))
          {
            throw std::runtime_error("matching parking session lost after home; refusing resume");
          }
          publish_result(snapshot, true, "all harvest rounds complete; arms returned home");
          RCLCPP_INFO(get_logger(), "Harvest stage %u stop %llu complete; resume notified",
            snapshot.stage_id, static_cast<unsigned long long>(snapshot.stop_id));
        }
      }
      catch (const HarvestTimeout &)
      {
        RCLCPP_WARN(get_logger(), "Harvest stage %u exceeded %.1f s; skip remaining targets and return home",
          snapshot.stage_id, harvest_timeout_s_);
        try
        {
          recover_home(snapshot);
          publish_result(snapshot, true, "harvest timed out; remaining targets skipped; arms home");
        }
        catch (const std::exception &e)
        {
          publish_result(snapshot, false, e.what());
          RCLCPP_ERROR(get_logger(), "Timeout return home failed; base remains stopped: %s", e.what());
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(get_logger(), "Harvest stage %u skipped: %s", snapshot.stage_id, e.what());
        try
        {
          if (snapshot.stop_id == 0 || dry_run_ || !allow_motion_ || stop_at_target_) {throw;}
          recover_home(snapshot);
          publish_result(snapshot, true, std::string("target skipped; arms home: ") + e.what());
        }
        catch (const std::exception &recovery_error)
        {
          publish_result(snapshot, false, recovery_error.what());
          RCLCPP_ERROR(get_logger(), "Harvest recovery failed; base remains stopped: %s", recovery_error.what());
        }
      }
      enforce_deadline_ = false;
    }
  }

  bool dry_run_{true};
  bool allow_motion_{false};
  bool stop_at_target_{false};
  bool approach_complete_when_stationary_{false};
  bool use_motor_{false};
  bool skip_strike_{false};
  double motor_return_delay_s_{1.0};
  bool left_transform_calibrated_{false};
  bool right_transform_calibrated_{false};
  bool left_ticks_calibrated_{false};
  bool right_ticks_calibrated_{false};
  std::vector<double> left_transform_;
  std::vector<double> right_transform_;
  std::vector<std::string> joint_names_;
  std::vector<double> native_zero_;
  std::vector<double> urdf_zero_;
  std::vector<double> ticks_per_radian_;
  std::vector<double> directions_;
  std::vector<int64_t> min_ticks_;
  std::vector<int64_t> max_ticks_;
  std::vector<double> left_lower_;
  std::vector<double> left_upper_;
  std::vector<double> right_lower_;
  std::vector<double> right_upper_;
  MotionProfile top_motion_;
  MotionProfile bottom_motion_;
  MotionProfile home_motion_;
  int64_t gripper_speed_{20};
  double target_tool_tilt_{0.0};
  double target_joint5_{0.0};
  double close_native_position_{-2.0};
  double pregrasp_native_position_{-0.2};
  double open_native_position_{0.0};
  bool use_size_profiles_{true};
  std::map<std::string, GraspProfile> grasp_profiles_;
  int64_t gripper_duration_ms_{2000};
  int64_t move_timeout_ms_{15000};
  int64_t home_timeout_ms_{15000};
  int64_t open_delay_ms_{500};
  double state_timeout_s_{2.0};
  double stop_timeout_s_{0.6};
  double harvest_timeout_s_{10.0};
  bool enforce_deadline_{false};  // Worker-thread only.
  bool arm_goal_unconfirmed_{false};
  std::chrono::steady_clock::time_point harvest_deadline_{};
  std::unordered_map<std::string, size_t> driver_index_;

  std::mutex model_mutex_;
  std::unique_ptr<motion::UrdfKinematics> left_kinematics_;
  std::unique_ptr<motion::UrdfKinematics> right_kinematics_;
  //std::mutex state_mutex_;
  mutable std::mutex state_mutex_;
  std::unordered_map<std::string, double> current_state_;
  std::chrono::steady_clock::time_point state_received_{};
  bool holding_{false};
  uint8_t holding_stage_{0};
  uint64_t holding_stop_id_{0};
  std::chrono::steady_clock::time_point holding_started_{};
  std::chrono::steady_clock::time_point stop_received_{};
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<HarvestArray> queue_;
  std::string last_snapshot_key_;
  std::unordered_set<uint64_t> accepted_stop_ids_;
  bool stopping_{false};
  std::thread worker_;

  rclcpp_action::Client<MoveJoints>::SharedPtr action_client_;
  rclcpp_action::Client<TriggerStrike>::SharedPtr motor_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<StopState>::SharedPtr stop_sub_;
  rclcpp::Subscription<HarvestArray>::SharedPtr targets_sub_;
  rclcpp::Publisher<robot_arm_interfaces::msg::HarvestResult>::SharedPtr result_pub_;
};
}  // namespace robot_arm_node::control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_arm_node::control::DualArmHarvestNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
