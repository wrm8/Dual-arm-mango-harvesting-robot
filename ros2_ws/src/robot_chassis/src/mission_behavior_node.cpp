#include "robot_chassis/mission_behavior_node.hpp"

#include "robot_arm_interfaces/action/follow_row.hpp"
#include "robot_arm_interfaces/action/pick_point.hpp"
#include "robot_arm_interfaces/action/shift_lane.hpp"
#include "robot_arm_interfaces/action/stop_base.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace robot_chassis
{
namespace
{
using FollowRow = robot_arm_interfaces::action::FollowRow;
using ShiftLane = robot_arm_interfaces::action::ShiftLane;
using StopBase = robot_arm_interfaces::action::StopBase;
using PickPoint = robot_arm_interfaces::action::PickPoint;
using FollowClient = rclcpp_action::Client<FollowRow>;
using ShiftClient = rclcpp_action::Client<ShiftLane>;
using StopClient = rclcpp_action::Client<StopBase>;
using PickClient = rclcpp_action::Client<PickPoint>;
using FollowGoalHandle = FollowClient::GoalHandle;
using ShiftGoalHandle = ShiftClient::GoalHandle;
using StopGoalHandle = StopClient::GoalHandle;
using PickGoalHandle = PickClient::GoalHandle;

MissionBehaviorNode *mission_node(const BT::NodeConfiguration &config)
{
  return config.blackboard->get<MissionBehaviorNode *>("mission_node");
}

bool succeeded(const rclcpp_action::ResultCode code)
{
  return code == rclcpp_action::ResultCode::SUCCEEDED;
}

const char *default_tree_xml()
{
  return R"xml(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveFallback name="mango_interrupt_has_priority">
      <Sequence name="mango_branch">
        <MangoReady/>
        <StopBase/>
        <PickMango/>
        <!-- This node marks the target handled and deliberately returns FAILURE.
             That makes the fallback continue with RouteStep in this tick. -->
        <MarkMangoHandled/>
      </Sequence>
      <RouteStep/>
    </ReactiveFallback>
  </BehaviorTree>
</root>
)xml";
}

class MangoReady final : public BT::ConditionNode
{
public:
  MangoReady(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), node_(mission_node(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return node_->latch_mango_if_ready() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  MissionBehaviorNode *node_;
};

class RouteStep final : public BT::StatefulActionNode
{
public:
  RouteStep(const std::string &name, const BT::NodeConfiguration &config)
  : BT::StatefulActionNode(name, config),
    node_(mission_node(config)),
    follow_client_(rclcpp_action::create_client<FollowRow>(node_, "follow_row")),
    shift_client_(rclcpp_action::create_client<ShiftLane>(node_, "shift_lane")) {}

  static BT::PortsList providedPorts() { return {}; }

private:
  enum class Phase { Follow, Shift };

  BT::NodeStatus onStart() override
  {
    std::scoped_lock lock(mutex_);
    halted_ = false;
    result_ready_ = false;
    goal_pending_ = false;
    send_in_flight_ = false;
    cancel_requested_ = false;
    if (node_->mission_fault()) {
      return BT::NodeStatus::FAILURE;
    }
    if (node_->route_complete()) {
      return BT::NodeStatus::SUCCESS;
    }
    phase_ = node_->next_action_is_shift() ? Phase::Shift : Phase::Follow;
    if (!send_current_goal_locked()) {
      // A missing action server is a normal startup condition. Keep the tree
      // alive and retry on subsequent ticks.
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::scoped_lock lock(mutex_);
    if (node_->mission_fault()) {
      return BT::NodeStatus::FAILURE;
    }
    if (!goal_pending_ && !send_in_flight_ && !result_ready_) {
      if (halted_) {
        return BT::NodeStatus::IDLE;
      }
      if (send_current_goal_locked()) {
        return BT::NodeStatus::RUNNING;
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!result_ready_) {
      return BT::NodeStatus::RUNNING;
    }
    result_ready_ = false;
    goal_pending_ = false;
    if (!result_success_) {
      node_->set_mission_fault("row navigation action failed");
      return BT::NodeStatus::FAILURE;
    }

    if (phase_ == Phase::Follow) {
      node_->follow_completed();
    } else {
      node_->shift_completed();
    }
    if (node_->route_complete()) {
      return BT::NodeStatus::SUCCESS;
    }
    phase_ = node_->next_action_is_shift() ? Phase::Shift : Phase::Follow;
    if (!send_current_goal_locked()) {
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::scoped_lock lock(mutex_);
    halted_ = true;
    cancel_requested_ = true;
    if (follow_goal_handle_) {
      follow_client_->async_cancel_goal(follow_goal_handle_);
    }
    if (shift_goal_handle_) {
      shift_client_->async_cancel_goal(shift_goal_handle_);
    }
    goal_pending_ = false;
    send_in_flight_ = false;
    result_ready_ = false;
    follow_goal_handle_.reset();
    shift_goal_handle_.reset();
  }

  bool send_current_goal_locked()
  {
    if (goal_pending_ || halted_) {
      return false;
    }
    const auto route = node_->route_parameters();
    if (phase_ == Phase::Follow) {
      if (!follow_client_->action_server_is_ready()) {
        return false;
      }
      FollowRow::Goal goal;
      goal.row_id = route.row_index;
      goal.direction = route.direction;
      goal.max_speed_mps = route.row_speed_mps;
      goal.side_target_m = route.side_target_m;
      goal.front_slow_distance_m = route.front_slow_distance_m;
      goal.front_stop_distance_m = route.front_stop_distance_m;
      goal.use_side_centering = true;
      auto options = FollowClient::SendGoalOptions();
      options.goal_response_callback = [this](FollowGoalHandle::SharedPtr handle) {
        std::scoped_lock lock(mutex_);
        if (halted_) {
          if (handle) {
            follow_client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          goal_pending_ = false;
          send_in_flight_ = false;
          // The server may reject while the previous route goal is still
          // processing its cancellation. Retry on the next tree tick.
          result_ready_ = false;
          return;
        }
        follow_goal_handle_ = handle;
        goal_pending_ = true;
        send_in_flight_ = false;
        if (cancel_requested_) {
          follow_client_->async_cancel_goal(handle);
        }
      };
      options.result_callback = [this](const FollowClient::WrappedResult &result) {
        std::scoped_lock lock(mutex_);
        if (halted_) {
          return;
        }
        goal_pending_ = false;
        send_in_flight_ = false;
        result_ready_ = true;
        result_success_ = succeeded(result.code);
        follow_goal_handle_.reset();
      };
      send_in_flight_ = true;
      follow_client_->async_send_goal(goal, options);
      return true;
    }

    if (!shift_client_->action_server_is_ready()) {
      return false;
    }
    ShiftLane::Goal goal;
    goal.direction = route.shift_direction;
    goal.distance_m = route.shift_distance_m;
    goal.max_speed_mps = route.shift_speed_mps;
    goal.safety_distance_m = route.shift_safety_distance_m;
    auto options = ShiftClient::SendGoalOptions();
    options.goal_response_callback = [this](ShiftGoalHandle::SharedPtr handle) {
      std::scoped_lock lock(mutex_);
      if (halted_) {
        if (handle) {
          shift_client_->async_cancel_goal(handle);
        }
        return;
      }
      if (!handle) {
        goal_pending_ = false;
        send_in_flight_ = false;
        result_ready_ = false;
        return;
      }
      shift_goal_handle_ = handle;
      goal_pending_ = true;
      send_in_flight_ = false;
      if (cancel_requested_) {
        shift_client_->async_cancel_goal(handle);
      }
    };
    options.result_callback = [this](const ShiftClient::WrappedResult &result) {
      std::scoped_lock lock(mutex_);
      if (halted_) {
        return;
      }
      goal_pending_ = false;
      send_in_flight_ = false;
      result_ready_ = true;
      result_success_ = succeeded(result.code);
      shift_goal_handle_.reset();
    };
    send_in_flight_ = true;
    shift_client_->async_send_goal(goal, options);
    return true;
  }

  MissionBehaviorNode *node_;
  FollowClient::SharedPtr follow_client_;
  ShiftClient::SharedPtr shift_client_;
  FollowGoalHandle::SharedPtr follow_goal_handle_;
  ShiftGoalHandle::SharedPtr shift_goal_handle_;
  Phase phase_{Phase::Follow};
  std::mutex mutex_;
  bool halted_{false};
  bool goal_pending_{false};
  bool send_in_flight_{false};
  bool result_ready_{false};
  bool result_success_{false};
  bool cancel_requested_{false};
};

class StopBaseAction final : public BT::StatefulActionNode
{
public:
  StopBaseAction(const std::string &name, const BT::NodeConfiguration &config)
  : BT::StatefulActionNode(name, config),
    client_(rclcpp_action::create_client<StopBase>(mission_node(config), "stop_base")) {}

  static BT::PortsList providedPorts() { return {}; }

private:
  BT::NodeStatus onStart() override
  {
    std::scoped_lock lock(mutex_);
    halted_ = false;
    result_ready_ = false;
    if (!client_->action_server_is_ready()) {
      return BT::NodeStatus::RUNNING;
    }
    StopBase::Goal goal;
    goal.timeout_s = 3.0;
    auto options = StopClient::SendGoalOptions();
    options.goal_response_callback = [this](StopGoalHandle::SharedPtr handle) {
      std::scoped_lock lock(mutex_);
      if (halted_) {
        if (handle) {
          client_->async_cancel_goal(handle);
        }
        return;
      }
      goal_handle_ = handle;
      if (!handle) {
        goal_sent_ = false;
        result_ready_ = false;
      }
    };
    options.result_callback = [this](const StopClient::WrappedResult &result) {
      std::scoped_lock lock(mutex_);
      if (halted_) {
        return;
      }
      result_ready_ = true;
      result_success_ = succeeded(result.code);
      goal_handle_.reset();
    };
    client_->async_send_goal(goal, options);
    goal_sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    {
      std::scoped_lock lock(mutex_);
      if (goal_sent_) {
        if (!result_ready_) {
          return BT::NodeStatus::RUNNING;
        }
        if (!result_success_) {
          mission_node(config())->set_mission_fault("base did not confirm stop");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    return onStart();
  }

  void onHalted() override
  {
    std::scoped_lock lock(mutex_);
    halted_ = true;
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    goal_sent_ = false;
    result_ready_ = false;
  }

  StopClient::SharedPtr client_;
  StopGoalHandle::SharedPtr goal_handle_;
  std::mutex mutex_;
  bool halted_{false};
  bool goal_sent_{false};
  bool result_ready_{false};
  bool result_success_{false};
};

class PickMangoAction final : public BT::StatefulActionNode
{
public:
  PickMangoAction(const std::string &name, const BT::NodeConfiguration &config)
  : BT::StatefulActionNode(name, config),
    node_(mission_node(config)),
    client_(rclcpp_action::create_client<PickPoint>(node_, "pick_point")) {}

  static BT::PortsList providedPorts() { return {}; }

private:
  BT::NodeStatus onStart() override
  {
    std::scoped_lock lock(mutex_);
    halted_ = false;
    result_ready_ = false;
    if (!client_->action_server_is_ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto target = node_->latched_mango();
    PickPoint::Goal goal;
    goal.target.header = target.header;
    if (goal.target.header.frame_id.empty()) {
      goal.target.header.frame_id = "base_link";
    }
    goal.target.point = target.center;
    goal.approach_height_m = node_->get_parameter("mango_approach_height_m").as_double();
    goal.grasp_offset_x_m = node_->get_parameter("mango_grasp_offset_x_m").as_double();
    goal.grasp_offset_y_m = node_->get_parameter("mango_grasp_offset_y_m").as_double();
    goal.grasp_offset_z_m = node_->get_parameter("mango_grasp_offset_z_m").as_double();
    goal.target_tool_tilt = node_->get_parameter("mango_tool_tilt_rad").as_double();
    goal.target_joint5 = node_->get_parameter("mango_joint5_rad").as_double();
    const auto duration = node_->get_parameter("pick_duration_s").as_double();
    goal.duration.sec = static_cast<int32_t>(std::floor(std::max(0.0, duration)));
    goal.duration.nanosec = static_cast<uint32_t>(
      std::llround((std::max(0.0, duration) - goal.duration.sec) * 1.0e9));
    goal.return_home = node_->get_parameter("pick_return_home").as_bool();
    auto options = PickClient::SendGoalOptions();
    options.goal_response_callback = [this](PickGoalHandle::SharedPtr handle) {
      std::scoped_lock lock(mutex_);
      if (halted_) {
        if (handle) {
          client_->async_cancel_goal(handle);
        }
        return;
      }
      goal_handle_ = handle;
      if (!handle) {
        goal_sent_ = false;
        result_ready_ = false;
      }
    };
    options.result_callback = [this](const PickClient::WrappedResult &result) {
      std::scoped_lock lock(mutex_);
      if (halted_) {
        return;
      }
      result_ready_ = true;
      result_success_ = succeeded(result.code) && result.result && result.result->success;
      goal_handle_.reset();
    };
    client_->async_send_goal(goal, options);
    goal_sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    {
      std::scoped_lock lock(mutex_);
      if (goal_sent_) {
        if (!result_ready_) {
          return BT::NodeStatus::RUNNING;
        }
        if (!result_success_) {
          node_->set_mission_fault("PickPoint action failed");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
      }
    }
    return onStart();
  }

  void onHalted() override
  {
    std::scoped_lock lock(mutex_);
    halted_ = true;
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    goal_sent_ = false;
    result_ready_ = false;
  }

  MissionBehaviorNode *node_;
  PickClient::SharedPtr client_;
  PickGoalHandle::SharedPtr goal_handle_;
  std::mutex mutex_;
  bool halted_{false};
  bool goal_sent_{false};
  bool result_ready_{false};
  bool result_success_{false};
};

class MarkMangoHandled final : public BT::SyncActionNode
{
public:
  MarkMangoHandled(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), node_(mission_node(config)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    node_->mark_mango_handled();
    // Return FAILURE so ReactiveFallback immediately resumes RouteStep.
    return BT::NodeStatus::FAILURE;
  }

private:
  MissionBehaviorNode *node_;
};

}  // namespace

MissionBehaviorNode::MissionBehaviorNode() : Node("mission_behavior_node")
{
  declare_parameter("row_count", 7);
  declare_parameter("row_speed_mps", 0.20);
  declare_parameter("shift_distance_m", 0.30);
  declare_parameter("shift_speed_mps", 0.10);
  declare_parameter("shift_direction", 1);
  declare_parameter("row_side_target_m", 0.05);
  declare_parameter("front_slow_distance_m", 0.80);
  declare_parameter("front_stop_distance_m", 0.35);
  declare_parameter("shift_safety_distance_m", 0.12);
  declare_parameter("mango_min_confidence", 0.75);
  declare_parameter("mango_stable_frames", 3);
  declare_parameter("mango_timeout_s", 0.5);
  declare_parameter("mango_rearm_delay_s", 1.0);
  declare_parameter("mango_approach_height_m", 0.10);
  declare_parameter("mango_grasp_offset_x_m", 0.0);
  declare_parameter("mango_grasp_offset_y_m", 0.0);
  declare_parameter("mango_grasp_offset_z_m", 0.0);
  declare_parameter("mango_tool_tilt_rad", 1.5);
  declare_parameter("mango_joint5_rad", 0.0);
  declare_parameter("pick_duration_s", 2.0);
  declare_parameter("pick_return_home", true);
  declare_parameter("tree_file", "");
  declare_parameter("tick_hz", 20.0);

  route_.row_count = get_parameter("row_count").as_int();
  route_.row_speed_mps = get_parameter("row_speed_mps").as_double();
  route_.shift_distance_m = get_parameter("shift_distance_m").as_double();
  route_.shift_speed_mps = get_parameter("shift_speed_mps").as_double();
  route_.shift_direction = static_cast<int8_t>(get_parameter("shift_direction").as_int());
  route_.side_target_m = get_parameter("row_side_target_m").as_double();
  route_.front_slow_distance_m = get_parameter("front_slow_distance_m").as_double();
  route_.front_stop_distance_m = get_parameter("front_stop_distance_m").as_double();
  route_.shift_safety_distance_m = get_parameter("shift_safety_distance_m").as_double();
  mango_min_confidence_ = get_parameter("mango_min_confidence").as_double();
  mango_stable_frames_ = get_parameter("mango_stable_frames").as_int();
  mango_timeout_s_ = get_parameter("mango_timeout_s").as_double();
  mango_rearm_delay_s_ = get_parameter("mango_rearm_delay_s").as_double();

  if (route_.row_count <= 0 || route_.direction != 1 || route_.shift_direction == 0 ||
      route_.row_speed_mps <= 0.0 || route_.shift_distance_m <= 0.0 ||
      route_.shift_speed_mps <= 0.0 || route_.front_slow_distance_m <= route_.front_stop_distance_m ||
      mango_stable_frames_ <= 0) {
    throw std::invalid_argument("invalid mission behavior parameters");
  }

  mango_sub_ = create_subscription<robot_arm_interfaces::msg::MangoTarget>(
    "/vision/mango_target", rclcpp::SensorDataQoS(),
    [this](const robot_arm_interfaces::msg::MangoTarget::SharedPtr message) {
      mango_callback(message);
    });

  tree_xml_ = default_tree_xml();
  const auto tree_file = get_parameter("tree_file").as_string();
  if (!tree_file.empty()) {
    std::ifstream input(tree_file);
    std::stringstream buffer;
    buffer << input.rdbuf();
    if (input.good() || input.eof()) {
      tree_xml_ = buffer.str();
    } else {
      RCLCPP_WARN(get_logger(), "cannot read tree_file '%s', using built-in tree", tree_file.c_str());
    }
  }

  factory_.registerNodeType<MangoReady>("MangoReady");
  factory_.registerNodeType<RouteStep>("RouteStep");
  factory_.registerNodeType<StopBaseAction>("StopBase");
  factory_.registerNodeType<PickMangoAction>("PickMango");
  factory_.registerNodeType<MarkMangoHandled>("MarkMangoHandled");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("mission_node", this);
  tree_ = factory_.createTreeFromText(tree_xml_, blackboard);

  const auto tick_hz = get_parameter("tick_hz").as_double();
  if (tick_hz <= 0.0) {
    throw std::invalid_argument("tick_hz must be positive");
  }
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / tick_hz)),
    [this]() { tick_tree(); });
  RCLCPP_INFO(get_logger(), "BehaviorTree mission ready: seven-row route with mango interruption");
}

void MissionBehaviorNode::mango_callback(
  const robot_arm_interfaces::msg::MangoTarget::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  latest_mango_ = *message;
  latest_mango_time_ = now;
  if (!message->valid || !std::isfinite(message->confidence) ||
      message->confidence < mango_min_confidence_) {
    stable_count_ = 0;
    target_valid_ = false;
    if (!message->valid) {
      waiting_for_target_clear_ = false;
    }
    return;
  }
  if (waiting_for_target_clear_) {
    const bool same_target = last_track_id_ == 0 || message->track_id == 0 ||
      message->track_id == last_track_id_;
    if (same_target) {
      stable_count_ = 0;
      target_valid_ = false;
      return;
    }
    waiting_for_target_clear_ = false;
  }
  if (target_valid_ && last_track_id_ != 0 && message->track_id != 0 &&
      message->track_id != last_track_id_) {
    stable_count_ = 0;
  }
  last_track_id_ = message->track_id;
  stable_count_ = std::min(stable_count_ + 1, mango_stable_frames_);
  target_valid_ = true;
}

bool MissionBehaviorNode::latch_mango_if_ready()
{
  std::scoped_lock lock(mutex_);
  if (fault_) {
    return false;
  }
  if (mango_handling_) {
    return true;
  }
  if (!target_valid_ || stable_count_ < mango_stable_frames_) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - latest_mango_time_).count() > mango_timeout_s_) {
    return false;
  }
  if (last_pick_time_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(now - last_pick_time_).count() < mango_rearm_delay_s_) {
    return false;
  }
  latched_mango_ = latest_mango_;
  mango_handling_ = true;
  RCLCPP_INFO(
    get_logger(), "mango target latched: x=%.3f y=%.3f z=%.3f confidence=%.2f",
    latched_mango_.center.x, latched_mango_.center.y, latched_mango_.center.z,
    static_cast<double>(latched_mango_.confidence));
  return true;
}

robot_arm_interfaces::msg::MangoTarget MissionBehaviorNode::latched_mango() const
{
  std::scoped_lock lock(mutex_);
  return latched_mango_;
}

void MissionBehaviorNode::mark_mango_handled()
{
  std::scoped_lock lock(mutex_);
  last_pick_time_ = std::chrono::steady_clock::now();
  last_track_id_ = latched_mango_.track_id;
  waiting_for_target_clear_ = true;
  target_valid_ = false;
  stable_count_ = 0;
  mango_handling_ = false;
  RCLCPP_INFO(get_logger(), "mango handled; waiting for a new target before rearming");
}

bool MissionBehaviorNode::mango_handling() const
{
  std::scoped_lock lock(mutex_);
  return mango_handling_;
}

bool MissionBehaviorNode::mission_fault() const
{
  std::scoped_lock lock(mutex_);
  return fault_;
}

void MissionBehaviorNode::set_mission_fault(const std::string &message)
{
  std::scoped_lock lock(mutex_);
  if (!fault_) {
    fault_ = true;
    fault_message_ = message;
    RCLCPP_ERROR(get_logger(), "mission fault: %s", message.c_str());
  }
}

MissionBehaviorNode::RouteParameters MissionBehaviorNode::route_parameters() const
{
  std::scoped_lock lock(mutex_);
  return route_;
}

bool MissionBehaviorNode::route_complete() const
{
  std::scoped_lock lock(mutex_);
  return route_complete_;
}

bool MissionBehaviorNode::next_action_is_shift() const
{
  std::scoped_lock lock(mutex_);
  return next_action_shift_;
}

void MissionBehaviorNode::follow_completed()
{
  std::scoped_lock lock(mutex_);
  if (route_.row_index >= route_.row_count - 1) {
    route_complete_ = true;
    RCLCPP_INFO(get_logger(), "all rows completed");
    return;
  }
  next_action_shift_ = true;
}

void MissionBehaviorNode::shift_completed()
{
  std::scoped_lock lock(mutex_);
  ++route_.row_index;
  route_.direction = static_cast<int8_t>(-route_.direction);
  route_.shift_direction = static_cast<int8_t>(-route_.shift_direction);
  next_action_shift_ = false;
}

const std::string &MissionBehaviorNode::tree_xml() const
{
  return tree_xml_;
}

void MissionBehaviorNode::tick_tree()
{
  if (mission_fault()) {
    if (tick_timer_) {
      tick_timer_->cancel();
    }
    return;
  }
  const auto status = tree_.tickRoot();
  if (status == BT::NodeStatus::SUCCESS && route_complete()) {
    RCLCPP_INFO(get_logger(), "mission completed; behavior tree stopped");
    if (tick_timer_) {
      tick_timer_->cancel();
    }
  } else if (status == BT::NodeStatus::FAILURE && mission_fault()) {
    if (tick_timer_) {
      tick_timer_->cancel();
    }
  }
}

}  // namespace robot_chassis

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_chassis::MissionBehaviorNode>());
  rclcpp::shutdown();
  return 0;
}
