#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace robot_chassis
{
// No ROS or wall-clock dependency: exercise braking/settling with recorded inputs.
class VisionStopController
{
public:
  enum class State : uint8_t {Idle, Searching, Aligning, Settling, Holding, Fault};
  struct Parameters
  {
    double kp{1.0};
    double ki{0.05};
    double integral_limit{0.2};
    double max_align_speed{0.12};
    double center_tolerance{0.02};  // Fraction of displayed image width, not meters.
    double settle_time{0.3};
    double observation_timeout{0.6};
    double search_distance{0.35};
    double alignment_timeout{15.0};
    double control_sign{1.0};
    double rearm_distance{0.10};
    double rearm_departure_ratio{0.08};
    double rearm_delay{1.0};
    double rearm_speed{0.10};
  };
  struct Input
  {
    double now{0.0};
    double observation_time{0.0};
    uint32_t frame_sequence{0};
    bool have_observation{false};
    bool mango_in_range{false};
    bool tree_valid{false};
    double tree_error{0.0};  // (trunk X - target X) / displayed image width.
    double lateral_position{0.0};
    bool stopped{false};  // Fresh measured x/y/angular velocity AND command are near zero.
  };
  struct Output
  {
    State state{State::Idle};
    double velocity{0.0};
    std::string message{"waiting for mango in depth range"};
  };

  State state() const {return state_;}
  bool active() const {return state_ != State::Idle;}
  bool rearming() const {return rearming_;}
  // Old-tree suppression can continue at route speed after the departure gates.
  bool slow_departure() const {return rearming_ && !departure_complete_;}

  // Called only after a matching successful batch result AND measured stop.
  bool resume_after_harvest(const Input &in)
  {
    if (!hold_latched_ || state_ != State::Holding || !in.stopped) {return false;}
    *this = VisionStopController{};
    rearming_ = true;
    resumed_at_ = in.now;
    resumed_position_ = in.lateral_position;
    return true;
  }

  Output update(const Input &in, const Parameters &p, bool enabled,
    double route_speed, int direction)
  {
    const bool fresh = in.have_observation &&
      in.now >= in.observation_time &&
      in.now - in.observation_time <= p.observation_timeout;
    if (state_ == State::Fault) {return {state_, 0.0, fault_};}
    if (rearming_) {
      if (!fresh) {return fail("camera observation is stale while leaving harvested tree");}
      // Adjacent trees may keep mango_in_range continuously true. Wait until
      // the parked trunk leaves the center region, rather than an empty scene.
      if (!in.tree_valid || (std::isfinite(in.tree_error) &&
          std::abs(in.tree_error) >= p.rearm_departure_ratio)) {
        departed_tree_ = true;
      }
      const double progress = direction * (in.lateral_position - resumed_position_);
      if (progress >= p.rearm_distance && in.now - resumed_at_ >= p.rearm_delay &&
        departed_tree_)
      {
        departure_complete_ = true;
      }
      // The old trunk moves behind the route direction. A new trunk can be
      // acquired on the approaching side, before it crosses the narrow center
      // tolerance between camera frames. Never steer back to the old trunk.
      const bool next_tree = in.tree_valid && std::isfinite(in.tree_error) &&
        (std::abs(in.tree_error) <= p.center_tolerance ||
         direction * p.control_sign * in.tree_error > p.center_tolerance);
      if (!departure_complete_ || !next_tree)
      {
        const double speed = slow_departure() ?
          std::min({route_speed, p.max_align_speed, p.rearm_speed}) : route_speed;
        return {State::Idle, direction * speed, slow_departure() ?
          "harvest complete; low-speed departure, suppressing previous tree" :
          "harvest complete; route speed restored, suppressing previous tree"};
      }
      rearming_ = false;
    }
    if (state_ == State::Idle) {
      if (!enabled || !fresh || !in.mango_in_range) {return {};}
      state_ = State::Searching;
      started_ = in.now;
      search_start_ = in.lateral_position;
      // Switch directly to the configured low speed after mango qualification.
      search_budget_ = p.search_distance;
    }
    // Only an explicit successful harvest result releases parking.
    if (hold_latched_) {
      if (!in.stopped) {
        state_ = State::Settling;
        settling_ = false;
      } else if (!settling_) {
        settling_ = true;
        settled_since_ = in.now;
      } else if (in.now - settled_since_ >= p.settle_time) {
        state_ = State::Holding;
      }
      return {state_, 0.0, "tree centered; holding zero velocity for inspection"};
    }
    if (!fresh) {return fail("camera observation is stale during visual stop");}
    if (in.now - started_ > p.alignment_timeout) {
      // No arm command is possible before HOLDING. Skip an uncenterable tree
      // instead of aborting the whole route; sensor/safety faults still stop.
      *this = VisionStopController{};
      rearming_ = true;
      resumed_at_ = in.now;
      resumed_position_ = in.lateral_position;
      return {State::Idle, 0.0, "tree centering timed out; skipping tree and continuing lane"};
    }
    const bool tree_valid = in.tree_valid && std::isfinite(in.tree_error);
    if (!tree_valid) {
      integral_ = 0.0;
      if (ever_aligned_) {
        state_ = State::Settling;
        settling_ = false;
        return {state_, 0.0, "tree lost; stop until the tree is reacquired"};
      }
      settling_ = false;
      state_ = State::Searching;
      const double remaining = std::max(0.0, search_budget_ -
        std::abs(in.lateral_position - search_start_));
      const double speed = remaining > 0.0 ?
        std::min(route_speed, p.max_align_speed) : 0.0;
      return {state_, direction * speed, "mango detected; low-speed while waiting for tree"};
    }

    ever_aligned_ = true;
    const double error = p.control_sign * in.tree_error;
    const double cap = std::min(route_speed, p.max_align_speed);
    if (std::abs(error) <= p.center_tolerance) {
      integral_ = 0.0;
      state_ = State::Settling;
      if (!in.stopped) {
        settling_ = false;
      } else if (!settling_) {
        settling_ = true;
        settled_since_ = in.now;
      } else if (in.now - settled_since_ >= p.settle_time) {
        state_ = State::Holding;
        hold_latched_ = true;
      }
      return {state_, 0.0, "tree centered; waiting for measured zero velocity"};
    }

    settling_ = false;
    state_ = State::Aligning;
    // Integrate once per new camera frame, not once per chassis control tick.
    if (!have_frame_ || in.frame_sequence != last_frame_) {
      const double dt = have_frame_ ?
        std::clamp(in.observation_time - last_observation_, 0.0, p.observation_timeout) : 0.0;
      if (last_error_ * error < 0.0) {integral_ = 0.0;}
      const double next = std::clamp(integral_ + error * dt,
        -p.integral_limit, p.integral_limit);
      const double unconstrained = p.kp * error + p.ki * next;
      if (std::abs(unconstrained) < cap || error * integral_ < 0.0) {integral_ = next;}
      last_observation_ = in.observation_time;
      last_frame_ = in.frame_sequence;
      have_frame_ = true;
      last_error_ = error;
    }
    const double velocity = std::clamp(p.kp * error + p.ki * integral_, -cap, cap);
    return {state_, velocity, "centering tree in camera image with PI under the low-speed cap"};
  }

private:
  Output fail(const std::string &message)
  {
    state_ = State::Fault;
    fault_ = message;
    return {state_, 0.0, message};
  }
  State state_{State::Idle};
  double started_{0.0};
  double search_start_{0.0};
  double search_budget_{0.0};
  double integral_{0.0};
  double last_error_{0.0};
  double last_observation_{0.0};
  double settled_since_{0.0};
  uint32_t last_frame_{0};
  bool have_frame_{false};
  bool ever_aligned_{false};
  bool settling_{false};
  bool hold_latched_{false};
  bool rearming_{false};
  bool departed_tree_{false};
  bool departure_complete_{false};
  double resumed_at_{0.0};
  double resumed_position_{0.0};
  std::string fault_;
};
}  // namespace robot_chassis
