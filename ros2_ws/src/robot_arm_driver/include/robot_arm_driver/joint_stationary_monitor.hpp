#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot_arm_driver
{
// Measure the full range in a continuous feedback window, not just the last
// increment: a slowly drifting joint must not be mistaken for a stopped joint.
class JointStationaryMonitor
{
public:
  using Clock = std::chrono::steady_clock;

  JointStationaryMonitor(int tolerance_ticks, std::chrono::milliseconds duration)
  : tolerance_ticks_(tolerance_ticks), duration_(duration) {}

  bool update(const std::vector<int16_t> &positions, bool all_fresh, Clock::time_point now)
  {
    if (!all_fresh || positions.empty())
    {
      samples_ = 0;
      return false;
    }
    if (samples_ == 0 || positions.size() != minimum_.size())
    {
      restart(positions, now);
      return false;
    }
    for (size_t i = 0; i < positions.size(); ++i)
    {
      minimum_[i] = std::min(minimum_[i], positions[i]);
      maximum_[i] = std::max(maximum_[i], positions[i]);
      if (static_cast<int>(maximum_[i]) - static_cast<int>(minimum_[i]) > tolerance_ticks_)
      {
        restart(positions, now);
        return false;
      }
    }
    ++samples_;
    return samples_ >= 3 && now - since_ >= duration_;
  }

private:
  void restart(const std::vector<int16_t> &positions, Clock::time_point now)
  {
    minimum_ = maximum_ = positions;
    since_ = now;
    samples_ = 1;
  }

  int tolerance_ticks_;
  std::chrono::milliseconds duration_;
  Clock::time_point since_{};
  size_t samples_{0};
  std::vector<int16_t> minimum_;
  std::vector<int16_t> maximum_;
};
}  // namespace robot_arm_driver
