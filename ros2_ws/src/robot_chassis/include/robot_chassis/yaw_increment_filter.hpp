#pragma once

#include <cmath>
#include <stdexcept>

namespace robot_chassis
{
// Drop each small raw increment, rather than comparing raw yaw with the held output.
// This also removes genuine turns whose per-feedback increment is below the threshold.
class YawIncrementFilter
{
public:
  struct Sample
  {
    double yaw;
    double delta;
    bool suppressed;
  };

  void set_threshold(double threshold)
  {
    if (!std::isfinite(threshold) || threshold < 0.0 || threshold > 3.14159265358979323846)
    {
      throw std::invalid_argument("imu_yaw_increment_deadband_rad must be finite and in 0..pi");
    }
    threshold_ = threshold;
  }

  void set_raw_reference(double raw_yaw)
  {
    if (!std::isfinite(raw_yaw))
    {
      throw std::invalid_argument("yaw must be finite");
    }
    previous_raw_ = normalize(raw_yaw);
    initialized_ = true;
  }

  Sample update(double raw_yaw)
  {
    if (!std::isfinite(raw_yaw))
    {
      throw std::invalid_argument("yaw must be finite");
    }
    if (!initialized_)
    {
      initialized_ = true;
      previous_raw_ = raw_yaw;
      filtered_yaw_ = normalize(raw_yaw);
      return {filtered_yaw_, 0.0, false};
    }
    const auto raw_delta = normalize(raw_yaw - previous_raw_);
    previous_raw_ = raw_yaw;
    const bool suppressed = std::abs(raw_delta) < threshold_;
    const auto delta = suppressed ? 0.0 : raw_delta;
    filtered_yaw_ = normalize(filtered_yaw_ + delta);
    return {filtered_yaw_, delta, suppressed};
  }

private:
  static double normalize(double value)
  {
    return std::atan2(std::sin(value), std::cos(value));
  }

  double threshold_{0.001};
  double previous_raw_{0.0};
  double filtered_yaw_{0.0};
  bool initialized_{false};
};
}  // namespace robot_chassis
