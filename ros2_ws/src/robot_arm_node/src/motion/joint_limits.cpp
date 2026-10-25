#include "robot_arm_node/motion/joint_limits.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_arm_node::motion
{
JointLimits::JointLimits(std::array<JointLimit, kArmJointCount> limits) : limits_(limits)
{
  for (const auto &limit : limits_)
  {
    if (!std::isfinite(limit.min_rad) || !std::isfinite(limit.max_rad) || limit.min_rad > limit.max_rad)
    {
      throw std::invalid_argument("each joint limit must be finite and min_rad <= max_rad");
    }
  }
}

bool JointLimits::contains(const JointVector &joints, std::string *reason) const
{
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    if (!std::isfinite(joints[i]) || joints[i] < limits_[i].min_rad || joints[i] > limits_[i].max_rad)
    {
      if (reason != nullptr)
      {
        *reason = "joint " + std::to_string(i + 1) + " exceeds its configured limit";
      }
      return false;
    }
  }
  return true;
}

JointVector JointLimits::clamp(const JointVector &joints) const
{
  JointVector result{};

  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    result[i] = std::clamp(joints[i], limits_[i].min_rad, limits_[i].max_rad);
  }
  return result;
}

const std::array<JointLimit, kArmJointCount> &JointLimits::values() const noexcept
{
  return limits_;
}
}  // namespace robot_arm_node::motion
