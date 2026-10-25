#pragma once

#include "robot_arm_node/motion/kinematics.hpp"

#include <array>
#include <string>

namespace robot_arm_node::motion
{
struct JointLimit
{
  double min_rad{};
  double max_rad{};
};

class JointLimits
{
  public:
    explicit JointLimits(std::array<JointLimit, kArmJointCount> limits);
    bool contains(const JointVector &joints, std::string *reason = nullptr) const;
    JointVector clamp(const JointVector &joints) const;
    const std::array<JointLimit, kArmJointCount> &values() const noexcept;

  private:
    std::array<JointLimit, kArmJointCount> limits_;
};
}  // namespace robot_arm_node::motion
