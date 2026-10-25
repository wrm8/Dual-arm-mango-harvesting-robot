#pragma once

#include "robot_arm_node/motion/kinematics.hpp"

#include <array>

namespace robot_arm_node::motion
{
using PositionJacobian = std::array<std::array<double, kArmJointCount>, 3>;

// Translational Jacobian: Cartesian tool linear velocity = J * joint velocity.
PositionJacobian position_jacobian(const ArmGeometry &geometry, const JointVector &joints);
double manipulability(const PositionJacobian &jacobian);
}  // namespace robot_arm_node::motion
