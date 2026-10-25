#pragma once

#include "robot_arm_node/motion/kinematics.hpp"

namespace robot_arm_node::motion
{
// Link centres of mass are measured from each link's proximal joint.
struct GravityModel
{
  double upper_arm_mass_kg{};
  double upper_arm_com_m{};
  double forearm_mass_kg{};
  double forearm_com_m{};
  double tool_mass_kg{};
  double tool_com_m{};
  double payload_mass_kg{};
  double payload_distance_m{};
  double gravity_mps2{9.80665};
};

// Positive output is the torque that must be applied to statically hold the arm.
JointVector gravity_compensation(const ArmGeometry &geometry, const GravityModel &model,
                                 const JointVector &joints);
}  // namespace robot_arm_node::motion
