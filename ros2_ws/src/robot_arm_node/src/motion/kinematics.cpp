#include "robot_arm_node/motion/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_arm_node::motion
{
Kinematics::Kinematics(ArmGeometry geometry) : geometry_(geometry)
{
  if (geometry_.upper_arm_length_m <= 0.0 || geometry_.forearm_length_m <= 0.0 ||
      geometry_.tool_length_m < 0.0)
  {
    throw std::invalid_argument("arm link lengths must be positive (tool length may be zero)");
  }
}

EndEffectorPose Kinematics::forward(const JointVector &q) const
{
  const double pitch = q[1] + q[2] + q[3];
  const double radial = geometry_.shoulder_offset_m +
                        geometry_.upper_arm_length_m * std::cos(q[1]) +
                        geometry_.forearm_length_m * std::cos(q[1] + q[2]) +
                        geometry_.tool_length_m * std::cos(pitch);
  const double x = radial * std::cos(q[0]);
  const double y = radial * std::sin(q[0]);
  const double z = geometry_.base_height_m +
                   geometry_.upper_arm_length_m * std::sin(q[1]) +
                   geometry_.forearm_length_m * std::sin(q[1] + q[2]) +
                   geometry_.tool_length_m * std::sin(pitch);

  return {{x, y, z}, pitch};
}

JointVector Kinematics::inverse(const EndEffectorPose &target, const bool elbow_up) const
{
  const double yaw = std::atan2(target.position.y, target.position.x);
  const double radial = std::hypot(target.position.x, target.position.y) - geometry_.shoulder_offset_m;
  const double wrist_r = radial - geometry_.tool_length_m * std::cos(target.pitch_rad);
  const double wrist_z = target.position.z - geometry_.base_height_m -
                         geometry_.tool_length_m * std::sin(target.pitch_rad);
  const double l1 = geometry_.upper_arm_length_m;
  const double l2 = geometry_.forearm_length_m;
  const double cosine_elbow = (wrist_r * wrist_r + wrist_z * wrist_z - l1 * l1 - l2 * l2) /
                              (2.0 * l1 * l2);

  if (!std::isfinite(cosine_elbow) || cosine_elbow < -1.0 - 1e-9 || cosine_elbow > 1.0 + 1e-9)
  {
    throw std::out_of_range("target is outside the configured IK workspace");
  }
  const double elbow = (elbow_up ? -1.0 : 1.0) * std::acos(std::clamp(cosine_elbow, -1.0, 1.0));
  const double shoulder = std::atan2(wrist_z, wrist_r) -
                          std::atan2(l2 * std::sin(elbow), l1 + l2 * std::cos(elbow));

  return {yaw, shoulder, elbow, target.pitch_rad - shoulder - elbow};
}

const ArmGeometry &Kinematics::geometry() const noexcept
{
  return geometry_;
}
}  // namespace robot_arm_node::motion
