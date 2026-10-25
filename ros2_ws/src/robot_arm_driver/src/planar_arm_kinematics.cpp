#include "robot_arm_driver/planar_arm_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_arm_driver
{
PlanarArmKinematics::PlanarArmKinematics(PlanarArmGeometry geometry) : geometry_(geometry)
{
  if (geometry_.upper_arm_length_m <= 0.0 || geometry_.forearm_length_m <= 0.0 ||
      geometry_.tool_length_m < 0.0)
  {
    throw std::invalid_argument("IK link lengths must be positive (tool length may be zero)");
  }
}

std::array<double, 4> PlanarArmKinematics::inverse(
  const double x_m, const double y_m, const double z_m, const double tool_pitch_rad,
  const bool elbow_up) const
{
  const double yaw = std::atan2(y_m, x_m);
  const double radial = std::hypot(x_m, y_m) - geometry_.shoulder_offset_m;
  const double wrist_r = radial - geometry_.tool_length_m * std::cos(tool_pitch_rad);
  const double wrist_z = z_m - geometry_.base_height_m - geometry_.tool_length_m * std::sin(tool_pitch_rad);
  const double l1 = geometry_.upper_arm_length_m;
  const double l2 = geometry_.forearm_length_m;
  const double cosine_elbow = (wrist_r * wrist_r + wrist_z * wrist_z - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);

  if (!std::isfinite(cosine_elbow) || cosine_elbow < -1.0 - 1e-9 || cosine_elbow > 1.0 + 1e-9)
  {
    throw std::out_of_range("target is outside the configured IK workspace");
  }
  const double elbow = (elbow_up ? -1.0 : 1.0) * std::acos(std::clamp(cosine_elbow, -1.0, 1.0));
  const double shoulder = std::atan2(wrist_z, wrist_r) - std::atan2(l2 * std::sin(elbow), l1 + l2 * std::cos(elbow));
  const double wrist = tool_pitch_rad - shoulder - elbow;

  return {yaw, shoulder, elbow, wrist};
}

std::array<double, 3> PlanarArmKinematics::forward_position(const std::array<double, 4> &joints) const
{
  const double radial = geometry_.shoulder_offset_m +
                        geometry_.upper_arm_length_m * std::cos(joints[1]) +
                        geometry_.forearm_length_m * std::cos(joints[1] + joints[2]) +
                        geometry_.tool_length_m * std::cos(joints[1] + joints[2] + joints[3]);
  const double z = geometry_.base_height_m +
                   geometry_.upper_arm_length_m * std::sin(joints[1]) +
                   geometry_.forearm_length_m * std::sin(joints[1] + joints[2]) +
                   geometry_.tool_length_m * std::sin(joints[1] + joints[2] + joints[3]);

  return {radial *std::cos(joints[0]), radial *std::sin(joints[0]), z};
}
}  // namespace robot_arm_driver
