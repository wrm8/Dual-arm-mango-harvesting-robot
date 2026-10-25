#pragma once

#include <array>

namespace robot_arm_driver
{
// Kinematic convention: joint_1 yaws about +Z. Joints 2-4 form a planar arm
// in the radial-Z plane. At zero pitch the tool points in positive radial X.
struct PlanarArmGeometry
{
  double base_height_m{};
  double shoulder_offset_m{};
  double upper_arm_length_m{};
  double forearm_length_m{};
  double tool_length_m{};
};

class PlanarArmKinematics
{
  public:
    explicit PlanarArmKinematics(PlanarArmGeometry geometry);
    std::array<double, 4> inverse(double x_m, double y_m, double z_m,
                                  double tool_pitch_rad, bool elbow_up = false) const;
    std::array<double, 3> forward_position(const std::array<double, 4> &joints) const;

  private:
    PlanarArmGeometry geometry_;
};
}  // namespace robot_arm_driver
