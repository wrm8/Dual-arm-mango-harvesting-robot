#pragma once

#include <array>

namespace robot_arm_node::motion
{
constexpr std::size_t kArmJointCount = 4;
using JointVector = std::array<double, kArmJointCount>;

struct Vec3
{
  double x{};
  double y{};
  double z{};
};

// joint_1 is a +Z yaw. joints_2..4 form a radial-Z planar arm.
struct ArmGeometry
{
  double base_height_m{};
  double shoulder_offset_m{};
  double upper_arm_length_m{};
  double forearm_length_m{};
  double tool_length_m{};
};

struct EndEffectorPose
{
  Vec3 position;
  double pitch_rad{};
};

class Kinematics
{
  public:
    explicit Kinematics(ArmGeometry geometry);

    EndEffectorPose forward(const JointVector &joints) const;
    // Throws std::out_of_range when the position is unreachable.
    JointVector inverse(const EndEffectorPose &target, bool elbow_up = false) const;
    const ArmGeometry &geometry() const noexcept;

  private:
    ArmGeometry geometry_;
};
}  // namespace robot_arm_node::motion
