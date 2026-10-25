#include "robot_arm_node/motion/jacobian.hpp"

#include <cmath>

namespace robot_arm_node::motion
{
PositionJacobian position_jacobian(const ArmGeometry &g, const JointVector &q)
{
  const double shoulder_angle = q[1];
  const double elbow_angle = q[1] + q[2];
  const double wrist_angle = elbow_angle + q[3];

  const double radial = g.shoulder_offset_m + g.upper_arm_length_m * std::cos(shoulder_angle) +
                        g.forearm_length_m * std::cos(elbow_angle) + g.tool_length_m * std::cos(wrist_angle);
  const std::array<double, 3> radial_derivatives{
    -g.upper_arm_length_m * std::sin(shoulder_angle) -
    g.forearm_length_m * std::sin(elbow_angle) -
    g.tool_length_m * std::sin(wrist_angle),
    -g.forearm_length_m * std::sin(elbow_angle) - g.tool_length_m * std::sin(wrist_angle),
    -g.tool_length_m * std::sin(wrist_angle)};
  const std::array<double, 3> height_derivatives{
    g.upper_arm_length_m * std::cos(shoulder_angle) +
    g.forearm_length_m * std::cos(elbow_angle) +
    g.tool_length_m * std::cos(wrist_angle),
    g.forearm_length_m * std::cos(elbow_angle) + g.tool_length_m * std::cos(wrist_angle),
    g.tool_length_m * std::cos(wrist_angle)};

  PositionJacobian j{};

  j[0][0] = -radial *std::sin(q[0]);
  j[1][0] = radial * std::cos(q[0]);

  for (std::size_t i = 0; i < radial_derivatives.size(); ++i)
  {
    j[0][i + 1] = std::cos(q[0]) * radial_derivatives[i];
    j[1][i + 1] = std::sin(q[0]) * radial_derivatives[i];
    j[2][i + 1] = height_derivatives[i];
  }

  return j;
}

double manipulability(const PositionJacobian &j)
{
  double a[3][3]{};

  for (std::size_t row = 0; row < 3; ++row)
  {
    for (std::size_t col = 0; col < 3; ++col)
    {
      for (std::size_t joint = 0; joint < kArmJointCount; ++joint)
      {
        a[row][col] += j[row][joint] * j[col][joint];
      }
    }
  }

  const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                     a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

  return std::sqrt(std::max(0.0, det));
}
}  // namespace robot_arm_node::motion
