#include "robot_arm_node/motion/gravity_model.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_arm_node::motion
{
namespace
{
void validate_gravity_model(const GravityModel &model)
{
  const double values[] = {
    model.upper_arm_mass_kg,
    model.upper_arm_com_m,
    model.forearm_mass_kg,
    model.forearm_com_m,
    model.tool_mass_kg,
    model.tool_com_m,
    model.payload_mass_kg,
    model.payload_distance_m,
    model.gravity_mps2};

  for (const double value : values)
  {
    if (!std::isfinite(value) || value < 0.0)
    {
      throw std::invalid_argument("gravity model values must be finite and non-negative");
    }
  }
}
}   // namespace

JointVector gravity_compensation(const ArmGeometry &g, const GravityModel &m, const JointVector &q)
{
  validate_gravity_model(m);
  if (m.gravity_mps2 == 0.0)
  {
    return {};
  }

  const double shoulder_angle = q[1];
  const double elbow_angle = q[1] + q[2];
  const double wrist_angle = elbow_angle + q[3];
  const double distal_mass = m.tool_mass_kg + m.payload_mass_kg;
  const double distal_first_moment =
    m.tool_mass_kg * m.tool_com_m + m.payload_mass_kg * m.payload_distance_m;

  JointVector torque{};

  torque[1] = m.gravity_mps2 * (
    m.upper_arm_mass_kg * m.upper_arm_com_m * std::cos(shoulder_angle) +
    (m.forearm_mass_kg + distal_mass) * g.upper_arm_length_m * std::cos(shoulder_angle) +
    m.forearm_mass_kg * m.forearm_com_m * std::cos(elbow_angle) +
    distal_first_moment * std::cos(wrist_angle));
  torque[2] = m.gravity_mps2 * (
    m.forearm_mass_kg * m.forearm_com_m * std::cos(elbow_angle) +
    distal_first_moment * std::cos(wrist_angle));
  torque[3] = m.gravity_mps2 * distal_first_moment * std::cos(wrist_angle);

  return torque;
}
}  // namespace robot_arm_node::motion
