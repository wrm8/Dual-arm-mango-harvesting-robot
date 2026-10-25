#pragma once

#include "robot_arm_node/motion/planning.hpp"

#include <vector>

namespace robot_arm_node::motion
{
struct MotionConstraints
{
  JointVector max_velocity_radps{};
  JointVector max_acceleration_radps2{};
};

struct TrajectoryPoint
{
  double time_from_start_s{};
  JointVector positions{};
  JointVector velocities{};
  JointVector accelerations{};
};

struct JointTrajectory
{
  std::vector<TrajectoryPoint> points;
};

// Produces synchronized, zero-end-velocity quintic segments for a joint path.
JointTrajectory time_parameterize(const JointPath &path, const MotionConstraints &constraints,
                                  double sample_period_s = 0.02);
}  // namespace robot_arm_node::motion
