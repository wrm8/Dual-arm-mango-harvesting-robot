#pragma once

#include "robot_arm_node/motion/joint_limits.hpp"

#include <vector>

namespace robot_arm_node::motion
{
struct JointPath
{
  std::vector<JointVector> waypoints;
};

// Plans a straight, collision-unaware joint-space path. max_step_rad must be positive.
// It validates every waypoint against limits and throws on an invalid start/goal.
JointPath plan_joint_path(const JointVector &start, const JointVector &goal,
                          const JointLimits &limits, double max_step_rad);
}  // namespace robot_arm_node::motion
