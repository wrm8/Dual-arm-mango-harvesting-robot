#include "robot_arm_node/motion/planning.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_arm_node::motion
{
JointPath plan_joint_path(const JointVector &start, const JointVector &goal,
                          const JointLimits &limits, const double max_step_rad)
{
  if (!std::isfinite(max_step_rad) || max_step_rad <= 0.0)
  {
    throw std::invalid_argument("max_step_rad must be positive and finite");
  }

  std::string reason;

  if (!limits.contains(start, &reason))
  {
    throw std::out_of_range("invalid path start: " + reason);
  }
  if (!limits.contains(goal, &reason))
  {
    throw std::out_of_range("invalid path goal: " + reason);
  }

  double largest_delta = 0.0;

  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    largest_delta = std::max(largest_delta, std::abs(goal[i] - start[i]));
  }

  const std::size_t segments = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(largest_delta / max_step_rad)));
  JointPath path;

  path.waypoints.reserve(segments + 1);

  for (std::size_t step = 0; step <= segments; ++step)
  {
    const double t = static_cast<double>(step) / static_cast<double>(segments);
    JointVector waypoint{};
    for (std::size_t i = 0; i < kArmJointCount; ++i)
    {
      waypoint[i] = start[i] + (goal[i] - start[i]) * t;
    }
    if (!limits.contains(waypoint, &reason))
    {
      throw std::out_of_range("planned waypoint is invalid: " + reason);
    }
    path.waypoints.push_back(waypoint);
  }
  return path;
}
}  // namespace robot_arm_node::motion
