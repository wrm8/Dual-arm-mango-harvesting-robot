#include "robot_arm_node/motion/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_arm_node::motion
{
namespace
{
double segment_duration(const JointVector &start, const JointVector &goal, const MotionConstraints &c)
{
  double duration = 0.0;

  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    const double max_velocity = c.max_velocity_radps[i];
    const double max_acceleration = c.max_acceleration_radps2[i];
    if (!std::isfinite(max_velocity) || !std::isfinite(max_acceleration) ||
        max_velocity <= 0.0 || max_acceleration <= 0.0)
    {
      throw std::invalid_argument(
              "velocity and acceleration constraints must be positive and finite");
    }

    const double distance = std::abs(goal[i] - start[i]);
    const double velocity_duration = 1.875 * distance / max_velocity;
    const double acceleration_duration = std::sqrt(5.7735026919 * distance / max_acceleration);
    duration = std::max(duration, std::max(velocity_duration, acceleration_duration));
  }
  return duration;
}

TrajectoryPoint quintic_point(const JointVector &start, const JointVector &goal, const double local_t, const double duration, const double global_t)
{
  const double u = std::clamp(local_t / duration, 0.0, 1.0);
  const double position_scale = 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
  const double velocity_scale =
    (30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u) / duration;
  const double acceleration_scale =
    (60.0 * u - 180.0 * u * u + 120.0 * u * u * u) / (duration * duration);

  TrajectoryPoint point;

  point.time_from_start_s = global_t;
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    const double delta = goal[i] - start[i];
    point.positions[i] = start[i] + delta * position_scale;
    point.velocities[i] = delta * velocity_scale;
    point.accelerations[i] = delta * acceleration_scale;
  }

  return point;
}
}   // namespace

JointTrajectory time_parameterize(const JointPath &path, const MotionConstraints &constraints, const double sample_period_s)
{
  if (path.waypoints.size() < 2)
  {
    throw std::invalid_argument("a path needs at least two waypoints");
  }
  if (!std::isfinite(sample_period_s) || sample_period_s <= 0.0)
  {
    throw std::invalid_argument("sample_period_s must be positive and finite");
  }

  JointTrajectory trajectory;
  double global_time = 0.0;

  for (std::size_t segment = 0; segment + 1 < path.waypoints.size(); ++segment)
  {
    const auto &start = path.waypoints[segment];
    const auto &goal = path.waypoints[segment + 1];
    const double duration = std::max(sample_period_s, segment_duration(start, goal, constraints));
    const std::size_t samples = static_cast<std::size_t>(std::ceil(duration / sample_period_s));

    for (std::size_t sample = segment == 0 ? 0 : 1; sample <= samples; ++sample)
    {
      const double local_t = std::min(
        duration, duration * static_cast<double>(sample) / static_cast<double>(samples));
      trajectory.points.push_back(
        quintic_point(start, goal, local_t, duration, global_time + local_t));
    }
    global_time += duration;
  }
  return trajectory;
}
}  // namespace robot_arm_node::motion
