#include "robot_arm_node/motion/jacobian.hpp"
#include "robot_arm_node/motion/joint_limits.hpp"
#include "robot_arm_node/motion/kinematics.hpp"
#include "robot_arm_node/motion/planning.hpp"
#include "robot_arm_node/motion/trajectory.hpp"

#include <gtest/gtest.h>

namespace motion = robot_arm_node::motion;

TEST(Motion, KinematicsRoundTrip)
{
  motion::Kinematics solver({0.10, 0.02, 0.15, 0.12, 0.08});
  const motion::JointVector input{0.20, 0.45, -0.70, -0.25};
  const auto solved = solver.inverse(solver.forward(input));
  const auto result = solver.forward(solved);

  EXPECT_NEAR(result.position.x, solver.forward(input).position.x, 1e-9);
  EXPECT_NEAR(result.position.y, solver.forward(input).position.y, 1e-9);
  EXPECT_NEAR(result.position.z, solver.forward(input).position.z, 1e-9);
}

TEST(Motion, PlanningAndTrajectoryStayBounded)
{
  motion::JointLimits limits({{{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}}});
  const auto path = motion::plan_joint_path({0.0, 0.0, 0.0, 0.0}, {0.4, -0.4, 0.2, 0.1}, limits, 0.1);

  ASSERT_GT(path.waypoints.size(), 2U);
  const auto trajectory = motion::time_parameterize(path, {{1.0, 1.0, 1.0, 1.0}, {2.0, 2.0, 2.0, 2.0}});

  ASSERT_FALSE(trajectory.points.empty());
  EXPECT_NEAR(trajectory.points.back().positions[0], 0.4, 1e-9);
}
