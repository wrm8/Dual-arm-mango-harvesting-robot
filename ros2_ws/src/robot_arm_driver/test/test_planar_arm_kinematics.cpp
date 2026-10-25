#include "robot_arm_driver/planar_arm_kinematics.hpp"

#include <gtest/gtest.h>

TEST(PlanarArmKinematics, InverseAndForwardAgree)
{
  robot_arm_driver::PlanarArmKinematics solver({0.10, 0.02, 0.15, 0.12, 0.08});
  const std::array<double, 4> expected{0.20, 0.45, -0.70, -0.25};
  const auto point = solver.forward_position(expected);
  const auto solved = solver.inverse(point[0], point[1], point[2], -0.50, true);
  const auto recomputed = solver.forward_position(solved);

  EXPECT_NEAR(recomputed[0], point[0], 1e-9);
  EXPECT_NEAR(recomputed[1], point[1], 1e-9);
  EXPECT_NEAR(recomputed[2], point[2], 1e-9);
  EXPECT_NEAR(solved[0], expected[0], 1e-9);
}
