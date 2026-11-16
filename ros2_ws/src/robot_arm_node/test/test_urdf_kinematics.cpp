#include "robot_arm_node/motion/urdf_kinematics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Solver = robot_arm_node::motion::UrdfKinematics;
using Joints = std::vector<double>;

std::string read_urdf()
{
  std::ifstream input(TEST_ARM_URDF_PATH);
  if (!input) {
    throw std::runtime_error("cannot read arm test URDF: " TEST_ARM_URDF_PATH);
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

double tool_tilt(const KDL::Frame &frame)
{
  const auto axis = frame.M.UnitZ();
  return std::atan2(axis.z(), std::hypot(axis.x(), axis.y()));
}

double joint_distance(const Joints &joints, const Joints &seed)
{
  double result = 0.0;
  for (std::size_t i = 0U; i < 4U; ++i) {
    result += (joints[i] - seed[i]) * (joints[i] - seed[i]);
  }
  return result;
}

void expect_solution(
  const Solver &solver, const Joints &joints, const KDL::Vector &target,
  const double tilt, const double joint5)
{
  ASSERT_EQ(joints.size(), 5U);
  for (std::size_t i = 0U; i < joints.size(); ++i) {
    EXPECT_TRUE(std::isfinite(joints[i]));
    EXPECT_GE(joints[i], solver.lower_limits()[i]);
    EXPECT_LE(joints[i], solver.upper_limits()[i]);
  }
  EXPECT_DOUBLE_EQ(joints[4], joint5);
  const auto frame = solver.forward(joints);
  EXPECT_LE((frame.p - target).Norm(), 1e-3);
  EXPECT_LE(std::abs(tool_tilt(frame) - tilt), 1e-2);
}

class UrdfTiltIk : public ::testing::Test
{
protected:
  Solver solver_{read_urdf(), "base_link", "tcp_link"};
  const Joints zero_{0.0, 0.0, 0.0, 0.0, 0.0};
};

TEST_F(UrdfTiltIk, FailedZeroSeedRecoversWithDefaultMultistart)
{
  EXPECT_THROW(
    solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.0, 1U),
    std::runtime_error);
  const auto solution = solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.0);
  expect_solution(solver_, solution, {0.35, 0.0, 0.25}, 0.0, 0.0);
  EXPECT_EQ(zero_, (Joints{0.0, 0.0, 0.0, 0.0, 0.0}));
}

TEST_F(UrdfTiltIk, PreviouslyReportedTargetsRecoverFromZero)
{
  for (const auto &target : std::vector<KDL::Vector>{{0.35, 0.0, 0.35}, {0.30, 0.0, 0.20}}) {
    const auto solution = solver_.inverse_xyz_tilt_joint5(
      zero_, target.x(), target.y(), target.z(), 0.0, 0.0);
    expect_solution(solver_, solution, target, 0.0, 0.0);
  }
}

TEST_F(UrdfTiltIk, AlreadyValidSeedIsReturnedUnchanged)
{
  const Joints seed{0.2, 0.5, 1.5, -0.2, 0.3};
  const auto target = solver_.forward(seed);
  const auto solution = solver_.inverse_xyz_tilt_joint5(
    seed, target.p.x(), target.p.y(), target.p.z(), tool_tilt(target), seed[4]);
  EXPECT_EQ(solution, seed);
}

TEST_F(UrdfTiltIk, FallbackPrefersNearbySolution)
{
  const auto reference = solver_.inverse_xyz_tilt_joint5(
    {0.0, 0.5, 1.5, 0.5, 0.0}, 0.35, 0.0, 0.25, 0.0, 0.0, 1U);
  const auto solution = solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.0);
  EXPECT_LE(joint_distance(solution, zero_), joint_distance(reference, zero_) + 1e-12);
}

TEST_F(UrdfTiltIk, RepeatedRequestsAreDeterministic)
{
  const auto first = solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.0);
  const auto second = solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.0);
  EXPECT_EQ(first, second);
}

TEST_F(UrdfTiltIk, RequestedJoint5IsFixedDuringFallback)
{
  const auto solution = solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.4);
  expect_solution(solver_, solution, {0.35, 0.0, 0.25}, 0.0, 0.4);
}

TEST_F(UrdfTiltIk, NonzeroYAndTiltTargetsFromForwardKinematics)
{
  for (const auto &truth : std::vector<Joints>{
      {0.2, 0.6, 1.2, 0.3, 0.4}, {-0.3, 0.4, 1.7, -0.4, -0.3},
      {0.1, 0.8, 0.7, 0.1, 0.0}}) {
    const auto target = solver_.forward(truth);
    const auto solution = solver_.inverse_xyz_tilt_joint5(
      zero_, target.p.x(), target.p.y(), target.p.z(), tool_tilt(target), truth[4]);
    expect_solution(solver_, solution, target.p, tool_tilt(target), truth[4]);
  }
}

TEST_F(UrdfTiltIk, FallbackRespectsNarrowedUrdfLimits)
{
  auto xml = std::regex_replace(read_urdf(), std::regex("lower=\"[^\"]*\""), "lower=\"-0.8\"");
  xml = std::regex_replace(xml, std::regex("upper=\"[^\"]*\""), "upper=\"2.5\"");
  Solver bounded(xml, "base_link", "tcp_link");
  for (std::size_t i = 0U; i < bounded.joint_count(); ++i) {
    ASSERT_DOUBLE_EQ(bounded.lower_limits()[i], -0.8);
    ASSERT_DOUBLE_EQ(bounded.upper_limits()[i], 2.5);
  }
  const auto solution = bounded.inverse_xyz_tilt_joint5(
    {10.0, -10.0, 10.0, -10.0, 10.0}, 0.35, 0.0, 0.25, 0.0, 0.0);
  expect_solution(bounded, solution, {0.35, 0.0, 0.25}, 0.0, 0.0);
}

TEST_F(UrdfTiltIk, InvalidSeedIsRejectedWithoutFallback)
{
  EXPECT_THROW(
    solver_.inverse_xyz_tilt_joint5({0.0}, 0.35, 0.0, 0.25, 0.0, 0.0),
    std::invalid_argument);
  auto invalid = zero_;
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    solver_.inverse_xyz_tilt_joint5(invalid, 0.35, 0.0, 0.25, 0.0, 0.0),
    std::invalid_argument);
}

TEST_F(UrdfTiltIk, InvalidTargetIsRejectedWithoutFallback)
{
  EXPECT_THROW(
    solver_.inverse_xyz_tilt_joint5(zero_, std::numeric_limits<double>::infinity(), 0.0, 0.25, 0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 2.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 100.0),
    std::invalid_argument);
}

TEST_F(UrdfTiltIk, AttemptCountIsBounded)
{
  for (const auto count : {0U, 65U}) {
    EXPECT_THROW(
      solver_.inverse_xyz_tilt_joint5(zero_, 0.35, 0.0, 0.25, 0.0, 0.0, count),
      std::invalid_argument);
  }
}

TEST_F(UrdfTiltIk, UnreachableTargetReportsAttemptsAndResiduals)
{
  try {
    solver_.inverse_xyz_tilt_joint5(zero_, 10.0, 0.0, 0.25, 0.0, 0.0, 4U);
    FAIL() << "unreachable target unexpectedly solved";
  } catch (const std::runtime_error &error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("all 4 initial seeds"), std::string::npos);
    EXPECT_NE(message.find("first seed:"), std::string::npos);
    EXPECT_NE(message.find("last seed:"), std::string::npos);
    EXPECT_NE(message.find("residual position="), std::string::npos);
  }
}
}  // namespace
