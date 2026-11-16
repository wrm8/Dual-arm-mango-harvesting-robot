#include "robot_arm_node/motion/urdf_kinematics.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <random>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Solver = robot_arm_node::motion::UrdfKinematics;
using Joints = std::vector<double>;
constexpr double kPi = 3.14159265358979323846;

std::string read_urdf()
{
  std::ifstream file(TEST_ARM_URDF_PATH);
  if (!file) {throw std::runtime_error("cannot read test URDF");}
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
double tilt_of(const KDL::Frame &frame)
{
  const auto axis = frame.M.UnitZ();
  return std::atan2(axis.z(), std::hypot(axis.x(), axis.y()));
}
void expect_solution(const Solver &solver, const Joints &q, const KDL::Vector &p,
  double tilt, double joint5)
{
  ASSERT_EQ(q.size(), 5U);
  for (std::size_t i = 0; i < q.size(); ++i) {
    EXPECT_TRUE(std::isfinite(q[i]));
    EXPECT_GE(q[i], solver.lower_limits()[i]);
    EXPECT_LE(q[i], solver.upper_limits()[i]);
  }
  EXPECT_DOUBLE_EQ(q[4], joint5);
  const auto fk = solver.forward(q);
  EXPECT_LE((fk.p - p).Norm(), 1e-6);
  EXPECT_LE(std::abs(tilt_of(fk) - tilt), 1e-5);
}
class AnalyticIk : public ::testing::Test
{
protected:
  Solver solver_{read_urdf(), "base_link", "tcp_link"};
  const Joints zero_{0, 0, 0, 0, 0};
};
TEST_F(AnalyticIk, ReportedXyzTargetsWithHorizontalTool)
{
  for (const KDL::Vector &p : {KDL::Vector(0.35, 0, 0.25), {0.30, 0, 0.20}, {0.35, 0, 0.35}}) {
    const auto q = solver_.inverse_xyz_tilt_joint5_analytic(zero_, p.x(), p.y(), p.z(), 0, 0);
    expect_solution(solver_, q, p, 0, 0);
  }
}
TEST_F(AnalyticIk, HundredsOfOriginalUrdfForwardTargets)
{
  std::mt19937 random(20261106);
  std::uniform_real_distribution<double> angle(-2.8, 2.8);
  for (unsigned int i = 0; i < 600U; ++i) {
    Joints truth(5);
    for (auto &value : truth) {value = angle(random);}
    SCOPED_TRACE(::testing::Message() << "case " << i);
    const auto fk = solver_.forward(truth);
    const auto q = solver_.inverse_xyz_tilt_joint5_analytic(
      zero_, fk.p.x(), fk.p.y(), fk.p.z(), tilt_of(fk), truth[4]);
    expect_solution(solver_, q, fk.p, tilt_of(fk), truth[4]);
  }
}
TEST_F(AnalyticIk, NearestSeedSelectsEitherElbowBranch)
{
  const Joints truth{0.2, 0.5, 1.3, -0.1, 0.4};
  const auto fk = solver_.forward(truth);
  for (const Joints &seed : {Joints{0.2, 0.48, 1.28, -0.08, 0.4},
      Joints{0.2, 1.5, -1.0, 1.0, 0.4}}) {
    const auto q = solver_.inverse_xyz_tilt_joint5_analytic(
      seed, fk.p.x(), fk.p.y(), fk.p.z(), tilt_of(fk), truth[4]);
    expect_solution(solver_, q, fk.p, tilt_of(fk), truth[4]);
    EXPECT_EQ(q[2] > 0, seed[2] > 0);
  }
}
TEST_F(AnalyticIk, AlreadyValidSeedUnchangedAndRequestsDeterministic)
{
  const Joints truth{0.3, 0.5, 1.2, -0.2, -0.6};
  const auto fk = solver_.forward(truth);
  EXPECT_EQ(truth, solver_.inverse_xyz_tilt_joint5_analytic(
      truth, fk.p.x(), fk.p.y(), fk.p.z(), tilt_of(fk), truth[4]));
  EXPECT_EQ(solver_.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0),
    solver_.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0));
}
TEST_F(AnalyticIk, EffectiveCalibratedLimitsFilterElbowBranches)
{
  // Actual current arm_driver.yaml mapping, including reversed joint3.
  const Joints zero_ticks{1908, 2012, 1640, 1491, 518};
  const Joints min_ticks{1470, 1430, 1640, 840, 135};
  const Joints max_ticks{2380, 3500, 3650, 3640, 890};
  const Joints scale{651.8986, 651.8986, -651.8986, 651.8986, 238.7324};
  Joints lower(5), upper(5);
  for (std::size_t i = 0; i < 5; ++i) {
    const auto a = (min_ticks[i] - zero_ticks[i]) / scale[i];
    const auto b = (max_ticks[i] - zero_ticks[i]) / scale[i];
    lower[i] = std::min(a, b);
    upper[i] = std::max(a, b);
  }
  Solver bounded(read_urdf(), "base_link", "tcp_link", 200, 1e-3, lower, upper);
  const auto q = bounded.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0);
  expect_solution(bounded, q, {.35, 0, .25}, 0, 0);
  EXPECT_LE(q[2], 0);
}
TEST_F(AnalyticIk, ManyTargetsInsideCalibratedLimits)
{
  const Joints lower{-.6, -.8, -3.0, -.9, -1.5}, upper{.7, 2.2, 0, 3.1, 1.5};
  Solver bounded(read_urdf(), "base_link", "tcp_link", 200, 1e-3, lower, upper);
  std::mt19937 random(83);
  for (unsigned int i = 0; i < 200; ++i) {
    Joints truth(5);
    for (std::size_t j = 0; j < 5; ++j) {
      truth[j] = std::uniform_real_distribution<double>(lower[j] + .02, upper[j] - .02)(random);
    }
    const auto fk = bounded.forward(truth);
    const auto q = bounded.inverse_xyz_tilt_joint5_analytic(
      zero_, fk.p.x(), fk.p.y(), fk.p.z(), tilt_of(fk), truth[4]);
    expect_solution(bounded, q, fk.p, tilt_of(fk), truth[4]);
  }
}
TEST_F(AnalyticIk, SingularAndJointBoundaryTargetsAreFkVerified)
{
  for (const Joints &truth : {zero_, Joints{0, .5, .158466, .1, 0},
      Joints{3.14, -.5, -3.14, 1.5, 3.14}, Joints{-.3, -.5, .8, -.3, 0}}) {
    const auto fk = solver_.forward(truth);
    const auto q = solver_.inverse_xyz_tilt_joint5_analytic(
      Joints{.1, .1, .1, .1, 0}, fk.p.x(), fk.p.y(), fk.p.z(), tilt_of(fk), truth[4]);
    expect_solution(solver_, q, fk.p, tilt_of(fk), truth[4]);
  }
}
TEST_F(AnalyticIk, UnsupportedTopologyFailsExplicitly)
{
  auto xml = read_urdf();
  const auto at = xml.find(".2255");
  ASSERT_NE(at, std::string::npos);
  xml.replace(at, 5, ".2455");
  Solver changed(xml, "base_link", "tcp_link");
  const Joints truth{.4, .7, 1.4, -.5, .3};
  const auto fk = changed.forward(truth);
  expect_solution(changed, changed.inverse_xyz_tilt_joint5_analytic(
      zero_, fk.p.x(), fk.p.y(), fk.p.z(), tilt_of(fk), truth[4]), fk.p, tilt_of(fk), truth[4]);
  // A different tip is not silently treated as the six-segment TCP chain.
  Solver unsupported(read_urdf(), "base_link", "link5");
  EXPECT_THROW(unsupported.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0),
    std::invalid_argument);
  xml = read_urdf();
  const auto tcp_joint = xml.find("<joint name=\"joint_tcp\"");
  ASSERT_NE(tcp_joint, std::string::npos);
  const auto tcp = xml.find("xyz=\"0 0 ", tcp_joint);
  ASSERT_NE(tcp, std::string::npos);
  xml.replace(tcp, std::string("xyz=\"0 0 ").size(), "xyz=\"0.01 0 ");
  Solver off_axis_tcp(xml, "base_link", "tcp_link");
  EXPECT_THROW(off_axis_tcp.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0),
    std::invalid_argument);
}
TEST_F(AnalyticIk, InvalidOrUnreachableTargetsNeverReturnAngles)
{
  EXPECT_THROW(solver_.inverse_xyz_tilt_joint5_analytic({0}, .35, 0, .25, 0, 0), std::invalid_argument);
  EXPECT_THROW(solver_.inverse_xyz_tilt_joint5_analytic(
      zero_, std::numeric_limits<double>::infinity(), 0, .25, 0, 0), std::invalid_argument);
  EXPECT_THROW(solver_.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, kPi, 0), std::invalid_argument);
  EXPECT_THROW(solver_.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 4), std::invalid_argument);
  EXPECT_THROW(solver_.inverse_xyz_tilt_joint5_analytic(zero_, 10, 0, .25, 0, 0), std::runtime_error);
  Joints lo(5, -.01), hi(5, .01);
  Solver blocked(read_urdf(), "base_link", "tcp_link", 200, 1e-3, lo, hi);
  EXPECT_THROW(blocked.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0), std::runtime_error);
}
TEST_F(AnalyticIk, InvalidLimitOverridesRejected)
{
  EXPECT_THROW(Solver(read_urdf(), "base_link", "tcp_link", 200, 1e-3, {0}, {1}), std::invalid_argument);
  EXPECT_THROW(Solver(read_urdf(), "base_link", "tcp_link", 200, 1e-3,
      Joints(5, 4), Joints(5, 5)), std::runtime_error);
}

TEST_F(AnalyticIk, BothPrefixedChainsInDualModelKeepLocalFkAndAnalyticIk)
{
  const auto original = read_urdf();
  const auto body_begin = original.find('>', original.find("<robot")) + 1U;
  const auto body = original.substr(body_begin, original.rfind("</robot>") - body_begin);
  std::string dual = "<robot name=\"dual_test\"><link name=\"camera_color_optical_frame\"/>";
  for (const auto &side : {std::string("left"), std::string("right")}) {
    dual += std::regex_replace(body, std::regex("(name|link)=\"([^\"]+)\""),
      "$1=\"" + side + "_$2\"");
    dual += "<joint name=\"" + side + "_mount\" type=\"fixed\">"
      "<parent link=\"camera_color_optical_frame\"/><child link=\"" + side + "_base_link\"/>"
      "<origin xyz=\"0.2 -0.1 0.3\" rpy=\"0.4 -0.5 0.6\"/></joint>";
  }
  dual += "</robot>";
  for (const auto &side : {std::string("left"), std::string("right")}) {
    Solver prefixed(dual, side + "_base_link", side + "_tcp_link");
    ASSERT_EQ(prefixed.joint_count(), 5U);
    for (std::size_t i = 0; i < 5U; ++i) {
      EXPECT_EQ(prefixed.joint_names()[i], side + "_joint" + std::to_string(i + 1U));
    }
    const auto fk = prefixed.forward({.2, .5, 1.3, -.1, .4});
    const auto single_fk = solver_.forward({.2, .5, 1.3, -.1, .4});
    EXPECT_LE(KDL::diff(fk, single_fk).vel.Norm(), 1e-12);
    EXPECT_LE(KDL::diff(fk, single_fk).rot.Norm(), 1e-12);
    const auto q = prefixed.inverse_xyz_tilt_joint5_analytic(zero_, .35, 0, .25, 0, 0);
    expect_solution(prefixed, q, {.35, 0, .25}, 0, 0);
  }
}
}  // namespace
