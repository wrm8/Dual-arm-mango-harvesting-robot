#include "robot_arm_node/motion/urdf_kinematics.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace robot_arm_node::motion
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kPositionTolerance = 1e-6;
constexpr double kTiltTolerance = 1e-5;
constexpr double kGeometryTolerance = 2e-5;
constexpr double kLimitSlack = 1e-4;  // Only for cardinal-angle rounding in the URDF.
using Joints = std::vector<double>;
using Error = Eigen::Matrix<double, 4, 1>;

double tilt_of(const KDL::Frame &frame)
{
  const auto z = frame.M.UnitZ();
  return std::atan2(z.z(), std::hypot(z.x(), z.y()));
}

// Revolute joints are bounded: choose a legal 2*pi representative, then use
// ordinary joint distance. A shortest-angle wrap must not cross a hard stop.
bool equivalent_in_limits(double angle, double lower, double upper, double seed, double &result)
{
  const double first = std::ceil((lower - kLimitSlack - angle) / kTwoPi);
  const double last = std::floor((upper + kLimitSlack - angle) / kTwoPi);
  if (first > last) {
    return false;
  }
  const double turns = std::clamp(std::round((seed - angle) / kTwoPi), first, last);
  result = std::clamp(angle + turns * kTwoPi, lower, upper);
  return true;
}

struct Geometry
{
  std::array<KDL::Frame, 6> origin;
  explicit Geometry(const KDL::Chain &chain)
  {
    // This is a solver for this robot's yaw + three parallel pitch axes +
    // tool-roll topology, not an unverified solver for arbitrary five-axis arms.
    if (chain.getNrOfSegments() != 6U || chain.getNrOfJoints() != 5U) {
      throw std::invalid_argument("analytic IK requires joint1..joint5 and one fixed TCP joint");
    }
    const std::array<KDL::Rotation, 6> rotations{
      KDL::Rotation::Identity(), KDL::Rotation::RotX(kPi / 2.0),
      KDL::Rotation::RotX(-kPi), KDL::Rotation::RotX(kPi),
      KDL::Rotation::RotX(-kPi / 2.0), KDL::Rotation::Identity()};
    const std::array<double, 5> signs{-1.0, -1.0, 1.0, -1.0, -1.0};
    // Identical arms may be instantiated as left_joint1..5/right_joint1..5.
    // Keep the order and geometry checks for every joint in the selected chain.
    const auto first_name = chain.getSegment(0).getJoint().getName();
    const std::string first_suffix = "joint1";
    if (first_name.size() < first_suffix.size() ||
        first_name.compare(first_name.size() - first_suffix.size(),
          first_suffix.size(), first_suffix) != 0) {
      throw std::invalid_argument("analytic IK chain must start with [prefix]joint1");
    }
    const auto prefix = first_name.substr(0, first_name.size() - first_suffix.size());
    for (unsigned int i = 0; i < 6U; ++i) {
      const auto &segment = chain.getSegment(i);
      origin[i] = segment.pose(0.0);
      const double rotation_error = KDL::diff(rotations[i], origin[i].M).Norm();
      if (rotation_error > kGeometryTolerance) {
        throw std::invalid_argument("analytic IK unsupported URDF origin rotation at segment " +
          std::to_string(i + 1U));
      }
      const auto &joint = segment.getJoint();
      if (i < 5U) {
        if (joint.getName() != prefix + "joint" + std::to_string(i + 1U) ||
            joint.getType() != KDL::Joint::RotAxis ||
            (joint.JointAxis() - origin[i].M * KDL::Vector(0, 0, signs[i])).Norm() >
            kGeometryTolerance) {
          throw std::invalid_argument("analytic IK unsupported axis/order for joint" +
            std::to_string(i + 1U));
        }
      } else if (joint.getType() != KDL::Joint::None ||
                 std::hypot(origin[i].p.x(), origin[i].p.y()) > 1e-9) {
        throw std::invalid_argument("analytic IK requires a fixed TCP on the joint5 Z axis");
      }
    }
    if (std::hypot(origin[2].p.x(), origin[2].p.y()) < 1e-8 ||
        std::hypot(origin[3].p.x(), origin[3].p.y()) < 1e-8) {
      throw std::invalid_argument("analytic IK requires nonzero upper-arm and forearm lengths");
    }
  }
};
}  // namespace

std::vector<double> UrdfKinematics::inverse_xyz_tilt_joint5_analytic(
  const std::vector<double> &seed, double x, double y, double z,
  double tilt, double joint5) const
{
  (void)to_kdl(seed);  // Common size and finite-value validation.
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(tilt) || !std::isfinite(joint5)) {
    throw std::invalid_argument("analytic IK target contains a non-finite value");
  }
  if (tilt < -kPi / 2.0 || tilt > kPi / 2.0) {
    throw std::invalid_argument("tool tilt must be within [-pi/2, pi/2]");
  }
  const Geometry geometry(chain_);
  const auto &o = geometry.origin;
  const auto &lo = lower_limits_values_;
  const auto &hi = upper_limits_values_;
  if (joint5 < lo[4] || joint5 > hi[4]) {
    throw std::invalid_argument("target joint5 is outside effective joint limits");
  }
  const KDL::Vector target(x, y, z);
  const auto error_at = [&](const Joints &q) {
    const auto frame = forward(q);  // Always the ORIGINAL, unsimplified URDF.
    Error error;
    error << x - frame.p.x(), y - frame.p.y(), z - frame.p.z(), tilt - tilt_of(frame);
    return error;
  };
  const auto solved = [](const Error &e) {
    return e.head<3>().norm() <= kPositionTolerance && std::abs(e(3)) <= kTiltTolerance;
  };
  Joints reference = seed;
  bool seed_in_limits = true;
  for (std::size_t i = 0; i < 5U; ++i) {
    seed_in_limits = seed_in_limits && seed[i] >= lo[i] && seed[i] <= hi[i];
    reference[i] = std::clamp(seed[i], lo[i], hi[i]);
  }
  if (seed_in_limits && seed[4] == joint5 && solved(error_at(seed))) {
    return seed;
  }

  // In the shoulder plane:
  //   P = R(-q2)*A + R(-q2-q3)*B + R(-S)*C, S=q2+q3+q4.
  // Rx(-pi) flips the forearm's local Y/Z signs. Preserve all translations.
  const double ax = o[2].p.x(), ay = o[2].p.y();
  const double bx = o[3].p.x(), by = -o[3].p.y();
  const double cx = o[4].p.x(), cy = o[4].p.y() + o[5].p.z();
  const double length_a = std::hypot(ax, ay), length_b = std::hypot(bx, by);
  const double phi_a = std::atan2(ay, ax), phi_b = std::atan2(by, bx);
  const double side = o[1].p.y() - (o[2].p.z() - o[3].p.z() + o[4].p.z());
  const double dx = x - o[0].p.x(), dy = y - o[0].p.y();
  const double radius = std::hypot(dx, dy);
  if (radius + 1e-6 < std::abs(side)) {
    throw std::runtime_error("analytic IK unreachable: target is inside the base-axis lateral offset");
  }
  const double radial_abs = std::sqrt(std::max(0.0, radius * radius - side * side));
  const double sum_abs = std::acos(std::clamp(std::sin(tilt), -1.0, 1.0));
  Joints best;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_position_error = std::numeric_limits<double>::infinity();
  double best_tilt_error = std::numeric_limits<double>::infinity();
  unsigned int geometric_branches = 0U, limit_rejections = 0U, fk_rejections = 0U;

  // Small, bounded correction for the exported 1.5708/3.1416 rotations.
  // This is not a random-seed search. Never return a candidate without exact FK.
  const auto refine = [&](Joints &q) {
    for (unsigned int iteration = 0; iteration < 8U; ++iteration) {
      Error e = error_at(q);
      if (solved(e)) {
        return true;
      }
      Eigen::Matrix4d jacobian;
      for (unsigned int column = 0; column < 4U; ++column) {
        auto plus = q, minus = q;
        plus[column] = std::min(q[column] + 1e-6, hi[column]);
        minus[column] = std::max(q[column] - 1e-6, lo[column]);
        const double span = plus[column] - minus[column];
        if (span <= 0.0) {return false;}
        jacobian.col(column) = (error_at(plus) - error_at(minus)) / span;
      }
      e(3) *= 0.1;
      jacobian.row(3) *= 0.1;
      const Eigen::Matrix4d system = jacobian * jacobian.transpose() +
        1e-12 * Eigen::Matrix4d::Identity();
      Eigen::Vector4d delta = -jacobian.transpose() * system.ldlt().solve(e);
      if (!delta.allFinite()) {return false;}
      if (delta.norm() > 0.03) {delta *= 0.03 / delta.norm();}
      bool improved = false;
      for (double scale = 1.0; scale >= 1.0 / 64.0; scale *= 0.5) {
        auto trial = q;
        for (unsigned int j = 0; j < 4U; ++j) {
          trial[j] = std::clamp(q[j] + scale * delta(j), lo[j], hi[j]);
        }
        auto trial_error = error_at(trial);
        trial_error(3) *= 0.1;
        if (trial_error.squaredNorm() < e.squaredNorm()) {
          q = std::move(trial);
          improved = true;
          break;
        }
      }
      if (!improved) {return false;}
    }
    return solved(error_at(q));
  };

  for (const double radial : {radial_abs, -radial_abs}) {
    // q1 has URDF axis -Z. Retain the fixed base offset and side offset.
    const double yaw = radius < 1e-12 ? reference[0] :
      std::atan2(side, radial) - std::atan2(dy, dx);
    for (const double sum : {sum_abs, -sum_abs}) {
      const double wrist_x = radial - o[1].p.x() - (std::cos(sum) * cx + std::sin(sum) * cy);
      const double wrist_z = z - o[0].p.z() - o[1].p.z() -
        (-std::sin(sum) * cx + std::cos(sum) * cy);
      double cosine = (wrist_x * wrist_x + wrist_z * wrist_z -
        length_a * length_a - length_b * length_b) / (2.0 * length_a * length_b);
      if (cosine < -1.00005 || cosine > 1.00005) {continue;}
      cosine = std::clamp(cosine, -1.0, 1.0);
      for (const double bend : {std::acos(cosine), -std::acos(cosine)}) {
        ++geometric_branches;
        const double alpha = std::atan2(wrist_z, wrist_x) -
          std::atan2(length_b * std::sin(bend), length_a + length_b * std::cos(bend));
        const double q2 = phi_a - alpha;
        const double q3 = phi_b - phi_a - bend;
        Joints candidate{yaw, q2, q3, sum - q2 - q3, joint5};
        bool within = true;
        for (unsigned int j = 0; j < 4U; ++j) {
          if (!equivalent_in_limits(candidate[j], lo[j], hi[j], reference[j], candidate[j])) {
            within = false;
            break;
          }
        }
        if (!within) {++limit_rejections; continue;}
        const bool valid = refine(candidate);
        const auto residual = error_at(candidate);
        if (residual.head<3>().norm() < best_position_error) {
          best_position_error = residual.head<3>().norm();
          best_tilt_error = std::abs(residual(3));
        }
        if (!valid || !solved(residual)) {++fk_rejections; continue;}
        double distance = 0.0;
        for (unsigned int j = 0; j < 4U; ++j) {
          distance += std::pow(candidate[j] - reference[j], 2);
        }
        if (distance < best_distance) {
          best_distance = distance;
          best = std::move(candidate);
        }
      }
    }
  }
  if (!best.empty()) {return best;}
  std::ostringstream message;
  message << "analytic IK: no feasible solution for xyz=(" << target.x() << ", " <<
    target.y() << ", " << target.z() << "), tool_tilt=" << tilt << ", joint5=" << joint5 <<
    "; geometric branches=" << geometric_branches << ", joint-limit rejections=" <<
    limit_rejections << ", exact-URDF FK rejections=" << fk_rejections;
  if (std::isfinite(best_position_error)) {
    message << "; best residual position=" << best_position_error <<
      " m, tool_tilt=" << best_tilt_error << " rad";
  }
  throw std::runtime_error(message.str());
}
}  // namespace robot_arm_node::motion
