#include "robot_arm_node/motion/urdf_kinematics.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <Eigen/Dense>
#include <algorithm>
namespace robot_arm_node::motion
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

// 统一角度到[-180°，180°]
double wrap_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

// 手写5个变量的位姿error获取
Eigen::Matrix<double, 5, 1> task_error(
  const KDL::Frame &current,
  const double target_x,
  const double target_y,
  const double target_z,
  const double target_pitch,
  const double target_roll)
{
  double current_roll{};
  double current_pitch{};
  double current_yaw{};

  current.M.GetRPY(current_roll, current_pitch, current_yaw);

  Eigen::Matrix<double, 5, 1> error;

  error(0) = target_x - current.p.x();
  error(1) = target_y - current.p.y();
  error(2) = target_z - current.p.z();
  error(3) = wrap_angle(target_pitch - current_pitch);
  error(4) = wrap_angle(target_roll - current_roll);

  return error;
}

bool is_movable(const KDL::Joint &joint)
{
  return joint.getType() != KDL::Joint::None;
}
}  // namespace

UrdfKinematics::UrdfKinematics(
  const std::string &robot_description,
  const std::string &base_link,
  const std::string &tip_link,
  const unsigned int max_iterations,
  const double tolerance) :max_iterations_(max_iterations), tolerance_(tolerance)
{
  if (robot_description.empty()) {
    throw std::invalid_argument("robot_description is empty");
  }
  if (base_link.empty() || tip_link.empty()) {
    throw std::invalid_argument("base_link and tip_link must not be empty");
  }
  if (max_iterations == 0U || !std::isfinite(tolerance) || tolerance <= 0.0) {
    throw std::invalid_argument("invalid IK solver settings");
  }

  urdf::Model model;
  if (!model.initString(robot_description)) {
    throw std::runtime_error("failed to parse robot_description as URDF");
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::runtime_error("failed to construct KDL tree from robot_description");
  }
  if (!tree.getChain(base_link, tip_link, chain_)) {
    throw std::runtime_error(
      "failed to find KDL chain from " + base_link + " to " + tip_link);
  }

  const auto joint_count = chain_.getNrOfJoints();
  if (joint_count == 0U) {
    throw std::runtime_error("the selected KDL chain has no movable joints");
  }
  
  lower_limits_ = KDL::JntArray(joint_count);
  upper_limits_ = KDL::JntArray(joint_count);

  unsigned int index = 0U;
  for (unsigned int segment_index = 0U;
       segment_index < chain_.getNrOfSegments(); ++segment_index) {
    const auto &joint = chain_.getSegment(segment_index).getJoint();
    if (!is_movable(joint)) {
      continue;
    }

    joint_names_.push_back(joint.getName());
    double lower = -kPi;
    double upper = kPi;

    const auto urdf_joint = model.getJoint(joint.getName());
    if (urdf_joint && urdf_joint->type != urdf::Joint::CONTINUOUS && urdf_joint->limits) {
      lower = urdf_joint->limits->lower;
      upper = urdf_joint->limits->upper;
    }
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
      throw std::runtime_error("invalid limits for joint " + joint.getName());
    }

    lower_limits_(index) = lower;
    upper_limits_(index) = upper;
    lower_limits_values_.push_back(lower);
    upper_limits_values_.push_back(upper);
    ++index;
  }

  forward_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
  inverse_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
    chain_, lower_limits_, upper_limits_, *forward_solver_, *velocity_solver_,
    max_iterations, tolerance);
}

KDL::Frame UrdfKinematics::forward(const std::vector<double> &joint_positions) const
{
  const auto joints = to_kdl(joint_positions);
  KDL::Frame result;
  const int status = forward_solver_->JntToCart(joints, result);
  if (status < 0) {
    throw std::runtime_error("KDL forward kinematics failed with code " + std::to_string(status));
  }
  return result;
}

std::vector<double> UrdfKinematics::inverse(
  const std::vector<double> &seed,
  const KDL::Frame &target) const
{
  const auto initial = to_kdl(seed);
  KDL::JntArray result(joint_count());
  const int status = inverse_solver_->CartToJnt(initial, target, result);
  if (status < 0) {
    throw std::runtime_error("KDL inverse kinematics failed with code " + std::to_string(status));
  }
  return from_kdl(result);
}

std::vector<double> UrdfKinematics::inverse_xyz_pitch_roll(
  const std::vector<double> &seed,
  const double target_x,
  const double target_y,
  const double target_z,
  const double target_pitch,
  const double target_roll) const
{
  if (joint_count() != 5U) {
    throw std::runtime_error(
      "xyz + pitch + roll IK currently requires exactly five movable joints");
  }

  KDL::JntArray q = to_kdl(seed);

  for (unsigned int i = 0; i < q.rows(); ++i) {
    q(i) = std::clamp(q(i), lower_limits_(i), upper_limits_(i));
  }

  constexpr double kJointStep = 1e-6;
  constexpr double kDamping = 1e-5;
  constexpr double kMaxStep = 0.25;

  const double position_tolerance = std::max(tolerance_, 1e-5);
  const double angle_tolerance = std::max(tolerance_, 1e-5);
  Eigen::Matrix<double, 5, 1> final_error =
    Eigen::Matrix<double, 5, 1>::Zero();

  for (unsigned int iteration = 0; iteration < max_iterations_; ++iteration) {
    KDL::Frame current_frame;

    const int fk_status = forward_solver_->JntToCart(q, current_frame);

    if (fk_status < 0) {
      throw std::runtime_error(
        "FK failed during constrained IK");
    }

    //实际误差用于判断是否走到位
    const auto error = task_error(
      current_frame,
      target_x,
      target_y,
      target_z,
      target_pitch,
      target_roll);
    final_error = error;

    if ( error.head<3>().norm() <= position_tolerance && std::abs(error(3)) <= angle_tolerance && std::abs(error(4)) <= angle_tolerance) {
      return from_kdl(q);
    }
    // if ( error.head<3>().norm() <= position_tolerance && std::abs(error(4)) <= angle_tolerance) {
    //   return from_kdl(q);
    // }

    Eigen::Matrix<double, 5, 5> jacobian;

    for (unsigned int column = 0; column < 5U; ++column) {
      KDL::JntArray q_plus = q;
      KDL::JntArray q_minus = q;

      q_plus(column) += kJointStep;
      q_minus(column) -= kJointStep;

      KDL::Frame frame_plus;
      KDL::Frame frame_minus;

      if (forward_solver_->JntToCart(q_plus, frame_plus) < 0 || forward_solver_->JntToCart(q_minus, frame_minus) < 0) {
        throw std::runtime_error("FK failed while calculating IK Jacobian");
      }

      const auto error_plus = task_error(
        frame_plus,
        target_x,
        target_y,
        target_z,
        target_pitch,
        target_roll);

      const auto error_minus = task_error(
        frame_minus,
        target_x,
        target_y,
        target_z,
        target_pitch,
        target_roll);

      jacobian.col(column) =
        (error_plus - error_minus) / (2.0 * kJointStep);
    }
    //构造单位矩阵
    const Eigen::Matrix<double, 5, 5> identity =
      Eigen::Matrix<double, 5, 5>::Identity();
    //雅可比x雅可比转置+阻尼，阻尼就是个对角矩阵，防止有奇异点时雅可比x雅可比转置有些为0，不可逆
    //既然雅可比不能求逆，我们构造一个永远可逆的矩阵来代替它
    //这就是阻尼最小二乘法（DLS）的核心思想
    const Eigen::Matrix<double, 5, 5> system =
      jacobian * jacobian.transpose() +
      kDamping * kDamping * identity;

    const Eigen::Matrix<double, 5, 1> delta =
      -jacobian.transpose() *
      system.ldlt().solve(error);

    Eigen::Matrix<double, 5, 1> limited_delta = delta;

    const double step_norm = limited_delta.norm();

    if (step_norm > kMaxStep) {
      limited_delta *= kMaxStep / step_norm;
    }

    for (unsigned int i = 0; i < 5U; ++i) {
      q(i) = std::clamp(q(i) + limited_delta(i), lower_limits_(i), upper_limits_(i));
    }
  }

  std::ostringstream message;
  message << "xyz + pitch + roll IK did not converge after " << max_iterations_
          << " iterations; residual position=" << final_error.head<3>().norm()
          << " m, pitch=" << final_error(3) << " rad, roll=" << final_error(4)
          << " rad";
  throw std::runtime_error(message.str());
}

std::vector<double> UrdfKinematics::inverse_xyz_tilt_joint5(
  const std::vector<double> &seed,
  const double target_x,
  const double target_y,
  const double target_z,
  const double target_tool_tilt,
  const double target_joint5) const
{
  using Error = Eigen::Matrix<double, 4, 1>;
  using Jacobian = Eigen::Matrix<double, 4, 4>;

  constexpr double kHalfPi = 1.57079632679489661923;
  constexpr double kPositionTolerance = 1e-3;  // 1 mm，调试起点
  constexpr double kTiltTolerance = 1e-2;      // 约 0.57 度，调试起点
  constexpr double kDifferenceStep = 1e-6;
  constexpr double kDamping = 1e-3;
  constexpr double kMaxStep = 0.25;           // 前四轴增量的二范数，rad
  constexpr double kAngleScale = 0.1;        // m/rad，任务权重

  if (joint_count() != 5U || joint_names_[4] != "joint5") {
    throw std::runtime_error(
      "xyz + tool tilt IK requires five joints with joint5 last");
  }
  if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(target_z) || !std::isfinite(target_tool_tilt) ||
      !std::isfinite(target_joint5)) {
    throw std::invalid_argument("IK target contains a non-finite value");
  }
  if (target_tool_tilt < -kHalfPi || target_tool_tilt > kHalfPi) {
    throw std::invalid_argument("tool tilt must be within [-pi/2, pi/2]");
  }
  if (target_joint5 < lower_limits_(4) || target_joint5 > upper_limits_(4)) {
    throw std::invalid_argument("target joint5 is outside URDF limits");
  }

  KDL::JntArray q = to_kdl(seed);
  for (unsigned int j = 0; j < 4U; ++j) {
    q(j) = std::clamp(q(j), lower_limits_(j), upper_limits_(j));
  }
  q(4) = target_joint5;  // 求解前固定，整个迭代期间都不改变

  const auto evaluate = [&](const KDL::JntArray &joints) -> Error {
    KDL::Frame frame;
    if (forward_solver_->JntToCart(joints, frame) < 0) {
      throw std::runtime_error("FK failed during xyz + tool tilt IK");
    }
    // TCP 正 Z 轴在 base_link 坐标系下的方向：旋转矩阵第三列。
    const KDL::Vector tool_z = frame.M.UnitZ();
    const double tilt = std::atan2(
      tool_z.z(), std::hypot(tool_z.x(), tool_z.y()));

    Error e;
    e << target_x - frame.p.x(),
         target_y - frame.p.y(),
         target_z - frame.p.z(),
         target_tool_tilt - tilt;
    return e;
  };
  const auto weighted = [](Error e) -> Error {
    e(3) *= kAngleScale;
    return e;
  };

  unsigned int updates = 0U;
  const char *reason = "iteration limit";
  while (true) {
    const Error error = evaluate(q);
    if (!error.allFinite()) {
      throw std::runtime_error("non-finite task error during IK");
    }
    if (error.head<3>().norm() <= kPositionTolerance &&
        std::abs(error(3)) <= kTiltTolerance) {
      return from_kdl(q);  // 返回五个角度，其中第五轴就是指定值
    }
    // 最后一次更新后也检查收敛，避免遗漏最后得到的可行解。
    if (updates >= max_iterations_) {
      break;
    }

    Jacobian jacobian;
    for (unsigned int column = 0; column < 4U; ++column) {
      KDL::JntArray q_plus = q;
      KDL::JntArray q_minus = q;
      q_plus(column) = std::min(
        q(column) + kDifferenceStep, upper_limits_(column));
      q_minus(column) = std::max(
        q(column) - kDifferenceStep, lower_limits_(column));
      const double span = q_plus(column) - q_minus(column);
      if (span <= 0.0) {
        throw std::runtime_error("invalid finite-difference interval");
      }
      // 求的是误差对关节角的导数，因此下方更新公式带负号。
      jacobian.col(column) =
        weighted((evaluate(q_plus) - evaluate(q_minus)) / span);
    }
    if (!jacobian.allFinite()) {
      throw std::runtime_error("non-finite IK Jacobian");
    }

    const Error scaled_error = weighted(error);
    const Jacobian system = jacobian * jacobian.transpose() +
      kDamping * kDamping * Jacobian::Identity();
    const Eigen::LDLT<Jacobian> decomposition(system);
    if (decomposition.info() != Eigen::Success) {
      throw std::runtime_error("IK linear system decomposition failed");
    }
    Error delta = -jacobian.transpose() * decomposition.solve(scaled_error);
    if (decomposition.info() != Eigen::Success || !delta.allFinite()) {
      throw std::runtime_error("IK linear system solve failed");
    }
    const double step_norm = delta.norm();
    if (step_norm > kMaxStep) {
      delta *= kMaxStep / step_norm;
    }

    bool accepted = false;
    double scale = 1.0;
    for (unsigned int trial = 0; trial < 12U; ++trial) {
      KDL::JntArray candidate = q;
      for (unsigned int j = 0; j < 4U; ++j) {
        candidate(j) = std::clamp(
          q(j) + scale * delta(j), lower_limits_(j), upper_limits_(j));
      }
      candidate(4) = target_joint5;
      const Error candidate_error = evaluate(candidate);
      if (candidate_error.allFinite() &&
          weighted(candidate_error).squaredNorm() < scaled_error.squaredNorm()) {
        q = candidate;
        accepted = true;
        break;
      }
      scale *= 0.5;
    }
    ++updates;
    if (!accepted) {
      reason = "no improving step; inspect seed, limits, and reachability";
      break;
    }
  }

  const Error final_error = evaluate(q);
  std::ostringstream message;
  message << "xyz + tool tilt IK failed after " << updates
          << " updates (" << reason << "); residual position="
          << final_error.head<3>().norm() << " m, tool_tilt="
          << final_error(3) << " rad, fixed joint5=" << q(4) << " rad";
  throw std::runtime_error(message.str());
}


const std::vector<std::string> &UrdfKinematics::joint_names() const noexcept
{
  return joint_names_;
}

const std::vector<double> &UrdfKinematics::lower_limits() const noexcept
{
  return lower_limits_values_;
}

const std::vector<double> &UrdfKinematics::upper_limits() const noexcept
{
  return upper_limits_values_;
}

std::size_t UrdfKinematics::joint_count() const noexcept
{
  return joint_names_.size();
}

KDL::JntArray UrdfKinematics::to_kdl(const std::vector<double> &values) const
{
  if (values.size() != joint_count()) {
    throw std::invalid_argument(
      "joint vector has " + std::to_string(values.size()) +
      " values; expected " + std::to_string(joint_count()));
  }

  KDL::JntArray result(joint_count());
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument("joint vector contains a non-finite value");
    }
    result(static_cast<unsigned int>(index)) = values[index];
  }
  return result;
}

std::vector<double> UrdfKinematics::from_kdl(const KDL::JntArray &values) const
{
  std::vector<double> result(values.rows());
  for (unsigned int index = 0U; index < values.rows(); ++index) {
    result[index] = values(index);
  }
  return result;
}
}  // namespace robot_arm_node::motion
