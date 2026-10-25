#pragma once

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace robot_arm_node::motion
{
class UrdfKinematics
{
public:
  // Builds a base_link -> tcp_link chain from robot_description XML.
  UrdfKinematics(
    const std::string &robot_description,
    const std::string &base_link,
    const std::string &tip_link,
    unsigned int max_iterations = 200,
    double tolerance = 1e-3);//1mm

  UrdfKinematics(const UrdfKinematics &) = delete;
  UrdfKinematics &operator=(const UrdfKinematics &) = delete;

  // Returns the end-effector frame in base_link coordinates.
  KDL::Frame forward(const std::vector<double> &joint_positions) const;

  // seed must contain one value for every movable joint in joint_names().
  std::vector<double> inverse(
    const std::vector<double> &seed,
    const KDL::Frame &target) const;

  std::vector<double> inverse_xyz_pitch_roll(
  const std::vector<double> &seed,
  const double target_x,
  const double target_y,
  const double target_z,
  const double target_pitch,
  const double target_roll) const;
  
  //改进pitch->水平面
  std::vector<double> inverse_xyz_tilt_joint5(
  const std::vector<double> &seed,
  double target_x,
  double target_y,
  double target_z,
  double target_tool_tilt,
  double target_joint5) const;

  const std::vector<std::string> &joint_names() const noexcept;
  const std::vector<double> &lower_limits() const noexcept;
  const std::vector<double> &upper_limits() const noexcept;
  std::size_t joint_count() const noexcept;

private:
  KDL::JntArray to_kdl(const std::vector<double> &values) const;
  std::vector<double> from_kdl(const KDL::JntArray &values) const;

  KDL::Chain chain_;
  unsigned int max_iterations_ ;
  double tolerance_ ;
  KDL::JntArray lower_limits_;
  KDL::JntArray upper_limits_;
  std::vector<std::string> joint_names_;
  std::vector<double> lower_limits_values_;
  std::vector<double> upper_limits_values_;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> forward_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> velocity_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> inverse_solver_;
};
}  // namespace robot_arm_node::motion
