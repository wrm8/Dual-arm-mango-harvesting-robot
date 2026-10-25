# 五轴机械臂：XYZ + 工具倾角 + 第五关节角

本方案根据本次提供的 URDF 和 UrdfKinematics 实现编写。它新增一个方法，不要求删除已有 inverse_xyz_pitch_roll()，也不改变 KDL 的正运动学及 URDF 解析部分。

## 目标定义

- XYZ：tcp_link 原点在 base_link 下的位置，单位 m。
- tool_tilt：TCP 的正 Z 轴相对 base_link 的 XY 平面的仰角，单位 rad。水平为 0，向上为 +π/2，向下为 -π/2。
- joint5_angle：joint5 的关节坐标，单位 rad；正方向遵循 URDF 中 axis="0 0 -1"。这是关节角，不是欧拉角 roll，也不是夹爪开口相对工件的绝对方向。

当前 URDF 的 TCP 位于 link5 的 Z 轴线上，且固定旋转为零。因此只改变第五关节时，TCP 位置和正 Z 轴指向不变，适合使用本方案。更换为偏轴 TCP 后，需要重新验证该性质。

工具倾角只描述仰角，不额外指定水平方向；水平方向由机械结构、目标位置和求解分支共同决定。该角在工具恰好竖直时有导数不唯一问题，本方案不是对所有姿态都保证收敛的通用逆解。

## 1. 在 urdf_kinematics.hpp 的 public 中新增声明

```cpp
std::vector<double> inverse_xyz_tilt_joint5(
  const std::vector<double> &seed,
  double target_x,
  double target_y,
  double target_z,
  double target_tool_tilt,
  double target_joint5) const;
```

## 2. 在 urdf_kinematics.cpp 中新增实现

将以下方法放在 robot_arm_node::motion 命名空间内，和其他 UrdfKinematics 方法并列。原文件已有所需的 Eigen/Dense、algorithm、cmath、sstream、stdexcept 头文件。

实现固定 q(4)，仅更新 q(0)～q(3)。它保留数值差分和阻尼最小二乘，并增加减小步长重试，避免接受使任务误差变大的更新。位置与角度采用独立容差；角度在优化代价中按 0.1 m/rad 换算权重，该权重不是位置容差。

```cpp
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
```

这仍是单初值的局部求解器。不能收敛时，不等价于目标一定不可达；可能需要选择不同的肘部构型、增加初值尝试，或使用适合此结构的几何逆解。不要通过无限放宽容差掩盖失败。

## 3. 修改 Action 接口语义和节点调用

建议将 PickPoint.action 的请求字段 target_pitch、target_roll 明确改名为：

```text
float64 target_tool_tilt
float64 target_joint5
```

只改字段名不够，还要重新生成接口、重新编译接口包及依赖它的节点并加载新环境。所有发送目标的客户端也需要同步修改。保留其他现有字段和 Action 分隔结构。

execute_pick() 中读取：

```cpp
const double target_tool_tilt = goal->target_tool_tilt;
const double target_joint5 = goal->target_joint5;
```

三处逆解调用分别改为：

```cpp
auto q_pregrasp = kinematics_->inverse_xyz_tilt_joint5(
  seed, target_x, target_y, target_z + approach_h,
  target_tool_tilt, target_joint5);

ik_stage = "grasp";
auto q_grasp = kinematics_->inverse_xyz_tilt_joint5(
  q_pregrasp, target_x, target_y, target_z,
  target_tool_tilt, target_joint5);

ik_stage = "lift";
auto q_lift = kinematics_->inverse_xyz_tilt_joint5(
  q_grasp, target_x, target_y, target_z + approach_h,
  target_tool_tilt, target_joint5);
```

最终继续发送含五个关节角的向量：

```cpp
send_trajectory(q_lift);
```

只发送 q_lift 的行为没有在本方案中改变。如果需要真正执行预抓取、下降和抬起，需要另行组织多个轨迹段和夹爪动作。

handle_goal() 中原本对 target_pitch、target_roll 的有限值检查也应替换为新字段，并检查工具倾角在 [-π/2, π/2] 内。第五关节限位由此方法检查，也可在接收目标时提前拒绝。

若希望保持第五关节当前角度，可以在获取 seed 后使用 const double target_joint5 = seed.at(4)，此时可不新增目标字段，但同一任务中三次 IK 必须一直使用这一个固定值。

如果要将迭代上限设为 500，节点构造时可显式传入：

```cpp
kinematics_ = std::make_unique<robot_arm_node::motion::UrdfKinematics>(
  message->data, base_link_, tip_link_, 500U, 1e-6);
```

新增方法的位置和倾角容差由方法内两个独立常量控制，不再使用构造函数的单个 tolerance_。原 KDL inverse() 的设置不变。

## 4. 日志应显示新的物理量

在 log_solution 中可以保留原有 RPY 用于观察，但成功与否应检查新的工具倾角，而不是欧拉角 pitch。新增：

```cpp
const auto tool_z = frame.M.UnitZ();
const double tool_tilt = std::atan2(
  tool_z.z(), std::hypot(tool_z.x(), tool_z.y()));
RCLCPP_INFO(get_logger(), "%s tool_tilt=%.6f rad, joint5=%.6f rad",
  stage, tool_tilt, joints.at(4));
```

这些数值来自目标关节角的 FK。轨迹完成后还需以最新 /joint_states 计算 FK，才能核对仿真实际到达状态。当前节点允许每个目标启动独立线程；接入时需避免多个目标同时操作共享求解器及控制器，本文件不实现任务串行管理。

## 5. 新请求示例

这是接口改名并重新编译后的示例，不保证该目标对任意安装、初始构型都可达。倾角 0 表示工具正 Z 轴水平；joint5=0.5 表示第五关节目标为 0.5 rad。

```bash
ros2 action send_goal --feedback /pick_point \
  robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.15, y: 0.0, z: 0.2}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: 0.0, target_joint5: 0.5, duration: {sec: 2, nanosec: 0}, return_home: false}"
```

原节点中的 duration、return_home 尚未实现对应行为，本方案不改变这一点。新旧 pitch 语义不同，不能原样搬用旧请求的 1.0、1.5 等数值。

## 验证范围

完成了基于所给 URDF 关节变换、用 Python 复现同一数值步骤的独立验证：

- 固定 XYZ=(0.15, 0, 0.2)、工具倾角=0，分别指定 joint5=-1、0、0.5、1.5 rad，均成功收敛；这四例位置残差约 0.227 mm、倾角残差约 0.00173°，第五轴严格等于指定值。
- 30 组由 FK 生成的可达目标，以目标关节角附近的扰动作为初值，均满足 1 mm 和 0.01 rad 的容差；这不代表任意远距离初值都能收敛。
- 单独改变第五关节时，TCP 位置和工具倾角保持不变。
- 超出限位的第五轴角、超出定义域的倾角被拒绝；明显超出工作范围的目标未被错误接受。

当前环境没有 ROS 2/KDL C++ 构建依赖，尚未完成该 C++ 实现的实际编译、Gazebo 执行及轨迹检查。此文件是需集成进现有工程的方法和接入说明，不是一个可以独立运行的 ROS 包。
