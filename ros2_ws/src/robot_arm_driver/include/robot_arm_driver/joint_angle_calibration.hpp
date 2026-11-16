#pragma once

#include <cmath>
#include <stdexcept>

namespace robot_arm_driver
{
// The native zero is the user's home; the URDF zero is the model's reference pose.
class JointAngleCalibration
{
  public:
    JointAngleCalibration(
      double native_zero_ticks, double urdf_zero_ticks, double ticks_per_radian, double direction)
    : native_zero_ticks_(native_zero_ticks), urdf_zero_ticks_(urdf_zero_ticks),
      scale_(direction * ticks_per_radian)
    {
      if (!std::isfinite(native_zero_ticks) || !std::isfinite(urdf_zero_ticks) ||
          !std::isfinite(ticks_per_radian) || ticks_per_radian <= 0.0 ||
          !std::isfinite(direction) || std::abs(direction) != 1.0)
      {
        throw std::invalid_argument("joint calibration requires finite zeros, a positive scale, and direction +/-1");
      }
    }

    double native_to_ticks(double position) const
    {
      return native_zero_ticks_ + scale_ * position;
    }

    double urdf_to_ticks(double position) const
    {
      return urdf_zero_ticks_ + scale_ * position;
    }

    double ticks_to_urdf(double ticks) const
    {
      return (ticks - urdf_zero_ticks_) / scale_;
    }

  private:
    double native_zero_ticks_;
    double urdf_zero_ticks_;
    double scale_;
};
}  // namespace robot_arm_driver
