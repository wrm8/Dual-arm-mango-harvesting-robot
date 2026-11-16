#include "robot_arm_driver/joint_angle_calibration.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using robot_arm_driver::JointAngleCalibration;

TEST(JointAngleCalibration, MeasuredModelZeroIsZeroAndNativeHomeIsPreserved)
{
  const double native[] = {1908, 2012, 1640, 1491, 518, 760};
  const double model[] = {1887, 2384, 3602, 1889, 518, 760};
  for (size_t i = 0; i < 6; ++i)
  {
    const double scale = i < 4 ? 651.8986 : 238.7324;
    const JointAngleCalibration calibration(native[i], model[i], scale, i == 5 ? -1 : 1);
    EXPECT_DOUBLE_EQ(calibration.ticks_to_urdf(model[i]), 0.0);
    EXPECT_DOUBLE_EQ(calibration.urdf_to_ticks(0.0), model[i]);
    EXPECT_DOUBLE_EQ(calibration.native_to_ticks(0.0), native[i]);
    EXPECT_NEAR(calibration.urdf_to_ticks(calibration.ticks_to_urdf(native[i])), native[i], 1e-10);
  }
}

TEST(JointAngleCalibration, MeasuredJoint3RangeExcludesTheOldPositiveElbowSolution)
{
  const JointAngleCalibration calibration(1640, 3602, 651.8986, 1);
  EXPECT_NEAR(calibration.ticks_to_urdf(1640), -3.00967, 1e-5);
  EXPECT_NEAR(calibration.ticks_to_urdf(3650), 0.07363, 1e-5);
  EXPECT_GT(calibration.urdf_to_ticks(1.55), 3650);
}

TEST(JointAngleCalibration, RoundedCommandsRoundTripWithinHalfATickForEitherDirection)
{
  for (const double direction : {-1.0, 1.0})
  {
    const JointAngleCalibration calibration(2012, 2384, 651.8986, direction);
    for (const double position : {-1.2, -0.3, 0.0, 0.4, 1.0})
    {
      const auto ticks = std::lround(calibration.urdf_to_ticks(position));
      EXPECT_NEAR(calibration.ticks_to_urdf(ticks), position, 0.5 / 651.8986);
    }
  }
}

TEST(JointAngleCalibration, RejectsNonFiniteAndInvalidCalibration)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(JointAngleCalibration(nan, 2384, 651.8986, 1), std::invalid_argument);
  EXPECT_THROW(JointAngleCalibration(2012, nan, 651.8986, 1), std::invalid_argument);
  EXPECT_THROW(JointAngleCalibration(2012, 2384, nan, 1), std::invalid_argument);
  EXPECT_THROW(JointAngleCalibration(2012, 2384, 0, 1), std::invalid_argument);
  EXPECT_THROW(JointAngleCalibration(2012, 2384, 651.8986, nan), std::invalid_argument);
  EXPECT_THROW(JointAngleCalibration(2012, 2384, 651.8986, 0), std::invalid_argument);
}
