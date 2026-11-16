#include "robot_arm_driver/joint_stationary_monitor.hpp"
#include <gtest/gtest.h>

using robot_arm_driver::JointStationaryMonitor;
using namespace std::chrono_literals;

TEST(JointStationaryMonitor, RequiresTimeAndAtLeastThreeFreshSamples)
{
  JointStationaryMonitor monitor(3, 500ms);
  const auto t = JointStationaryMonitor::Clock::time_point{};
  EXPECT_FALSE(monitor.update({100, 200}, true, t));
  EXPECT_FALSE(monitor.update({101, 199}, true, t + 600ms));
  EXPECT_TRUE(monitor.update({100, 201}, true, t + 700ms));
}

TEST(JointStationaryMonitor, AnyMovingJointRestartsTheWholeWindow)
{
  JointStationaryMonitor monitor(3, 500ms);
  const auto t = JointStationaryMonitor::Clock::time_point{};
  EXPECT_FALSE(monitor.update({100, 200}, true, t));
  EXPECT_FALSE(monitor.update({100, 204}, true, t + 400ms));
  EXPECT_FALSE(monitor.update({100, 204}, true, t + 600ms));
  EXPECT_TRUE(monitor.update({100, 204}, true, t + 900ms));
}

TEST(JointStationaryMonitor, SmallIncrementsDoNotHideSlowDrift)
{
  JointStationaryMonitor monitor(3, 500ms);
  const auto t = JointStationaryMonitor::Clock::time_point{};
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_FALSE(monitor.update({static_cast<int16_t>(100 + i)}, true, t + i * 100ms));
  }
}

TEST(JointStationaryMonitor, LostFeedbackDiscardsEarlierStability)
{
  JointStationaryMonitor monitor(3, 500ms);
  const auto t = JointStationaryMonitor::Clock::time_point{};
  EXPECT_FALSE(monitor.update({100}, true, t));
  EXPECT_FALSE(monitor.update({100}, true, t + 300ms));
  EXPECT_FALSE(monitor.update({100}, false, t + 400ms));
  EXPECT_FALSE(monitor.update({100}, true, t + 600ms));
  EXPECT_FALSE(monitor.update({100}, true, t + 900ms));
  EXPECT_TRUE(monitor.update({100}, true, t + 1100ms));
}
