#include "robot_arm_driver/coordinate_protocol.hpp"

#include <gtest/gtest.h>

TEST(CoordinateProtocol, MatchesLegacyCoordinateFrame)
{
  robot_arm_driver::CartesianCommand command;

  command.x_mm = 200;
  command.y_mm = 0;
  command.z_mm = 200;
  command.pitch_deci_degrees = -900;
  command.duration_ms = 1500;
  const auto frame = robot_arm_driver::encode_coordinate_command(command);

  EXPECT_EQ(frame, (std::vector<uint8_t>{
    0xff, 0xff, 0xff, 16, 8, 0x7c, 0xfc, 0xc8, 0x00, 0x00, 0x00,
    0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0xff}));
}

TEST(CoordinateProtocol, MatchesLegacyGripperWrite)
{
  const auto frame = robot_arm_driver::encode_gripper_command(6, 1086, 2500);

  EXPECT_EQ(frame, (std::vector<uint8_t>{
    0xff, 0xff, 6, 10, 3, 41, 0, 0x3e, 0x04, 0, 0, 0xc4, 0x09, 0xb4}));
}

TEST(CoordinateProtocol, EncodesSignedHxPositionAndTorque)
{
  EXPECT_EQ(robot_arm_driver::encode_hx_torque_command(1, true), (std::vector<uint8_t>{0xff, 0xff, 1, 4, 3, 40, 1, 206}));
  EXPECT_EQ(robot_arm_driver::encode_hx_mode_command(1, 0), (std::vector<uint8_t>{0xff, 0xff, 1, 4, 3, 33, 0, 214}));
  EXPECT_EQ(robot_arm_driver::encode_hx_position_command(1, -3000, 100, 0), (std::vector<uint8_t>{
    0xff, 0xff, 1, 10, 3, 41, 0, 0xb8, 0x8b, 0, 0, 100, 0, 33}));
  EXPECT_EQ(robot_arm_driver::encode_hx_reg_position_command(1, 100, 100, 0), (std::vector<uint8_t>{
    0xff, 0xff, 1, 10, 4, 41, 0, 100, 0, 0, 0, 100, 0, 255}));
  EXPECT_EQ(robot_arm_driver::encode_hx_action_command(), (std::vector<uint8_t>{0xff, 0xff, 0xfe, 2, 5, 250}));
  EXPECT_EQ(robot_arm_driver::encode_hx_sync_position_command({{2, 100, 100, 0}, {3, 200, 100, 0}}),
            (std::vector<uint8_t>{0xff, 0xff, 0xfe, 20, 131, 41, 7, 2, 0, 100, 0, 0, 0, 100, 0, 3, 0, 200, 0, 0, 0, 100, 0, 65}));
}
