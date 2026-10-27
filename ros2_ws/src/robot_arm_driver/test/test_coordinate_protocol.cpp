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

TEST(CoordinateProtocol, DecodesHxSignedMagnitudePosition)
{
  EXPECT_EQ(robot_arm_driver::decode_hx_position(100, 0), 100);
  EXPECT_EQ(robot_arm_driver::decode_hx_position(0xb8, 0x8b), -3000);
  EXPECT_EQ(robot_arm_driver::decode_hx_position(0xff, 0x7f), 32767);
  EXPECT_EQ(robot_arm_driver::decode_hx_position(0xff, 0xff), -32767);
}

TEST(CoordinateProtocol, EncodesDocumentedPresentPositionRead)
{
  EXPECT_EQ(
    robot_arm_driver::encode_at32_frame(
      1, robot_arm_driver::kHxRead,
      {robot_arm_driver::kPresentPositionL, robot_arm_driver::kPresentPositionSize}),
    (std::vector<uint8_t>{0xff, 0xff, 1, 4, 2, 0x38, 2, 0xbe}));
}

TEST(CoordinateProtocol, EncodesV3PositionRead)
{
  EXPECT_EQ(
    robot_arm_driver::encode_v3_frame(5, robot_arm_driver::kV3PositionReadCommand, {}),
    (std::vector<uint8_t>{0x55, 0x55, 5, 3, 28, 0xdb}));
  EXPECT_EQ(
    robot_arm_driver::encode_v3_position_command(5, 500, 1000),
    (std::vector<uint8_t>{0x55, 0x55, 5, 7, 1, 0xf4, 0x01, 0xe8, 0x03, 0x12}));
  EXPECT_EQ(robot_arm_driver::encode_v3_stop_command(5),
            (std::vector<uint8_t>{0x55, 0x55, 5, 3, 12, 0xeb}));
  EXPECT_EQ(robot_arm_driver::decode_v3_position(0xe8, 0x03), 1000);
  EXPECT_EQ(robot_arm_driver::decode_v3_position(0xf4, 0x01), 500);
}
