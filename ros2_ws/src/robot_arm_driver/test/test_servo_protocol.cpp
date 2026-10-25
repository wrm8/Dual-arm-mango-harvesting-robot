#include "robot_arm_driver/servo_protocol.hpp"

#include <gtest/gtest.h>

TEST(ServoProtocol, EncodesMoveTimeFrame)
{
  const auto frame = robot_arm_driver::encode_frame(1, robot_arm_driver::kMoveTimeWrite,
                                                    robot_arm_driver::move_time_payload(500, 1000));

  EXPECT_EQ(frame, (std::vector<uint8_t>{0x55, 0x55, 1, 7, 1, 0xf4, 1, 0xe8, 3, 0x16}));
}

TEST(ServoProtocol, DecodesValidFrameAndRejectsBadChecksum)
{
  const std::vector<uint8_t> valid{0x55, 0x55, 1, 5, 28, 0xf4, 1, 0xe8};
  const auto decoded = robot_arm_driver::decode_frame(valid);

  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->id, 1);
  EXPECT_EQ(decoded->command, 28);
  EXPECT_EQ(decoded->parameters, (std::vector<uint8_t>{0xf4, 1}));
  auto invalid = valid;

  invalid.back() = 0;
  EXPECT_FALSE(robot_arm_driver::decode_frame(invalid).has_value());
}
