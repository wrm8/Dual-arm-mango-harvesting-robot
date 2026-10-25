#pragma once

#include <cstdint>
#include <vector>

namespace robot_arm_driver
{
constexpr uint8_t kCoordinateControllerId = 0xff;
constexpr uint8_t kServoBroadcastId = 0xfe;
constexpr uint8_t kCoordinateSetCommand = 8;
constexpr uint8_t kServoWriteCommand = 3;
constexpr uint8_t kServoRegWriteCommand = 4;
constexpr uint8_t kServoActionCommand = 5;
constexpr uint8_t kServoSyncWriteCommand = 131;

struct CartesianCommand
{
  int16_t x_mm{};
  int16_t y_mm{};
  int16_t z_mm{};
  int16_t pitch_deci_degrees{};
  int16_t roll_degrees{};
  int16_t claw_position{};
  uint16_t duration_ms{};
};

struct HxPositionTarget
{
  uint8_t id{};
  int16_t position{};
  uint16_t speed{};
  uint8_t acceleration{};
};

std::vector<uint8_t> encode_at32_frame(uint8_t id, uint8_t command,
                                       const std::vector<uint8_t> &payload);
std::vector<uint8_t> encode_coordinate_command(const CartesianCommand &command);
// HX-30HM direct-servo commands.  Position is signed (two's complement) and
// the bus runs at 1 Mbps by default.
std::vector<uint8_t> encode_hx_torque_command(uint8_t servo_id, bool enabled);
std::vector<uint8_t> encode_hx_mode_command(uint8_t servo_id, uint8_t mode);
std::vector<uint8_t> encode_hx_position_command(
  uint8_t servo_id, int16_t position, uint16_t speed, uint8_t acceleration);
std::vector<uint8_t> encode_hx_reg_position_command(
  uint8_t servo_id, int16_t position, uint16_t speed, uint8_t acceleration);
std::vector<uint8_t> encode_hx_action_command();
std::vector<uint8_t> encode_hx_sync_position_command(const std::vector<HxPositionTarget> &targets);
std::vector<uint8_t> encode_gripper_command(uint8_t servo_id, uint16_t position, uint16_t speed);
}  // namespace robot_arm_driver
