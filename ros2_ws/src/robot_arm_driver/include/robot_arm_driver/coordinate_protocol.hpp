#pragma once

#include <cstdint>
#include <vector>

namespace robot_arm_driver
{
constexpr uint8_t kCoordinateControllerId = 0xff;
constexpr uint8_t kServoBroadcastId = 0xfe;
constexpr uint8_t kHxRead = 2;
constexpr uint8_t kCoordinateSetCommand = 8;
constexpr uint8_t kServoWriteCommand = 3;
constexpr uint8_t kServoRegWriteCommand = 4;
constexpr uint8_t kServoActionCommand = 5;
constexpr uint8_t kServoSyncWriteCommand = 131;
constexpr uint8_t kPresentPositionL = 56;
constexpr uint8_t kPresentPositionSize = 2;
constexpr uint8_t kV3PositionReadCommand = 28;
constexpr uint8_t kV3MoveTimeWriteCommand = 1;
constexpr uint8_t kV3MoveStopCommand = 12;

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
std::vector<uint8_t> encode_v3_frame(uint8_t id, uint8_t command,
                                     const std::vector<uint8_t> &parameters);
std::vector<uint8_t> encode_v3_position_command(uint8_t id, uint16_t position,
                                                uint16_t duration_ms);
std::vector<uint8_t> encode_v3_stop_command(uint8_t id);
std::vector<uint8_t> encode_coordinate_command(const CartesianCommand &command);
int16_t decode_hx_position(uint8_t low, uint8_t high);
int16_t decode_v3_position(uint8_t low, uint8_t high);
// HX-HM direct-servo commands. Position uses the sign-bit plus magnitude
// encoding used by the original HX firmware; the bus rate is configurable.
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
