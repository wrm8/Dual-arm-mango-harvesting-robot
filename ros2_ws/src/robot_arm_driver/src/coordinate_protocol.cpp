#include "robot_arm_driver/coordinate_protocol.hpp"

#include <cstddef>
#include <stdexcept>

namespace robot_arm_driver
{
namespace
{
void append_i16_le(std::vector<uint8_t> &bytes, const int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);

  bytes.push_back(static_cast<uint8_t>(raw & 0xffU));
  bytes.push_back(static_cast<uint8_t>(raw >> 8U));
}
void append_u16_le(std::vector<uint8_t> &bytes, const uint16_t value)
{
  bytes.push_back(static_cast<uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<uint8_t>(value >> 8U));
}

uint16_t hx_signed_15bit(const int16_t value)
{
  return value < 0 ? static_cast<uint16_t>(-value) | 0x8000U : static_cast<uint16_t>(value);
}
}   // namespace

std::vector<uint8_t> encode_at32_frame(
  const uint8_t id, const uint8_t command, const std::vector<uint8_t> &payload)
{
  std::vector<uint8_t> frame{0xff, 0xff, id, static_cast<uint8_t>(payload.size() + 2U), command};

  frame.insert(frame.end(), payload.begin(), payload.end());
  uint16_t sum = 0;

  for (std::size_t i = 2; i < frame.size(); ++i)
  {
    sum += frame[i];
  }
  frame.push_back(static_cast<uint8_t>(~sum));
  return frame;
}

std::vector<uint8_t> encode_v3_frame(
  const uint8_t id, const uint8_t command, const std::vector<uint8_t> &parameters)
{
  const auto length = static_cast<uint8_t>(parameters.size() + 3U);
  uint16_t sum = static_cast<uint16_t>(id) + length + command;
  std::vector<uint8_t> frame{0x55, 0x55, id, length, command};

  for (const auto value : parameters)
  {
    frame.push_back(value);
    sum += value;
  }
  frame.push_back(static_cast<uint8_t>(255U - (sum & 0xffU)));
  return frame;
}

std::vector<uint8_t> encode_v3_position_command(
  const uint8_t id, const uint16_t position, const uint16_t duration_ms)
{
  return encode_v3_frame(
    id, kV3MoveTimeWriteCommand,
    {static_cast<uint8_t>(position & 0xffU), static_cast<uint8_t>(position >> 8U),
     static_cast<uint8_t>(duration_ms & 0xffU), static_cast<uint8_t>(duration_ms >> 8U)});
}

std::vector<uint8_t> encode_v3_stop_command(const uint8_t id)
{
  return encode_v3_frame(id, kV3MoveStopCommand, {});
}

std::vector<uint8_t> encode_coordinate_command(const CartesianCommand &command)
{
  std::vector<uint8_t> payload;

  payload.reserve(14);
  // Exact legacy field order: pitch, x, y, z, roll, claw, duration.
  append_i16_le(payload, command.pitch_deci_degrees);
  append_i16_le(payload, command.x_mm);
  append_i16_le(payload, command.y_mm);
  append_i16_le(payload, command.z_mm);
  append_i16_le(payload, command.roll_degrees);
  append_i16_le(payload, command.claw_position);
  append_u16_le(payload, command.duration_ms);
  return encode_at32_frame(kCoordinateControllerId, kCoordinateSetCommand, payload);
}

int16_t decode_hx_position(const uint8_t low, const uint8_t high)
{
  const auto raw = static_cast<uint16_t>(low) |
                   (static_cast<uint16_t>(high) << 8U);
  const auto magnitude = static_cast<int16_t>(raw & 0x7fffU);
  return (raw & 0x8000U) != 0U ? static_cast<int16_t>(-magnitude) : magnitude;
}

int16_t decode_v3_position(const uint8_t low, const uint8_t high)
{
  const auto raw = static_cast<uint16_t>(low) |
                   (static_cast<uint16_t>(high) << 8U);
  return static_cast<int16_t>(raw);
}

std::vector<uint8_t> encode_gripper_command(
  const uint8_t servo_id, const uint16_t position, const uint16_t speed)
{
  return encode_hx_position_command(servo_id, static_cast<int16_t>(position), speed, 0);
}

std::vector<uint8_t> encode_hx_torque_command(const uint8_t servo_id, const bool enabled)
{
  return encode_at32_frame(servo_id, kServoWriteCommand, {40, enabled ? uint8_t{1} : uint8_t{0}});
}

std::vector<uint8_t> encode_hx_mode_command(const uint8_t servo_id, const uint8_t mode)
{
  return encode_at32_frame(servo_id, kServoWriteCommand, {33, mode});
}

std::vector<uint8_t> encode_hx_position_command(
  const uint8_t servo_id, const int16_t position, const uint16_t speed,
  const uint8_t acceleration)
{
  // Copied from HX_30HM::write_pos_ex: start at register 41, then
  // ACC, target position, PWM=0, and closed-loop goal speed.
  const auto raw_position = hx_signed_15bit(position);
  std::vector<uint8_t> payload{41, acceleration,
                               static_cast<uint8_t>(raw_position & 0xffU), static_cast<uint8_t>(raw_position >> 8U),
                               0, 0, static_cast<uint8_t>(speed & 0xffU), static_cast<uint8_t>(speed >> 8U)};

  return encode_at32_frame(servo_id, kServoWriteCommand, payload);
}

std::vector<uint8_t> encode_hx_reg_position_command(
  const uint8_t servo_id, const int16_t position, const uint16_t speed,
  const uint8_t acceleration)
{
  const auto raw_position = hx_signed_15bit(position);
  std::vector<uint8_t> payload{41, acceleration,
                               static_cast<uint8_t>(raw_position & 0xffU), static_cast<uint8_t>(raw_position >> 8U),
                               0, 0, static_cast<uint8_t>(speed & 0xffU), static_cast<uint8_t>(speed >> 8U)};

  return encode_at32_frame(servo_id, kServoRegWriteCommand, payload);
}

std::vector<uint8_t> encode_hx_action_command()
{
  return encode_at32_frame(kServoBroadcastId, kServoActionCommand, {});
}

std::vector<uint8_t> encode_hx_sync_position_command(const std::vector<HxPositionTarget> &targets)
{
  if (targets.empty() || targets.size() > 30U)
  {
    throw std::invalid_argument("HX SYNC_WRITE needs 1 to 30 targets");
  }
  std::vector<uint8_t> payload{41, 7};

  payload.reserve(2U + targets.size() * 8U);
  for (const auto &target : targets)
  {
    const auto raw_position = hx_signed_15bit(target.position);
    payload.insert(payload.end(), {target.id, target.acceleration,
                                   static_cast<uint8_t>(raw_position & 0xffU), static_cast<uint8_t>(raw_position >> 8U),
                                   0, 0, static_cast<uint8_t>(target.speed & 0xffU), static_cast<uint8_t>(target.speed >> 8U)});
  }
  return encode_at32_frame(kServoBroadcastId, kServoSyncWriteCommand, payload);
}
}  // namespace robot_arm_driver
