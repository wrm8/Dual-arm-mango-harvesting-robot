#include "robot_arm_driver/servo_protocol.hpp"

namespace robot_arm_driver
{
std::vector<uint8_t> encode_frame(
  uint8_t id, uint8_t command, const std::vector<uint8_t> &parameters)
{
  const auto length = static_cast<uint8_t>(parameters.size() + 3U);
  uint16_t sum = id + length + command;
  std::vector<uint8_t> frame{kHeader, kHeader, id, length, command};

  for (const auto value : parameters)
  {
    frame.push_back(value);
    sum += value;
  }
  frame.push_back(static_cast<uint8_t>(255U - (sum & 0xffU)));
  return frame;
}

std::optional<ServoFrame> decode_frame(const std::vector<uint8_t> &bytes)
{
  if (bytes.size() < 6U || bytes[0] != kHeader || bytes[1] != kHeader)
  {
    return std::nullopt;
  }
  const auto length = bytes[3];

  if (length < 3U || bytes.size() != static_cast<size_t>(length) + 3U)
  {
    return std::nullopt;
  }
  uint16_t sum = 0;

  for (size_t i = 2; i + 1 < bytes.size(); ++i)
  {
    sum += bytes[i];
  }
  if (bytes.back() != static_cast<uint8_t>(255U - (sum & 0xffU)))
  {
    return std::nullopt;
  }
  ServoFrame frame;

  frame.id = bytes[2];
  frame.command = bytes[4];
  frame.parameters.assign(bytes.begin() + 5, bytes.end() - 1);
  return frame;
}

std::vector<uint8_t> move_time_payload(uint16_t position, uint16_t duration_ms)
{
  return {
    static_cast<uint8_t>(position & 0xffU), static_cast<uint8_t>(position >> 8U),
    static_cast<uint8_t>(duration_ms & 0xffU), static_cast<uint8_t>(duration_ms >> 8U)};
}
}  // namespace robot_arm_driver
