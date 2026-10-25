#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace robot_arm_driver
{
constexpr uint8_t kHeader = 0x55;
constexpr uint8_t kBroadcastId = 0xfe;
constexpr uint8_t kMoveTimeWrite = 1;
constexpr uint8_t kMoveStop = 12;
constexpr uint8_t kPositionRead = 28;

struct ServoFrame
{
  uint8_t id{};
  uint8_t command{};
  std::vector<uint8_t> parameters;
};

std::vector<uint8_t> encode_frame(uint8_t id, uint8_t command,
                                  const std::vector<uint8_t> &parameters);
std::optional<ServoFrame> decode_frame(const std::vector<uint8_t> &bytes);
std::vector<uint8_t> move_time_payload(uint16_t position, uint16_t duration_ms);
}  // namespace robot_arm_driver
