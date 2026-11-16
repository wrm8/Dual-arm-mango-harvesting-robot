#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace robot_sensors
{

struct WitAngles
{
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
};

// WIT standard binary UART: 0x55, type, 8 payload bytes, checksum.
class WitImuParser
{
public:
  std::optional<WitAngles> push(uint8_t byte)
  {
    ++bytes_received_;
    if (size_ == 0 && byte != 0x55) {
      return std::nullopt;
    }
    buffer_[size_++] = byte;
    if (size_ < buffer_.size()) {
      return std::nullopt;
    }
    uint8_t sum = 0;
    for (std::size_t i = 0; i < 10; ++i) {
      sum = static_cast<uint8_t>(sum + buffer_[i]);
    }
    if (sum != buffer_[10]) {
      ++checksum_errors_;
      // Retain a possible header inside a corrupt/truncated packet.
      std::size_t next = 1;
      while (next < buffer_.size() && buffer_[next] != 0x55) {
        ++next;
      }
      size_ = buffer_.size() - next;
      for (std::size_t i = 0; i < size_; ++i) {
        buffer_[i] = buffer_[next + i];
      }
      return std::nullopt;
    }
    size_ = 0;
    ++valid_packets_;
    if (buffer_[1] != 0x53) {
      return std::nullopt;
    }
    ++angle_packets_;
    return WitAngles{decode_angle(2), decode_angle(4), decode_angle(6)};
  }

  // Reconnect drops partial bytes but preserves lifetime diagnostic counters.
  void reset() {size_ = 0;}
  uint64_t bytes_received() const {return bytes_received_;}
  uint64_t valid_packets() const {return valid_packets_;}
  uint64_t angle_packets() const {return angle_packets_;}
  uint64_t checksum_errors() const {return checksum_errors_;}

private:
  double decode_angle(std::size_t offset) const
  {
    const uint32_t raw = static_cast<uint32_t>(buffer_[offset]) |
      (static_cast<uint32_t>(buffer_[offset + 1]) << 8);
    const int32_t value = raw >= 32768U ?
      static_cast<int32_t>(raw) - 65536 : static_cast<int32_t>(raw);
    return static_cast<double>(value) * 180.0 / 32768.0;
  }

  std::array<uint8_t, 11> buffer_{};
  std::size_t size_{0};
  uint64_t bytes_received_{0};
  uint64_t valid_packets_{0};
  uint64_t angle_packets_{0};
  uint64_t checksum_errors_{0};
};

inline double wrap_degrees(double degrees)
{
  double result = std::fmod(degrees + 180.0, 360.0);
  if (result < 0.0) {
    result += 360.0;
  }
  return result - 180.0;
}

class WitYawReference
{
public:
  WitYawReference(bool zero_on_start, double sign, double offset_deg)
  : zero_on_start_(zero_on_start), sign_(sign), offset_deg_(offset_deg) {}

  double apply(double raw_deg)
  {
    if (!initialized_) {
      reference_deg_ = zero_on_start_ ? raw_deg : 0.0;
      initialized_ = true;
    }
    return wrap_degrees(sign_ * (raw_deg - reference_deg_) + offset_deg_);
  }

private:
  bool zero_on_start_;
  double sign_;
  double offset_deg_;
  bool initialized_{false};
  double reference_deg_{0.0};
};

}  // namespace robot_sensors
