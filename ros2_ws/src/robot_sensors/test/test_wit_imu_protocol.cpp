#include "robot_sensors/wit_imu_protocol.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>

using robot_sensors::WitAngles;
using robot_sensors::WitImuParser;
using robot_sensors::WitYawReference;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

bool near(double actual, double expected)
{
  return std::abs(actual - expected) < 1e-9;
}

std::array<uint8_t, 11> packet(int roll, int pitch, int yaw, uint8_t type = 0x53)
{
  std::array<uint8_t, 11> bytes{{0x55, type, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  const std::array<int, 3> angles{{roll, pitch, yaw}};
  for (std::size_t i = 0; i < angles.size(); ++i) {
    const auto value = static_cast<uint16_t>(angles[i]);
    bytes[2 + i * 2] = static_cast<uint8_t>(value & 0xff);
    bytes[3 + i * 2] = static_cast<uint8_t>(value >> 8);
  }
  for (std::size_t i = 0; i < 10; ++i) {
    bytes[10] = static_cast<uint8_t>(bytes[10] + bytes[i]);
  }
  return bytes;
}

std::optional<WitAngles> feed(WitImuParser & parser, const std::array<uint8_t, 11> & bytes)
{
  std::optional<WitAngles> last;
  for (const auto byte : bytes) {
    if (const auto result = parser.push(byte)) {
      last = result;
    }
  }
  return last;
}
}  // namespace

int main()
{
  WitImuParser parser;
  const auto frame = packet(16384, -16384, -32768);
  for (std::size_t i = 0; i < frame.size() - 1; ++i) {
    require(!parser.push(frame[i]), "partial frame must not publish");
  }
  auto angles = parser.push(frame.back());
  require(angles && near(angles->roll_deg, 90.0) && near(angles->pitch_deg, -90.0) &&
    near(angles->yaw_deg, -180.0), "signed little-endian decoding");
  angles = feed(parser, packet(0, 0, 32767));
  require(angles && near(angles->yaw_deg, 32767.0 * 180.0 / 32768.0), "positive limit");
  require(!feed(parser, packet(0, 0, 16384, 0x51)), "acceleration must not publish yaw");
  require(!feed(parser, packet(0, 0, 16384, 0x52)), "gyro must not publish yaw");
  auto corrupt = packet(0, 0, 100);
  corrupt[10] ^= 1;
  require(!feed(parser, corrupt), "checksum rejection");
  for (const auto byte : {0x23, 0x00, 0x77, 0x55, 0x53, 0x01}) {
    require(!parser.push(static_cast<uint8_t>(byte)), "noise must not publish");
  }
  angles = feed(parser, packet(0, 0, 8192));
  require(angles && near(angles->yaw_deg, 45.0), "recover header inside corrupt frame");
  angles = feed(parser, packet(0x55, 0x55, -8192));
  require(angles && near(angles->yaw_deg, -45.0), "header bytes inside valid payload");
  parser.push(0x55);
  parser.push(0x53);
  const auto previous_bytes = parser.bytes_received();
  parser.reset();
  require(parser.bytes_received() == previous_bytes, "reset preserves lifetime counters");
  angles = feed(parser, packet(0, 0, 0));
  require(angles && near(angles->yaw_deg, 0.0), "reset drops stale partial bytes");
  require(parser.angle_packets() == 5 && parser.valid_packets() == 7 &&
    parser.checksum_errors() >= 2, "diagnostic counters");

  WitYawReference absolute(false, 1.0, 0.0);
  require(near(absolute.apply(90.0), 90.0), "default retains raw heading");
  WitYawReference relative(true, 1.0, 0.0);
  require(near(relative.apply(179.0), 0.0), "first-frame reference");
  require(near(relative.apply(-179.0), 2.0), "wrap crossing");
  require(near(relative.apply(170.0), -9.0), "reference retained across later frames");
  WitYawReference adjusted(true, -1.0, 5.0);
  require(near(adjusted.apply(10.0), 5.0), "offset after relative reference");
  require(near(adjusted.apply(30.0), -15.0), "sign and offset");
  require(near(robot_sensors::wrap_degrees(180.0), -180.0) &&
    near(robot_sensors::wrap_degrees(-540.0), -180.0), "normalized interval");
  std::cout << "WIT parser and yaw reference tests passed\n";
}
