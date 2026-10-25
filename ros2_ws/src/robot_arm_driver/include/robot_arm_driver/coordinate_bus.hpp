#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace robot_arm_driver
{
class CoordinateBus
{
  public:
    CoordinateBus(std::string device, int baud_rate);
    ~CoordinateBus();
    CoordinateBus(const CoordinateBus &) = delete;
    CoordinateBus &operator=(const CoordinateBus &) = delete;

    void open();
    void send(const std::vector<uint8_t> &frame);
    std::optional<std::vector<uint8_t> > request(const std::vector<uint8_t> &frame, int timeout_ms = 50);

  private:
    std::optional<std::vector<uint8_t> > read_frame(int timeout_ms);
    std::string device_;
    int baud_rate_;
    int fd_{-1};
    std::mutex mutex_;
};
}  // namespace robot_arm_driver
