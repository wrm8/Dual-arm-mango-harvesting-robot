#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace robot_arm_driver
{
struct BusConfig
{
  std::string device{"/dev/ttyACM1"};
  int baud_rate{115200};
  bool use_direction_gpio{false};
  int tx_enable_gpio{-1};
  int rx_enable_gpio{-1};
};

class ServoBus
{
  public:
    explicit ServoBus(BusConfig config);
    ~ServoBus();
    ServoBus(const ServoBus &) = delete;
    ServoBus &operator=(const ServoBus &) = delete;

    void open();
    void close();
    bool is_open() const;
    void move(uint8_t id, uint16_t position, uint16_t duration_ms);
    void stop(uint8_t id);
    std::optional<uint16_t> read_position(uint8_t id, int timeout_ms = 50);

  private:
    void set_write_direction();
    void set_read_direction();
    void write_all(const std::vector<uint8_t> &data);
    std::vector<uint8_t> read_one_frame(int timeout_ms);
    void setup_gpio(int gpio);
    void write_gpio(int gpio, bool value);

    BusConfig config_;
    int fd_{-1};
    mutable std::mutex mutex_;
};
}  // namespace robot_arm_driver
