#include "robot_arm_driver/servo_bus.hpp"

#include "robot_arm_driver/servo_protocol.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace robot_arm_driver
{
namespace
{
speed_t baud_to_termios(const int baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("unsupported baud rate");
  }
}

void write_text_file(const std::string &path, const std::string &value)
{
  std::ofstream file(path);

  if (!file)
  {
    throw std::runtime_error("cannot open " + path + "; check GPIO permissions and mapping");
  }
  file << value;
  if (!file)
  {
    throw std::runtime_error("cannot write " + path);
  }
}
}   // namespace

ServoBus::ServoBus(BusConfig config) : config_(std::move(config))
{
}
ServoBus::~ServoBus()
{
  close();
}

void ServoBus::open()
{
  std::scoped_lock lock(mutex_);
  const auto cleanup_fd = [this]() {
                            if (fd_ >= 0)
                            {
                              ::close(fd_);
                              fd_ = -1;
                            }
                          };

  if (fd_ >= 0)
  {
    return;
  }
  const auto baud = baud_to_termios(config_.baud_rate);

  fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0)
  {
    throw std::runtime_error("cannot open serial device " + config_.device + ": " + std::strerror(errno));
  }
  termios tty{};

  if (tcgetattr(fd_, &tty) != 0)
  {
    const auto message = std::strerror(errno);
    cleanup_fd();
    throw std::runtime_error("tcgetattr failed: " + std::string(message));
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    const auto message = std::strerror(errno);
    cleanup_fd();
    throw std::runtime_error("tcsetattr failed: " + std::string(message));
  }
  if (config_.use_direction_gpio)
  {
    if (config_.tx_enable_gpio < 0 || config_.rx_enable_gpio < 0)
    {
      cleanup_fd();
      throw std::invalid_argument("direction GPIO is enabled but GPIO numbers are not configured");
    }
    setup_gpio(config_.tx_enable_gpio);
    setup_gpio(config_.rx_enable_gpio);
    set_write_direction();
  }
}

void ServoBus::close()
{
  std::scoped_lock lock(mutex_);

  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool ServoBus::is_open() const
{
  return fd_ >= 0;
}

void ServoBus::setup_gpio(const int gpio)
{
  const std::string root = "/sys/class/gpio/gpio" + std::to_string(gpio);

  if (::access(root.c_str(), F_OK) != 0)
  {
    write_text_file("/sys/class/gpio/export", std::to_string(gpio));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  write_text_file(root + "/direction", "out");
}

void ServoBus::write_gpio(const int gpio, const bool value)
{
  write_text_file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value", value ? "1" : "0");
}

void ServoBus::set_write_direction()
{
  if (config_.use_direction_gpio)
  {
    write_gpio(config_.tx_enable_gpio, true);
    write_gpio(config_.rx_enable_gpio, false);
  }
}

void ServoBus::set_read_direction()
{
  if (config_.use_direction_gpio)
  {
    write_gpio(config_.tx_enable_gpio, false);
    write_gpio(config_.rx_enable_gpio, true);
  }
}

void ServoBus::write_all(const std::vector<uint8_t> &data)
{
  size_t written = 0;

  while (written < data.size())
  {
    const auto count = ::write(fd_, data.data() + written, data.size() - written);
    if (count < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    written += static_cast<size_t>(count);
  }
  tcdrain(fd_);
}

void ServoBus::move(const uint8_t id, const uint16_t position, const uint16_t duration_ms)
{
  std::scoped_lock lock(mutex_);

  if (fd_ < 0)
  {
    throw std::runtime_error("serial bus is not open");
  }
  set_write_direction();
  write_all(encode_frame(id, kMoveTimeWrite, move_time_payload(position, duration_ms)));
}

void ServoBus::stop(const uint8_t id)
{
  std::scoped_lock lock(mutex_);

  if (fd_ < 0)
  {
    return;
  }
  set_write_direction();
  write_all(encode_frame(id, kMoveStop, {}));
}

std::vector<uint8_t> ServoBus::read_one_frame(const int timeout_ms)
{
  std::vector<uint8_t> result;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline)
  {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    timeval tv{static_cast<long>(remaining / 1000), static_cast<long>((remaining % 1000) * 1000)};
    const auto ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ready <= 0)
    {
      continue;
    }
    uint8_t byte{};
    if (::read(fd_, &byte, 1) != 1)
    {
      continue;
    }
    result.push_back(byte);
    if (result.size() >= 4U && result[0] == kHeader && result[1] == kHeader &&
        result.size() == static_cast<size_t>(result[3]) + 3U)
    {
      return result;
    }
    if (result.size() > 260U || (result.size() == 2U && result[1] != kHeader))
    {
      result.clear();
    }
  }
  return {};
}

std::optional<uint16_t> ServoBus::read_position(const uint8_t id, const int timeout_ms)
{
  std::scoped_lock lock(mutex_);

  if (fd_ < 0)
  {
    throw std::runtime_error("serial bus is not open");
  }
  tcflush(fd_, TCIFLUSH);
  set_write_direction();
  write_all(encode_frame(id, kPositionRead, {}));
  std::this_thread::sleep_for(std::chrono::microseconds(340));
  set_read_direction();
  const auto frame = decode_frame(read_one_frame(timeout_ms));

  set_write_direction();
  if (!frame || frame->id != id || frame->command != kPositionRead || frame->parameters.size() < 2U)
  {
    return std::nullopt;
  }
  return static_cast<uint16_t>(frame->parameters[0] | (frame->parameters[1] << 8U));
}
}  // namespace robot_arm_driver
