#include "robot_arm_driver/coordinate_bus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <chrono>
#include <optional>
#include <sys/select.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace robot_arm_driver
{
namespace
{
speed_t baud_to_termios(const int baud)
{
  switch (baud)
  {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: throw std::invalid_argument("unsupported coordinate-controller baud rate");
  }
}
}   // namespace

CoordinateBus::CoordinateBus(std::string device, const int baud_rate)
  : device_(std::move(device)), baud_rate_(baud_rate)
{
}

CoordinateBus::~CoordinateBus()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
  }
}

void CoordinateBus::open()
{
  std::scoped_lock lock(mutex_);

  if (fd_ >= 0)
  {
    return;
  }
  const auto baud = baud_to_termios(baud_rate_);

  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0)
  {
    throw std::runtime_error("cannot open " + device_ + ": " + std::strerror(errno));
  }
  termios tty{};

  if (tcgetattr(fd_, &tty) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
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
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
  }
}

void CoordinateBus::send(const std::vector<uint8_t> &frame)
{
  std::scoped_lock lock(mutex_);

  if (fd_ < 0)
  {
    throw std::runtime_error("coordinate bus is not open");
  }
  size_t sent = 0;

  while (sent < frame.size())
  {
    const auto count = ::write(fd_, frame.data() + sent, frame.size() - sent);
    if (count < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    sent += static_cast<size_t>(count);
  }
  tcdrain(fd_);
}

std::optional<std::vector<uint8_t> > CoordinateBus::read_frame(const int timeout_ms)
{
  std::vector<uint8_t> bytes;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline)
  {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    timeval timeout{static_cast<long>(remaining / 1000), static_cast<long>((remaining % 1000) * 1000)};
    const auto ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0)
    {
      continue;
    }
    uint8_t byte{};
    if (::read(fd_, &byte, 1) != 1)
    {
      continue;
    }
    bytes.push_back(byte);
    if (bytes.size() == 1U && bytes[0] != 0xffU)
    {
      bytes.clear();
      continue;
    }
    if (bytes.size() == 2U && bytes[1] != 0xffU)
    {
      bytes.clear();
      continue;
    }
    if (bytes.size() >= 4U)
    {
      const auto expected_size = static_cast<size_t>(bytes[3]) + 4U;
      if (bytes[3] < 2U || expected_size > 260U)
      {
        bytes.clear();
        continue;
      }
      if (bytes.size() == expected_size)
      {
        uint16_t checksum = 0;
        for (size_t i = 2; i + 1 < bytes.size(); ++i)
        {
          checksum += bytes[i];
        }
        if (bytes.back() == static_cast<uint8_t>(~checksum))
        {
          return bytes;
        }
        bytes.clear();
      }
    }
  }
  return std::nullopt;
}

std::optional<std::vector<uint8_t> > CoordinateBus::read_v3_frame(const int timeout_ms)
{
  std::vector<uint8_t> bytes;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline)
  {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    timeval timeout{static_cast<long>(remaining / 1000), static_cast<long>((remaining % 1000) * 1000)};
    const auto ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0)
    {
      continue;
    }
    uint8_t byte{};
    if (::read(fd_, &byte, 1) != 1)
    {
      continue;
    }
    bytes.push_back(byte);
    if (bytes.size() == 1U && bytes[0] != 0x55U)
    {
      bytes.clear();
      continue;
    }
    if (bytes.size() == 2U && bytes[1] != 0x55U)
    {
      bytes.clear();
      continue;
    }
    if (bytes.size() >= 4U)
    {
      const auto expected_size = static_cast<size_t>(bytes[3]) + 3U;
      if (bytes[3] < 3U || expected_size > 260U)
      {
        bytes.clear();
        continue;
      }
      if (bytes.size() == expected_size)
      {
        uint16_t checksum = 0;
        for (size_t i = 2; i + 1 < bytes.size(); ++i)
        {
          checksum += bytes[i];
        }
        if (bytes.back() == static_cast<uint8_t>(255U - (checksum & 0xffU)))
        {
          return bytes;
        }
        bytes.clear();
      }
    }
  }
  return std::nullopt;
}

std::optional<std::vector<uint8_t> > CoordinateBus::request(const std::vector<uint8_t> &frame, const int timeout_ms)
{
  std::scoped_lock lock(mutex_);

  if (fd_ < 0)
  {
    throw std::runtime_error("coordinate bus is not open");
  }
  tcflush(fd_, TCIFLUSH);
  size_t sent = 0;

  while (sent < frame.size())
  {
    const auto count = ::write(fd_, frame.data() + sent, frame.size() - sent);
    if (count < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    sent += static_cast<size_t>(count);
  }
  tcdrain(fd_);
  return read_frame(timeout_ms);
}

std::optional<std::vector<uint8_t> > CoordinateBus::request_v3(
  const std::vector<uint8_t> &frame, const int timeout_ms)
{
  std::scoped_lock lock(mutex_);

  if (fd_ < 0)
  {
    throw std::runtime_error("coordinate bus is not open");
  }
  tcflush(fd_, TCIFLUSH);
  size_t sent = 0;

  while (sent < frame.size())
  {
    const auto count = ::write(fd_, frame.data() + sent, frame.size() - sent);
    if (count < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    sent += static_cast<size_t>(count);
  }
  tcdrain(fd_);
  return read_v3_frame(timeout_ms);
}
}  // namespace robot_arm_driver
