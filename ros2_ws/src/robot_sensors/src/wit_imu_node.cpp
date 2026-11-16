#include "robot_sensors/wit_imu_protocol.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace robot_sensors
{
namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
using SteadyClock = std::chrono::steady_clock;

speed_t serial_speed(int baud)
{
  switch (baud) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw std::invalid_argument("unsupported baud_rate");
  }
}
}  // namespace

class WitImuNode : public rclcpp::Node
{
public:
  WitImuNode() : Node("wit_imu_node")
  {
    serial_device_ = parameter("serial_device", std::string("/dev/ttyUSB0"));
    baud_rate_ = parameter("baud_rate", 9600);
    frame_id_ = parameter("frame_id", std::string("wit_imu_link"));
    const double poll_hz = parameter("poll_rate_hz", 200.0);
    stale_timeout_s_ = parameter("stale_timeout_s", 0.5);
    reconnect_interval_s_ = parameter("reconnect_interval_s", 1.0);
    const double yaw_sign = parameter("yaw_sign", 1.0);
    const double yaw_offset = parameter("yaw_offset_deg", 0.0);
    const bool zero_on_start = parameter("zero_yaw_on_start", false);
    if (serial_device_.empty() || frame_id_.empty() ||
      !std::isfinite(poll_hz) || poll_hz <= 0.0 || poll_hz > 1000.0 ||
      !std::isfinite(stale_timeout_s_) || stale_timeout_s_ <= 0.0 ||
      !std::isfinite(reconnect_interval_s_) || reconnect_interval_s_ <= 0.0 ||
      !std::isfinite(yaw_offset) || (yaw_sign != 1.0 && yaw_sign != -1.0))
    {
      throw std::invalid_argument("invalid serial/frame/timing/yaw parameters");
    }
    speed_ = serial_speed(baud_rate_);
    yaw_reference_ = std::make_unique<WitYawReference>(zero_on_start, yaw_sign, yaw_offset);
    const auto qos = rclcpp::SensorDataQoS();
    raw_pub_ = create_publisher<std_msgs::msg::Float64>("/imu/wit/yaw_raw_deg", qos);
    deg_pub_ = create_publisher<std_msgs::msg::Float64>("/imu/wit/yaw_deg", qos);
    rad_pub_ = create_publisher<std_msgs::msg::Float64>("/imu/wit/yaw_rad", qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/wit/data", qos);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / poll_hz));
    timer_ = create_wall_timer(period, [this]() {poll_serial();});
    RCLCPP_INFO(get_logger(),
      "WIT IMU reader: %s, %d baud, 8N1; parameters are startup-only; no sensor writes",
      serial_device_.c_str(), baud_rate_);
  }

  ~WitImuNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Read at startup; restart the node to change this parameter.";
    return declare_parameter<T>(name, default_value, descriptor);
  }

  void disconnect(const std::string & reason)
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    parser_.reset();
    next_attempt_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(reconnect_interval_s_));
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "WIT serial unavailable (%s): %s; retrying without publishing stale yaw",
      serial_device_.c_str(), reason.c_str());
  }

  bool connect_serial()
  {
    fd_ = ::open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      disconnect(std::string("open: ") + std::strerror(errno));
      return false;
    }
    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) {
      disconnect(std::string("tcgetattr: ") + std::strerror(errno));
      return false;
    }
    ::cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, speed_) != 0 ||
      ::cfsetospeed(&settings, speed_) != 0 || ::tcsetattr(fd_, TCSANOW, &settings) != 0)
    {
      disconnect(std::string("configure: ") + std::strerror(errno));
      return false;
    }
    ::tcflush(fd_, TCIFLUSH);
    parser_.reset();
    last_valid_ = SteadyClock::now();
    RCLCPP_INFO(get_logger(), "WIT serial connected: %s", serial_device_.c_str());
    return true;
  }

  void publish_angles(const WitAngles & angles)
  {
    last_valid_ = SteadyClock::now();
    const double yaw_deg = yaw_reference_->apply(angles.yaw_deg);
    std_msgs::msg::Float64 value;
    value.data = angles.yaw_deg;
    raw_pub_->publish(value);
    value.data = yaw_deg;
    deg_pub_->publish(value);
    value.data = yaw_deg * kDegToRad;
    rad_pub_->publish(value);

    sensor_msgs::msg::Imu imu;
    // The standard 0x53 frame has no acquisition timestamp: use host receipt time.
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;
    const double sr = std::sin(angles.roll_deg * kDegToRad * 0.5);
    const double cr = std::cos(angles.roll_deg * kDegToRad * 0.5);
    const double sp = std::sin(angles.pitch_deg * kDegToRad * 0.5);
    const double cp = std::cos(angles.pitch_deg * kDegToRad * 0.5);
    const double sy = std::sin(yaw_deg * kDegToRad * 0.5);
    const double cy = std::cos(yaw_deg * kDegToRad * 0.5);
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;
    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    // Zero orientation covariance means unknown; the other measurements are absent.
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);
    if (!received_first_) {
      received_first_ = true;
      RCLCPP_INFO(get_logger(), "First WIT angle frame: raw yaw %.3f deg, output %.3f deg",
        angles.yaw_deg, yaw_deg);
    }
  }

  void poll_serial()
  {
    if (fd_ < 0 && (SteadyClock::now() < next_attempt_ || !connect_serial())) {
      return;
    }
    pollfd descriptor{fd_, POLLIN, 0};
    const int status = ::poll(&descriptor, 1, 0);
    if (status < 0) {
      if (errno != EINTR) {
        disconnect(std::string("poll: ") + std::strerror(errno));
      }
      return;
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      disconnect("poll reported disconnect/error");
      return;
    }
    if (descriptor.revents & POLLIN) {
      uint8_t bytes[512];
      // Bound work per callback, even if an unexpected device floods the port.
      for (int batch = 0; batch < 8; ++batch) {
        const ssize_t count = ::read(fd_, bytes, sizeof(bytes));
        if (count > 0) {
          for (ssize_t i = 0; i < count; ++i) {
            if (const auto angles = parser_.push(bytes[i])) {
              publish_angles(*angles);
            }
          }
        } else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        } else if (errno != EINTR) {
          disconnect(std::string("read: ") + std::strerror(errno));
          return;
        }
      }
    }
    if (std::chrono::duration<double>(SteadyClock::now() - last_valid_).count() > stale_timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No fresh WIT 0x53 angle frame for %.2f s; lifetime bytes=%llu packets=%llu "
        "angles=%llu checksum_errors=%llu; check baud/output settings/wiring",
        stale_timeout_s_, static_cast<unsigned long long>(parser_.bytes_received()),
        static_cast<unsigned long long>(parser_.valid_packets()),
        static_cast<unsigned long long>(parser_.angle_packets()),
        static_cast<unsigned long long>(parser_.checksum_errors()));
    }
  }

  int fd_{-1};
  int baud_rate_{9600};
  speed_t speed_{B9600};
  std::string serial_device_;
  std::string frame_id_;
  double stale_timeout_s_{0.5};
  double reconnect_interval_s_{1.0};
  bool received_first_{false};
  SteadyClock::time_point next_attempt_{};
  SteadyClock::time_point last_valid_{};
  WitImuParser parser_;
  std::unique_ptr<WitYawReference> yaw_reference_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_pub_, deg_pub_, rad_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robot_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<robot_sensors::WitImuNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wit_imu_node"), "startup/runtime failure: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
