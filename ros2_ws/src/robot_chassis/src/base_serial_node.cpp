#include "robot_chassis/base_serial_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robot_chassis
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
    default: throw std::invalid_argument("unsupported serial baud rate");
  }
}

bool finite(const double value)
{
  return std::isfinite(value);
}

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

BaseSerialNode::BaseSerialNode() : Node("base_serial_node")
{
  declare_parameter("serial_device", "/dev/serial/by-id/REPLACE_WITH_STM32");
  declare_parameter("baud_rate", 115200);
  declare_parameter("dry_run", true);
  declare_parameter("allow_motion", false);
  declare_parameter("command_timeout_s", 0.2);
  declare_parameter("publish_rate_hz", 20.0);

  serial_device_ = get_parameter("serial_device").as_string();
  baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
  dry_run_ = get_parameter("dry_run").as_bool();
  allow_motion_ = get_parameter("allow_motion").as_bool();
  command_timeout_s_ = get_parameter("command_timeout_s").as_double();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

  if (command_timeout_s_ <= 0.0 || publish_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("command_timeout_s and publish_rate_hz must be positive");
  }

  command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/base/cmd_vel", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      command_callback(message);
    });
  state_pub_ = create_publisher<robot_arm_interfaces::msg::BaseState>(
    "/base/state", rclcpp::QoS(10));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/base/imu", rclcpp::SensorDataQoS());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/base/odom", rclcpp::SensorDataQoS());

  last_command_time_ = std::chrono::steady_clock::now();
  if (!dry_run_)
  {
    open_serial();
    running_.store(true);
    read_thread_ = std::thread(&BaseSerialNode::read_loop, this);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "dry_run=true: STM32 serial I/O is disabled");
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  watchdog_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    [this]() { watchdog_callback(); });

  RCLCPP_INFO(
    get_logger(), "base serial ready; motion=%s, protocol=CMD_VEL vx vy wz newline",
    allow_motion_ ? "enabled" : "disabled");
}

BaseSerialNode::~BaseSerialNode()
{
  running_.store(false);
  if (read_thread_.joinable())
  {
    read_thread_.join();
  }
  close_serial();
}

void BaseSerialNode::open_serial()
{
  const auto speed = baud_to_termios(baud_rate_);
  fd_ = ::open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    throw std::runtime_error(
            "cannot open chassis serial device " + serial_device_ + ": " + std::strerror(errno));
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0)
  {
    const auto message = std::strerror(errno);
    close_serial();
    throw std::runtime_error("tcgetattr failed: " + std::string(message));
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
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
    close_serial();
    throw std::runtime_error("tcsetattr failed: " + std::string(message));
  }
}

void BaseSerialNode::close_serial()
{
  std::scoped_lock lock(io_mutex_);
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

void BaseSerialNode::command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!finite(message->linear.x) || !finite(message->linear.y) ||
      !finite(message->angular.z))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignoring non-finite base command");
    return;
  }

  geometry_msgs::msg::Twist command = *message;
  command.linear.x = clamp(command.linear.x, -1.0, 1.0);
  command.linear.y = clamp(command.linear.y, -1.0, 1.0);
  command.angular.z = clamp(command.angular.z, -2.0, 2.0);

  {
    std::scoped_lock lock(state_mutex_);
    last_command_ = command;
    last_command_time_ = std::chrono::steady_clock::now();
    timeout_active_ = false;
  }
  write_command(command);
}

void BaseSerialNode::write_command(const geometry_msgs::msg::Twist &command)
{
  if (!allow_motion_ || dry_run_ || fd_ < 0)
  {
    return;
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6)
         << "CMD_VEL " << command.linear.x << ' ' << command.linear.y << ' '
         << command.angular.z << '\n';
  const auto text = stream.str();

  std::scoped_lock lock(io_mutex_);
  std::size_t sent = 0;
  while (sent < text.size())
  {
    const auto count = ::write(fd_, text.data() + sent, text.size() - sent);
    if (count < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "chassis serial write failed: %s", std::strerror(errno));
      return;
    }
    sent += static_cast<std::size_t>(count);
  }
}

void BaseSerialNode::send_zero_once()
{
  geometry_msgs::msg::Twist zero;
  write_command(zero);
}

void BaseSerialNode::watchdog_callback()
{
  const auto now = std::chrono::steady_clock::now();
  bool timed_out = false;
  {
    std::scoped_lock lock(state_mutex_);
    timed_out = std::chrono::duration<double>(now - last_command_time_).count() > command_timeout_s_;
    if (timed_out && !timeout_active_)
    {
      timeout_active_ = true;
      last_command_ = geometry_msgs::msg::Twist();
      RCLCPP_ERROR(get_logger(), "base command timeout: sending zero velocity");
    }
  }
  if (timed_out)
  {
    send_zero_once();
  }
  publish_state();
}

void BaseSerialNode::publish_state()
{
  robot_arm_interfaces::msg::BaseState state;
  sensor_msgs::msg::Imu imu;
  nav_msgs::msg::Odometry odom;
  {
    std::scoped_lock lock(state_mutex_);
    state.header.stamp = now();
    state.vx_mps = vx_mps_;
    state.vy_mps = vy_mps_;
    state.wz_rad_s = wz_rad_s_;
    state.yaw_rad = yaw_rad_;
    state.stopped = std::abs(vx_mps_) < 0.03 && std::abs(vy_mps_) < 0.03 &&
      std::abs(wz_rad_s_) < 0.08;
    state.command_timed_out = timeout_active_;

    imu.header.stamp = state.header.stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.z = std::sin(yaw_rad_ * 0.5);
    imu.orientation.w = std::cos(yaw_rad_ * 0.5);
    imu.angular_velocity.z = gyro_z_rad_s_;

    odom.header.stamp = state.header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    odom.pose.pose.orientation = imu.orientation;
    odom.twist.twist.linear.x = vx_mps_;
    odom.twist.twist.linear.y = vy_mps_;
    odom.twist.twist.angular.z = wz_rad_s_;
  }
  state_pub_->publish(state);
  if (have_imu_)
  {
    imu_pub_->publish(imu);
  }
  if (have_odom_)
  {
    odom_pub_->publish(odom);
  }
}

void BaseSerialNode::read_loop()
{
  char buffer[256];
  while (running_.load())
  {
    const auto count = ::read(fd_, buffer, sizeof(buffer));
    if (count > 0)
    {
      receive_buffer_.append(buffer, static_cast<std::size_t>(count));
      std::size_t newline = 0;
      while ((newline = receive_buffer_.find('\n')) != std::string::npos)
      {
        auto line = receive_buffer_.substr(0, newline);
        receive_buffer_.erase(0, newline + 1);
        if (!line.empty() && line.back() == '\r')
        {
          line.pop_back();
        }
        parse_line(line);
      }
    }
    else if (count < 0 && errno != EAGAIN && errno != EINTR)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "chassis serial read failed: %s", std::strerror(errno));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void BaseSerialNode::parse_line(const std::string &line)
{
  std::istringstream stream(line);
  std::string tag;
  stream >> tag;
  if (tag == "IMU")
  {
    double yaw = 0.0;
    double gyro_z = 0.0;
    if (stream >> yaw >> gyro_z && finite(yaw) && finite(gyro_z))
    {
      std::scoped_lock lock(state_mutex_);
      yaw_rad_ = yaw;
      gyro_z_rad_s_ = gyro_z;
      have_imu_ = true;
    }
    return;
  }

  if (tag == "ODOM")
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    if (stream >> x >> y >> yaw >> vx >> vy >> wz && finite(x) && finite(y) &&
        finite(yaw) && finite(vx) && finite(vy) && finite(wz))
    {
      std::scoped_lock lock(state_mutex_);
      x_m_ = x;
      y_m_ = y;
      yaw_rad_ = yaw;
      vx_mps_ = vx;
      vy_mps_ = vy;
      wz_rad_s_ = wz;
      have_odom_ = true;
    }
  }
}
}  // namespace robot_chassis

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_chassis::BaseSerialNode>());
  rclcpp::shutdown();
  return 0;
}
