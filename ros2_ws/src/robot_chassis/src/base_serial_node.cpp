#include "robot_chassis/base_serial_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robot_chassis
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

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
  declare_parameter("stage_protocol_enabled", false);
  declare_parameter("command_timeout_s", 0.2);
  declare_parameter("publish_rate_hz", 20.0);
  declare_parameter("wheel_diameter_m", 0.075);
  declare_parameter("wheel_base_m", 0.170);
  declare_parameter("track_width_m", 0.186);
  declare_parameter("encoder_counts_per_wheel_rev", 1560.0);
  declare_parameter("max_wheel_speed_mps", 1.0);
  declare_parameter("feedback_timeout_s", 0.3);
  declare_parameter("imu_yaw_increment_deadband_rad", 0.0001);
  declare_parameter("imu_yaw_increment_deadband_startup_delay_s", 5.0);
  declare_parameter("wheel_sign_a", 1);
  declare_parameter("wheel_sign_b", 1);
  declare_parameter("wheel_sign_c", 1);
  declare_parameter("wheel_sign_d", 1);

  serial_device_ = get_parameter("serial_device").as_string();
  baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
  dry_run_ = get_parameter("dry_run").as_bool();
  allow_motion_ = get_parameter("allow_motion").as_bool();
  stage_protocol_enabled_ = get_parameter("stage_protocol_enabled").as_bool();
  command_timeout_s_ = get_parameter("command_timeout_s").as_double();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  wheel_diameter_m_ = get_parameter("wheel_diameter_m").as_double();
  wheel_base_m_ = get_parameter("wheel_base_m").as_double();
  track_width_m_ = get_parameter("track_width_m").as_double();
  encoder_counts_per_wheel_rev_ = get_parameter("encoder_counts_per_wheel_rev").as_double();
  max_wheel_speed_mps_ = get_parameter("max_wheel_speed_mps").as_double();
  feedback_timeout_s_ = get_parameter("feedback_timeout_s").as_double();
  const auto yaw_deadband = get_parameter("imu_yaw_increment_deadband_rad").as_double();
  const auto yaw_deadband_startup_delay_s =
    get_parameter("imu_yaw_increment_deadband_startup_delay_s").as_double();
  yaw_filter_.set_threshold(yaw_deadband);
  if (!finite(yaw_deadband_startup_delay_s) || yaw_deadband_startup_delay_s < 0.0)
  {
    throw std::invalid_argument(
            "imu_yaw_increment_deadband_startup_delay_s must be finite and non-negative");
  }
  yaw_deadband_rad_ = yaw_deadband;
  yaw_filter_startup_delay_s_ = yaw_deadband_startup_delay_s;
  yaw_filter_enable_time_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(yaw_filter_startup_delay_s_));
  yaw_filter_.set_threshold(0.0);
  wheel_sign_ = {
    static_cast<double>(get_parameter("wheel_sign_a").as_int()),
    static_cast<double>(get_parameter("wheel_sign_b").as_int()),
    static_cast<double>(get_parameter("wheel_sign_c").as_int()),
    static_cast<double>(get_parameter("wheel_sign_d").as_int())};

  if (command_timeout_s_ <= 0.0 || publish_rate_hz_ <= 0.0 || wheel_diameter_m_ <= 0.0 ||
      wheel_base_m_ <= 0.0 || track_width_m_ <= 0.0 || encoder_counts_per_wheel_rev_ <= 0.0 ||
      max_wheel_speed_mps_ <= 0.0 || feedback_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("invalid chassis serial or mecanum geometry parameter");
  }
  for (const auto sign : wheel_sign_)
  {
    if (sign != 1.0 && sign != -1.0)
    {
      throw std::invalid_argument("wheel_sign_a..d must be either 1 or -1");
    }
  }

  command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/base/cmd_vel", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      command_callback(message);
    });
  route_command_sub_ = create_subscription<robot_arm_interfaces::msg::RouteCommand>(
    "/base/route_command", rclcpp::QoS(10),
    [this](const robot_arm_interfaces::msg::RouteCommand::SharedPtr message) {
      route_command_callback(message);
    });
  stage_feedback_pub_ = create_publisher<robot_arm_interfaces::msg::StageFeedback>(
    "/base/stage_feedback", rclcpp::QoS(10));
  state_pub_ = create_publisher<robot_arm_interfaces::msg::BaseState>(
    "/base/state", rclcpp::QoS(10));
  wheel_state_pub_ = create_publisher<robot_arm_interfaces::msg::WheelState>(
    "/base/wheel_state", rclcpp::SensorDataQoS());
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
    get_logger(), "base serial ready; motion=%s, stage protocol=%s",
    allow_motion_ ? "enabled" : "disabled",
    stage_protocol_enabled_ ? "enabled" : "disabled");
  RCLCPP_INFO(
    get_logger(), "IMU yaw deadband %.6f rad/frame will activate after %.2f s",
    yaw_deadband, yaw_deadband_startup_delay_s);
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
  // The route topic is authoritative in staged mode; cmd_vel is its monitor mirror.
  if (!stage_protocol_enabled_) {
    process_command(*message, 0);
  }
}

void BaseSerialNode::route_command_callback(
  const robot_arm_interfaces::msg::RouteCommand::SharedPtr message)
{
  if (stage_protocol_enabled_ && message->stage_id <= 16) {
    process_command(message->velocity, message->stage_id);
  }
}

void BaseSerialNode::process_command(const geometry_msgs::msg::Twist &input, const uint8_t stage_id)
{
  if (!finite(input.linear.x) || !finite(input.linear.y) || !finite(input.angular.z))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignoring non-finite base command");
    return;
  }

  geometry_msgs::msg::Twist command = input;
  command.linear.x = clamp(command.linear.x, -1.0, 1.0);
  command.linear.y = clamp(command.linear.y, -1.0, 1.0);
  command.angular.z = clamp(command.angular.z, -2.0, 2.0);

  const auto rotation_radius = 0.5 * (wheel_base_m_ + track_width_m_);
  std::array<double, 4> wheel_speed_mps = {
    command.linear.x + command.linear.y - rotation_radius * command.angular.z,
    command.linear.x - command.linear.y - rotation_radius * command.angular.z,
    command.linear.x + command.linear.y + rotation_radius * command.angular.z,
    command.linear.x - command.linear.y + rotation_radius * command.angular.z};
  for (auto &speed : wheel_speed_mps)
  {
    speed = clamp(speed, -max_wheel_speed_mps_, max_wheel_speed_mps_);
  }

  {
    std::scoped_lock lock(state_mutex_);
    last_command_ = command;
    last_command_time_ = std::chrono::steady_clock::now();
    timeout_active_ = false;
  }
  write_wheel_command(wheel_speed_mps, stage_id);
}

void BaseSerialNode::write_wheel_command(
  const std::array<double, 4> &wheel_speed_mps, const std::optional<uint8_t> stage_id)
{
  std::scoped_lock lock(io_mutex_);
  if (stage_id && *stage_id != stage_id_) {
    stage_id_ = *stage_id;
    stage_reached_ = false;
  }
  write_wheel_command_locked(wheel_speed_mps);
}

void BaseSerialNode::write_wheel_command_locked(const std::array<double, 4> &wheel_speed_mps)
{
  if (!allow_motion_ || dry_run_ || fd_ < 0)
  {
    return;
  }

  std::ostringstream stream;
  stream << "CMD_WHEELS";
  for (const auto speed : wheel_speed_mps)
  {
    stream << ' ' << (stage_reached_ ? 0LL :
      static_cast<long long>(std::llround(speed * 1000.0)));
  }
  if (stage_protocol_enabled_) {
    stream << ' ' << static_cast<unsigned>(stage_id_);
  }
  stream << "\r\n";
  const auto text = stream.str();

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
    if (count == 0) {
      RCLCPP_ERROR(get_logger(), "chassis serial write made no progress");
      return;
    }
    sent += static_cast<std::size_t>(count);
  }
}

void BaseSerialNode::send_zero_once()
{
  write_wheel_command({0.0, 0.0, 0.0, 0.0});
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
      RCLCPP_ERROR(get_logger(), "base command timeout: sending zero wheel speeds");
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
  robot_arm_interfaces::msg::WheelState wheel_state;
  sensor_msgs::msg::Imu imu;
  nav_msgs::msg::Odometry odom;
  bool publish_feedback = false;
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

    wheel_state.header = state.header;
    wheel_state.speed_mps = wheel_speed_mps_;
    wheel_state.position_m = wheel_position_m_;
    wheel_state.encoder_delta = wheel_encoder_delta_;
    wheel_state.yaw_rad = yaw_rad_;
    wheel_state.valid = have_feedback_;
    publish_feedback = have_feedback_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_feedback_time_).count() <=
      feedback_timeout_s_;
  }
  state_pub_->publish(state);
  if (publish_feedback)
  {
    wheel_state_pub_->publish(wheel_state);
    imu_pub_->publish(imu);
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

YawIncrementFilter::Sample BaseSerialNode::update_yaw(const double raw_yaw)
{
  const auto now = std::chrono::steady_clock::now();
  if (have_yaw_feedback_ &&
      std::chrono::duration<double>(now - last_yaw_feedback_time_).count() > feedback_timeout_s_)
  {
    yaw_filter_.set_raw_reference(raw_yaw);
    yaw_filter_.set_threshold(0.0);
    yaw_filter_enabled_ = false;
    yaw_filter_enable_time_ = now +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(yaw_filter_startup_delay_s_));
    RCLCPP_INFO(
      get_logger(), "Yaw feedback resumed after a gap; deadband will activate after %.2f s",
      yaw_filter_startup_delay_s_);
  }
  if (!yaw_filter_enabled_ && now >= yaw_filter_enable_time_)
  {
    yaw_filter_.set_threshold(yaw_deadband_rad_);
    yaw_filter_enabled_ = true;
    RCLCPP_INFO(
      get_logger(), "IMU yaw increment deadband enabled: %.6f rad per feedback frame",
      yaw_deadband_rad_);
  }
  have_yaw_feedback_ = true;
  last_yaw_feedback_time_ = now;
  return yaw_filter_.update(raw_yaw);
}

void BaseSerialNode::parse_line(const std::string &line)
{
  std::istringstream stream(line);
  std::string tag;
  stream >> tag;
  if (tag == "STAGE_FB") {
    int stage = 0;
    int reached = 0;
    std::string extra;
    if (!stage_protocol_enabled_ || !(stream >> stage >> reached) ||
        stage < 1 || stage > 15 || (reached != 0 && reached != 1) || (stream >> extra)) {
      return;
    }
    robot_arm_interfaces::msg::StageFeedback message;
    {
      std::scoped_lock lock(io_mutex_);
      if (stage != stage_id_) {
        return;
      }
      // A later false report or repeated speed command cannot release this stage.
      stage_reached_ = stage_reached_ || reached == 1;
      if (stage_reached_) {
        write_wheel_command_locked({0.0, 0.0, 0.0, 0.0});
      }
      message.header.stamp = now();
      message.stage_id = stage_id_;
      message.reached = stage_reached_;
      stage_feedback_pub_->publish(message);
    }
    return;
  }
  if (tag == "FB")
  {
    uint32_t sequence = 0;
    double period_ms = 0.0;
    std::array<int32_t, 4> raw_delta{};
    double yaw_mdeg = 0.0;
    if (!(stream >> sequence >> period_ms >> raw_delta[0] >> raw_delta[1] >> raw_delta[2] >>
        raw_delta[3] >> yaw_mdeg) || !finite(period_ms) || !finite(yaw_mdeg) ||
        period_ms <= 0.0 || period_ms > 2000.0)
    {
      return;
    }

    const auto period_s = period_ms / 1000.0;
    const auto circumference_m = kPi * wheel_diameter_m_;
    std::scoped_lock lock(state_mutex_);
    for (std::size_t i = 0; i < raw_delta.size(); ++i)
    {
      const auto delta = wheel_sign_[i] > 0.0 ? raw_delta[i] : -raw_delta[i];
      wheel_encoder_delta_[i] = delta;
      const auto distance_m = static_cast<double>(delta) * circumference_m /
        encoder_counts_per_wheel_rev_;
      wheel_position_m_[i] += distance_m;
      wheel_speed_mps_[i] = distance_m / period_s;
    }

    const auto filtered = update_yaw(yaw_mdeg * kPi / 180000.0);
    const auto yaw = filtered.yaw;
    if (have_previous_yaw_)
    {
      wz_rad_s_ = filtered.delta / period_s;
    }
    else
    {
      wz_rad_s_ = 0.0;
      have_previous_yaw_ = true;
    }
    yaw_rad_ = yaw;
    gyro_z_rad_s_ = wz_rad_s_;

    const auto &wheel = wheel_speed_mps_;
    vx_mps_ = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) * 0.25;
    vy_mps_ = (wheel[0] - wheel[1] + wheel[2] - wheel[3]) * 0.25;
    const auto rotation_radius = 0.5 * (wheel_base_m_ + track_width_m_);
    const auto wheel_wz = (-wheel[0] - wheel[1] + wheel[2] + wheel[3]) /
      (4.0 * rotation_radius);
    if (!finite(wz_rad_s_))
    {
      wz_rad_s_ = wheel_wz;
    }
    x_m_ += (vx_mps_ * std::cos(yaw) - vy_mps_ * std::sin(yaw)) * period_s;
    y_m_ += (vx_mps_ * std::sin(yaw) + vy_mps_ * std::cos(yaw)) * period_s;
    last_feedback_time_ = std::chrono::steady_clock::now();
    have_feedback_ = true;
    have_odom_ = true;
    have_imu_ = true;
    (void)sequence;
    return;
  }

  if (tag == "IMU")
  {
    double yaw = 0.0;
    double gyro_z = 0.0;
    if (stream >> yaw >> gyro_z && finite(yaw) && finite(gyro_z))
    {
      std::scoped_lock lock(state_mutex_);
      const auto filtered = update_yaw(yaw);
      yaw_rad_ = filtered.yaw;
      gyro_z_rad_s_ = filtered.suppressed ? 0.0 : gyro_z;
      wz_rad_s_ = gyro_z_rad_s_;
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
      const auto filtered = update_yaw(yaw);
      yaw_rad_ = filtered.yaw;
      vx_mps_ = vx;
      vy_mps_ = vy;
      wz_rad_s_ = filtered.suppressed ? 0.0 : wz;
      gyro_z_rad_s_ = wz_rad_s_;
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
