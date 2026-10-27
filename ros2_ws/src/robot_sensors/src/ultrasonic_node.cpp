#include "robot_sensors/ultrasonic_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace robot_sensors
{
UltrasonicNode::UltrasonicNode() : Node("ultrasonic_node")
{
  declare_parameter("serial_enabled", true);
  declare_parameter("serial_baud_rate", 115200);
  declare_parameter("uart_output_mode", "auto");
  declare_parameter("controlled_trigger_period_s", 0.20);
  declare_parameter("distance_unit", "mm");
  declare_parameter("serial_read_buffer_size", 128);
  declare_parameter("serial_min_range_m", 0.03);
  declare_parameter("serial_max_range_m", 5.0);
  declare_parameter("field_of_view_rad", 1.22);
  declare_parameter("front_serial_device", "/dev/ttyCH9344USB0");
  declare_parameter("rear_serial_device", "/dev/ttyCH9344USB1");
  declare_parameter("left_serial_device", "/dev/ttyCH9344USB2");
  declare_parameter("right_serial_device", "/dev/ttyCH9344USB3");
  declare_parameter("front_frame_id", "ultrasonic_front_link");
  declare_parameter("rear_frame_id", "ultrasonic_rear_link");
  declare_parameter("left_frame_id", "ultrasonic_left_link");
  declare_parameter("right_frame_id", "ultrasonic_right_link");
  declare_parameter("front_topic", "/ultrasonic/front");
  declare_parameter("rear_topic", "/ultrasonic/rear");
  declare_parameter("left_topic", "/ultrasonic/left");
  declare_parameter("right_topic", "/ultrasonic/right");
  declare_parameter("publish_rate_hz", 20.0);
  declare_parameter("stale_timeout_s", 0.6);

  const auto front_topic = get_parameter("front_topic").as_string();
  const auto rear_topic = get_parameter("rear_topic").as_string();
  const auto left_topic = get_parameter("left_topic").as_string();
  const auto right_topic = get_parameter("right_topic").as_string();
  const auto publish_rate = get_parameter("publish_rate_hz").as_double();
  serial_enabled_ = get_parameter("serial_enabled").as_bool();
  serial_baud_rate_ = static_cast<int>(get_parameter("serial_baud_rate").as_int());
  uart_output_mode_ = get_parameter("uart_output_mode").as_string();
  controlled_trigger_period_s_ = get_parameter("controlled_trigger_period_s").as_double();
  const auto distance_unit = get_parameter("distance_unit").as_string();
  distance_data_is_microseconds_ = distance_unit == "us";
  serial_read_buffer_size_ = static_cast<int>(get_parameter("serial_read_buffer_size").as_int());
  serial_min_range_m_ = get_parameter("serial_min_range_m").as_double();
  serial_max_range_m_ = get_parameter("serial_max_range_m").as_double();
  field_of_view_rad_ = get_parameter("field_of_view_rad").as_double();
  stale_timeout_s_ = get_parameter("stale_timeout_s").as_double();

  if (publish_rate <= 0.0 || stale_timeout_s_ <= 0.0 || serial_baud_rate_ <= 0 ||
      serial_read_buffer_size_ <= 0 || serial_min_range_m_ <= 0.0 ||
      serial_max_range_m_ <= serial_min_range_m_ ||
      field_of_view_rad_ <= 0.0 || field_of_view_rad_ > 6.283185307179586 ||
      (uart_output_mode_ != "auto" && uart_output_mode_ != "controlled") ||
      controlled_trigger_period_s_ <= 0.15 ||
      (distance_unit != "mm" && distance_unit != "us"))
  {
    throw std::invalid_argument("invalid ultrasonic serial or timing parameter");
  }

  front_cm_pub_ = create_publisher<std_msgs::msg::Float64>(front_topic + "_cm", rclcpp::SensorDataQoS());
  rear_cm_pub_ = create_publisher<std_msgs::msg::Float64>(rear_topic + "_cm", rclcpp::SensorDataQoS());
  left_cm_pub_ = create_publisher<std_msgs::msg::Float64>(left_topic + "_cm", rclcpp::SensorDataQoS());
  right_cm_pub_ = create_publisher<std_msgs::msg::Float64>(right_topic + "_cm", rclcpp::SensorDataQoS());

  serial_channels_[0].device = get_parameter("front_serial_device").as_string();
  serial_channels_[1].device = get_parameter("rear_serial_device").as_string();
  serial_channels_[2].device = get_parameter("left_serial_device").as_string();
  serial_channels_[3].device = get_parameter("right_serial_device").as_string();
  serial_channels_[0].frame_id = get_parameter("front_frame_id").as_string();
  serial_channels_[1].frame_id = get_parameter("rear_frame_id").as_string();
  serial_channels_[2].frame_id = get_parameter("left_frame_id").as_string();
  serial_channels_[3].frame_id = get_parameter("right_frame_id").as_string();

  if (!serial_enabled_)
  {
    front_sub_ = create_subscription<sensor_msgs::msg::Range>(
      front_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Range::ConstSharedPtr message) { update_front(message); });
    rear_sub_ = create_subscription<sensor_msgs::msg::Range>(
      rear_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Range::ConstSharedPtr message) { update_rear(message); });
    left_sub_ = create_subscription<sensor_msgs::msg::Range>(
      left_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Range::ConstSharedPtr message) { update_left(message); });
    right_sub_ = create_subscription<sensor_msgs::msg::Range>(
      right_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Range::ConstSharedPtr message) { update_right(message); });
  }

  if (serial_enabled_)
  {
    front_range_pub_ = create_publisher<sensor_msgs::msg::Range>(front_topic, rclcpp::SensorDataQoS());
    rear_range_pub_ = create_publisher<sensor_msgs::msg::Range>(rear_topic, rclcpp::SensorDataQoS());
    left_range_pub_ = create_publisher<sensor_msgs::msg::Range>(left_topic, rclcpp::SensorDataQoS());
    right_range_pub_ = create_publisher<sensor_msgs::msg::Range>(right_topic, rclcpp::SensorDataQoS());
    open_serial_channels();
  }

  publisher_ = create_publisher<robot_arm_interfaces::msg::UltrasonicArray>(
    "/base/ultrasonic", rclcpp::SensorDataQoS());
  const auto period = std::chrono::duration<double>(1.0 / publish_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    [this]() { publish(); });

  RCLCPP_INFO(
    get_logger(), "ultrasonic node ready; source=%s, baud=%d, UART mode=%s, distance unit=%s",
    serial_enabled_ ? "DYP-A21 UART" : "ROS Range topics", serial_baud_rate_,
    uart_output_mode_.c_str(), distance_unit.c_str());
}

UltrasonicNode::~UltrasonicNode()
{
  close_serial_channels();
}

namespace
{
speed_t ultrasonic_baud_to_termios(const int baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::invalid_argument("unsupported ultrasonic serial baud rate");
  }
}
}

void UltrasonicNode::open_serial_channels()
{
  const auto speed = ultrasonic_baud_to_termios(serial_baud_rate_);
  try
  {
    for (auto &channel : serial_channels_)
    {
      if (channel.device.empty())
      {
        throw std::invalid_argument("ultrasonic serial device path is empty");
      }
      channel.fd = ::open(channel.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (channel.fd < 0)
      {
        throw std::runtime_error(
                "cannot open ultrasonic serial device " + channel.device + ": " + std::strerror(errno));
      }

      termios tty{};
      if (tcgetattr(channel.fd, &tty) != 0)
      {
        throw std::runtime_error(
                "tcgetattr failed for ultrasonic serial device " + channel.device + ": " +
                std::strerror(errno));
      }
      if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0)
      {
        throw std::runtime_error(
                "setting baud rate failed for ultrasonic serial device " + channel.device + ": " +
                std::strerror(errno));
      }
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
      tty.c_iflag = 0;
      tty.c_oflag = 0;
      tty.c_lflag = 0;
      tty.c_cflag |= CLOCAL | CREAD;
      tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 0;
      if (tcsetattr(channel.fd, TCSANOW, &tty) != 0)
      {
        throw std::runtime_error(
                "tcsetattr failed for ultrasonic serial device " + channel.device + ": " +
                std::strerror(errno));
      }
      tcflush(channel.fd, TCIFLUSH);
      RCLCPP_INFO(get_logger(), "opened ultrasonic serial port %s", channel.device.c_str());
    }
  }
  catch (...)
  {
    close_serial_channels();
    throw;
  }
}

void UltrasonicNode::close_serial_channels()
{
  for (auto &channel : serial_channels_)
  {
    if (channel.fd >= 0)
    {
      ::close(channel.fd);
      channel.fd = -1;
    }
  }
}

void UltrasonicNode::poll_serial()
{
  std::vector<char> read_buffer(static_cast<std::size_t>(serial_read_buffer_size_));
  for (std::size_t i = 0; i < serial_channels_.size(); ++i)
  {
    auto &channel = serial_channels_[i];
    if (channel.fd < 0)
    {
      continue;
    }
    if (uart_output_mode_ == "controlled")
    {
      const auto trigger_time = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration<double>(trigger_time - channel.last_trigger).count();
      if (elapsed >= controlled_trigger_period_s_)
      {
        constexpr char kMeasurementTrigger = 0x00;
        if (::write(channel.fd, &kMeasurementTrigger, sizeof(kMeasurementTrigger)) ==
          static_cast<ssize_t>(sizeof(kMeasurementTrigger)))
        {
          channel.last_trigger = trigger_time;
        }
        else
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "ultrasonic trigger write failed on %s: %s",
            channel.device.c_str(), std::strerror(errno));
        }
      }
    }
    const auto count = ::read(channel.fd, read_buffer.data(), read_buffer.size());
    if (count > 0)
    {
      channel.receive_buffer.append(read_buffer.data(), static_cast<std::size_t>(count));
      parse_serial_channel(i);
    }
    else if (count < 0 && errno != EAGAIN && errno != EINTR)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "ultrasonic serial read failed on %s: %s",
        channel.device.c_str(), std::strerror(errno));
    }
  }
}

void UltrasonicNode::parse_serial_channel(const std::size_t index)
{
  auto &channel = serial_channels_.at(index);
  constexpr uint8_t kFrameHeader = 0xff;
  constexpr std::size_t kFrameSize = 4;

  while (channel.receive_buffer.size() >= kFrameSize)
  {
    const auto header = channel.receive_buffer.find(static_cast<char>(kFrameHeader));
    if (header == std::string::npos)
    {
      channel.receive_buffer.clear();
      return;
    }
    if (header > 0U)
    {
      channel.receive_buffer.erase(0, header);
    }
    if (channel.receive_buffer.size() < kFrameSize)
    {
      return;
    }

    const auto high = static_cast<uint8_t>(channel.receive_buffer[1]);
    const auto low = static_cast<uint8_t>(channel.receive_buffer[2]);
    const auto checksum = static_cast<uint8_t>(channel.receive_buffer[3]);
    const auto expected = static_cast<uint8_t>(
      (static_cast<unsigned int>(kFrameHeader) + high + low) & 0xffU);
    if (checksum != expected)
    {
      channel.receive_buffer.erase(0, 1);
      continue;
    }

    const auto distance_raw = static_cast<unsigned int>(high) * 256U + low;
    const auto distance_mm = distance_data_is_microseconds_ ?
      static_cast<double>(distance_raw) / 5.75 : static_cast<double>(distance_raw);
    const auto distance_m = distance_mm / 1000.0;
    const bool sensor_status_frame = distance_raw == 0xfffdU || distance_raw == 0xfffeU;
    {
      std::scoped_lock lock(mutex_);
      channel.reading.value_m = distance_m;
      channel.reading.valid = !sensor_status_frame && distance_m >= serial_min_range_m_ &&
        distance_m <= serial_max_range_m_;
      channel.reading.stamp = std::chrono::steady_clock::now();
    }
    channel.receive_buffer.erase(0, kFrameSize);
  }
}

UltrasonicNode::Reading UltrasonicNode::convert(const sensor_msgs::msg::Range &message)
{
  Reading reading;
  reading.value_m = static_cast<double>(message.range);
  reading.valid = std::isfinite(reading.value_m) && reading.value_m >= message.min_range &&
    reading.value_m <= message.max_range;
  reading.stamp = std::chrono::steady_clock::now();
  return reading;
}

void UltrasonicNode::update_front(const sensor_msgs::msg::Range::ConstSharedPtr message)
{
  std::scoped_lock lock(mutex_);
  front_ = convert(*message);
}

void UltrasonicNode::update_rear(const sensor_msgs::msg::Range::ConstSharedPtr message)
{
  std::scoped_lock lock(mutex_);
  rear_ = convert(*message);
}

void UltrasonicNode::update_left(const sensor_msgs::msg::Range::ConstSharedPtr message)
{
  std::scoped_lock lock(mutex_);
  left_ = convert(*message);
}

void UltrasonicNode::update_right(const sensor_msgs::msg::Range::ConstSharedPtr message)
{
  std::scoped_lock lock(mutex_);
  right_ = convert(*message);
}

void UltrasonicNode::publish()
{
  if (serial_enabled_)
  {
    poll_serial();
  }

  const auto now = std::chrono::steady_clock::now();
  Reading front;
  Reading rear;
  Reading left;
  Reading right;
  {
    std::scoped_lock lock(mutex_);
    if (serial_enabled_)
    {
      front = serial_channels_[0].reading;
      rear = serial_channels_[1].reading;
      left = serial_channels_[2].reading;
      right = serial_channels_[3].reading;
    }
    else
    {
      front = front_;
      rear = rear_;
      left = left_;
      right = right_;
    }
  }

  const auto is_fresh = [this, now](const Reading &reading) {
    return reading.valid &&
      std::chrono::duration<double>(now - reading.stamp).count() <= stale_timeout_s_;
  };
  front.valid = is_fresh(front);
  rear.valid = is_fresh(rear);
  left.valid = is_fresh(left);
  right.valid = is_fresh(right);

  robot_arm_interfaces::msg::UltrasonicArray message;
  message.header.stamp = this->now();
  message.header.frame_id = "base_link";
  message.front_m = front.value_m;
  message.rear_m = rear.value_m;
  message.left_m = left.value_m;
  message.right_m = right.value_m;
  message.front_valid = front.valid;
  message.rear_valid = rear.valid;
  message.left_valid = left.valid;
  message.right_valid = right.valid;
  publisher_->publish(message);

  if (serial_enabled_)
  {
    publish_range(front, serial_channels_[0].frame_id, front_range_pub_, front_cm_pub_);
    publish_range(rear, serial_channels_[1].frame_id, rear_range_pub_, rear_cm_pub_);
    publish_range(left, serial_channels_[2].frame_id, left_range_pub_, left_cm_pub_);
    publish_range(right, serial_channels_[3].frame_id, right_range_pub_, right_cm_pub_);
  }
  else
  {
    publish_centimeters(front, front_cm_pub_);
    publish_centimeters(rear, rear_cm_pub_);
    publish_centimeters(left, left_cm_pub_);
    publish_centimeters(right, right_cm_pub_);
  }
}

void UltrasonicNode::publish_centimeters(
  const Reading &reading,
  const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &publisher)
{
  std_msgs::msg::Float64 cm;
  cm.data = reading.valid ? std::round(reading.value_m * 1000.0) / 10.0 :
    std::numeric_limits<double>::quiet_NaN();
  publisher->publish(cm);
}

void UltrasonicNode::publish_range(
  const Reading &reading, const std::string &frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr &publisher,
  const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &cm_publisher)
{
  sensor_msgs::msg::Range range;
  range.header.stamp = now();
  range.header.frame_id = frame_id;
  range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range.field_of_view = static_cast<float>(field_of_view_rad_);
  range.min_range = static_cast<float>(serial_min_range_m_);
  range.max_range = static_cast<float>(serial_max_range_m_);
  range.range = reading.valid ? static_cast<float>(reading.value_m) :
    std::numeric_limits<float>::quiet_NaN();
  publisher->publish(range);
  publish_centimeters(reading, cm_publisher);
}
}  // namespace robot_sensors

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    const auto node = std::make_shared<robot_sensors::UltrasonicNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ultrasonic_node"), "startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
