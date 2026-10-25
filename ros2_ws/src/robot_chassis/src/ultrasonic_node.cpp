#include "robot_chassis/ultrasonic_node.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_chassis
{
UltrasonicNode::UltrasonicNode() : Node("ultrasonic_node")
{
  declare_parameter("front_topic", "/ultrasonic/front");
  declare_parameter("rear_topic", "/ultrasonic/rear");
  declare_parameter("left_topic", "/ultrasonic/left");
  declare_parameter("right_topic", "/ultrasonic/right");
  declare_parameter("publish_rate_hz", 20.0);
  declare_parameter("stale_timeout_s", 0.2);

  const auto front_topic = get_parameter("front_topic").as_string();
  const auto rear_topic = get_parameter("rear_topic").as_string();
  const auto left_topic = get_parameter("left_topic").as_string();
  const auto right_topic = get_parameter("right_topic").as_string();
  const auto publish_rate = get_parameter("publish_rate_hz").as_double();
  stale_timeout_s_ = get_parameter("stale_timeout_s").as_double();

  if (publish_rate <= 0.0 || stale_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("publish_rate_hz and stale_timeout_s must be positive");
  }

  front_sub_ = create_subscription<sensor_msgs::msg::Range>(
    front_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Range::SharedPtr message) { update_front(message); });
  rear_sub_ = create_subscription<sensor_msgs::msg::Range>(
    rear_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Range::SharedPtr message) { update_rear(message); });
  left_sub_ = create_subscription<sensor_msgs::msg::Range>(
    left_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Range::SharedPtr message) { update_left(message); });
  right_sub_ = create_subscription<sensor_msgs::msg::Range>(
    right_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Range::SharedPtr message) { update_right(message); });

  publisher_ = create_publisher<robot_arm_interfaces::msg::UltrasonicArray>(
    "/base/ultrasonic", rclcpp::SensorDataQoS());
  const auto period = std::chrono::duration<double>(1.0 / publish_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    [this]() { publish(); });
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

void UltrasonicNode::update_front(const sensor_msgs::msg::Range::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  front_ = convert(*message);
}

void UltrasonicNode::update_rear(const sensor_msgs::msg::Range::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  rear_ = convert(*message);
}

void UltrasonicNode::update_left(const sensor_msgs::msg::Range::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  left_ = convert(*message);
}

void UltrasonicNode::update_right(const sensor_msgs::msg::Range::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  right_ = convert(*message);
}

void UltrasonicNode::publish()
{
  const auto now = std::chrono::steady_clock::now();
  Reading front;
  Reading rear;
  Reading left;
  Reading right;
  {
    std::scoped_lock lock(mutex_);
    front = front_;
    rear = rear_;
    left = left_;
    right = right_;
  }

  const auto is_fresh = [this, now](const Reading &reading) {
    return reading.valid &&
      std::chrono::duration<double>(now - reading.stamp).count() <= stale_timeout_s_;
  };

  robot_arm_interfaces::msg::UltrasonicArray message;
  message.header.stamp = this->now();
  message.header.frame_id = "base_link";
  message.front_m = front.value_m;
  message.rear_m = rear.value_m;
  message.left_m = left.value_m;
  message.right_m = right.value_m;
  message.front_valid = is_fresh(front);
  message.rear_valid = is_fresh(rear);
  message.left_valid = is_fresh(left);
  message.right_valid = is_fresh(right);
  publisher_->publish(message);
}
}  // namespace robot_chassis

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_chassis::UltrasonicNode>());
  rclcpp::shutdown();
  return 0;
}
