#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class PickSequenceNode : public rclcpp::Node
{
public:
  PickSequenceNode()
  : Node("pick_sequence_node")
  {
    publisher_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        publish_sequence();
        timer_->cancel();
      });
  }

private:
  void publish_sequence()
  {
    trajectory_msgs::msg::JointTrajectory msg;

    msg.joint_names = {
      "joint1", "joint2", "joint3", "joint4", "joint5"
    };

    add_point(msg, {0.0, 0.0, 0.0, 0.0, 0.0}, 2.0);  // 起始位
    add_point(msg, {0.0, 0.5, 0.5, 0.2, 0.0}, 6.0); // 抓取位
    add_point(msg, {0.0, 0.0, 0.0, 0.0, 0.0}, 10.0);// 返回起始位

    publisher_->publish(msg);
  }

  void add_point(
    trajectory_msgs::msg::JointTrajectory &msg,
    const std::array<double, 5> &positions,
    double time_sec)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(positions.begin(), positions.end());
    point.time_from_start.sec = static_cast<int32_t>(time_sec);
    msg.points.push_back(point);
  }

  rclcpp::Publisher<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickSequenceNode>());
  rclcpp::shutdown();
  return 0;
}