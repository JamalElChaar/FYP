#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Joint1LoopNode : public rclcpp::Node {
public:
  Joint1LoopNode() : Node("joint1_loop_node") {
    low_angle_ = this->declare_parameter<double>("low_angle_deg", 60.0);
    high_angle_ = this->declare_parameter<double>("high_angle_deg", 90.0);
    period_sec_ = this->declare_parameter<double>("period_sec", 1.0);

    if (period_sec_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid period_sec=%.3f, forcing to 1.0", period_sec_);
      period_sec_ = 1.0;
    }

    cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/esp32/joint_commands", 10);

    state_subscriber_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/esp32/joint_states", 10,
            std::bind(&Joint1LoopNode::state_callback, this,
                      std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period_sec_)),
        std::bind(&Joint1LoopNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "joint1_loop_node started: publishing /esp32/joint_commands, "
                "pattern %.1f <-> %.1f every %.2f sec",
                low_angle_, high_angle_, period_sec_);
  }

private:
  void timer_callback() {
    const double joint1 = use_high_ ? high_angle_ : low_angle_;
    use_high_ = !use_high_;

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {joint1, 90.0, 90.0, 90.0, 90.0, 90.0};
    cmd_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Published command: [%.1f, 90, 90, 90, 90, 90]", joint1);
  }

  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!msg->data.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "State feedback joint1=%.2f",
                   msg->data[0]);
    }
  }

  double low_angle_ = 60.0;
  double high_angle_ = 90.0;
  double period_sec_ = 1.0;
  bool use_high_ = false;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      state_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joint1LoopNode>());
  rclcpp::shutdown();
  return 0;
}
