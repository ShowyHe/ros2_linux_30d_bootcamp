#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("cpp_talker"), count_(0)
  {
    this->declare_parameter<double>("publish_rate", 2.0);     // Hz
    this->declare_parameter<std::string>("prefix", "w2_d3");  // message prefix

    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("prefix", prefix_);

    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_rate <= 0, fallback to 1.0");
      publish_rate_ = 1.0;
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("/cpp_chatter", 10);

    int period_ms = static_cast<int>(1000.0 / publish_rate_);
    if (period_ms < 1) period_ms = 1;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&Talker::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "started: publish_rate=%.2fHz prefix=%s",
                publish_rate_, prefix_.c_str());
  }

private:
  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = prefix_ + " hello #" + std::to_string(count_++);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "pub: %s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  double publish_rate_;
  std::string prefix_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}