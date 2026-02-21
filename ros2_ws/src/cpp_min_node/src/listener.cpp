#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("cpp_listener")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cpp_chatter", 10,
      std::bind(&Listener::on_msg, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "started: listening /cpp_chatter");
  }

private:
  void on_msg(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "sub: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}