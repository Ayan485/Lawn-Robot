#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float64>("Lawn", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private: 
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) 
    {
      _Float64 dataReciever = msg->data - 3;
      RCLCPP_INFO(this->get_logger(), "I'm Recieving: '%f'", dataReciever);
    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}