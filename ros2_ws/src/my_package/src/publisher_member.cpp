#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
    public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
        // Tạo publisher với topic "chatter" và queue size là 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        
        // Tạo timer gọi callback mỗi 500ms
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Tạo message
        auto message = std_msgs::msg::String();
        message.data = "Hello from Publisher! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // Publish message
        publisher_->publish(message);
        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}