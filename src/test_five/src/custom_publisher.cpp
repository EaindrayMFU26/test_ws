#include "rclcpp/rclcpp.hpp"
#include "test_five_msgs/msg/custom_data.hpp"

using namespace std::chrono_literals;

class CustomPublisher : public rclcpp::Node
{
public:
    CustomPublisher() : Node("custom_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<test_five_msgs::msg::CustomData>("custom_topic", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&CustomPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = test_five_msgs::msg::CustomData();
        message.number = count_++;
        message.value = count_ * 1.1;
        message.message = "Hello from Publisher";

        RCLCPP_INFO(this->get_logger(), "Publishing: number=%d, value=%.2f, message='%s'",
                    message.number, message.value, message.message.c_str());

        publisher_->publish(message);
    }

    rclcpp::Publisher<test_five_msgs::msg::CustomData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomPublisher>());
    rclcpp::shutdown();
    return 0;
}
