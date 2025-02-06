#include "rclcpp/rclcpp.hpp"
#include "test_five_msgs/msg/custom_data.hpp"

class CustomSubscriber : public rclcpp::Node
{
public:
    CustomSubscriber() : Node("custom_subscriber")
    {
        subscription_ = this->create_subscription<test_five_msgs::msg::CustomData>(
            "custom_topic", 10,
            std::bind(&CustomSubscriber::message_callback, this, std::placeholders::_1));
    }

private:
    void message_callback(const test_five_msgs::msg::CustomData::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: number=%d, value=%.2f, message='%s'",
                    msg->number, msg->value, msg->message.c_str());
    }

    rclcpp::Subscription<test_five_msgs::msg::CustomData>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
