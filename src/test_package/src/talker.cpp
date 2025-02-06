#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Talker : public rclcpp::Node {

    public:
        Talker(int input_value): Node("talker"), value_(input_value){
            publisher_ = this-> create_publisher <std_msgs::msg::Int32>("my_topic", 10);
            timer_ = this ->create_wall_timer(std::chrono::seconds(1), 
                                              std::bind(&Talker::publish_num,this));
        }


    private:
        void publish_num(){
            auto msg = std_msgs::msg::Int32();
            msg.data = value_;
            RCLCPP_INFO(this-> get_logger(), "Publishing Hello: %d", msg.data);
            publisher_->publish(msg);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        int value_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    int value = std::stoi(argv[1]);
    auto node = std::make_shared<Talker>(value);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}