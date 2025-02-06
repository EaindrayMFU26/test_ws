#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Listener : public rclcpp::Node{

    public:
        Listener():Node("listener"){
            subscripton_ = this-> create_subscription<std_msgs::msg::Int32>(
                "my_topic", 10, std::bind(&Listener::topic_callback,this, std::placeholders::_1));
            
        }


    private:
        void topic_callback(const std_msgs::msg::Int32::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Listening : %d", msg->data);
        }
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscripton_;


};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}