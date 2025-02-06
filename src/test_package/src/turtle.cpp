#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class Turtle: public rclcpp::Node {
    public:
     Turtle() : Node("square_turtle"), step_(0){
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(2s, std::bind(&Turtle::move_turtle, this));

     }


    private:
         void move_turtle(){
            auto msg = geometry_msgs::msg::Twist();

            if(step_ % 2 == 0){
                msg.linear.x = 2.0;
                msg.angular.z = 0.0;

            }else{
                msg.linear.x = 0.0;
                msg.angular.z = 1.57;
            }

            publisher_->publish(msg);
            step_ = (step_ + 1) % 8;
         }



    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    int step_;




};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtle>());
    rclcpp::shutdown();
    return 0;
}