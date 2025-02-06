#include "rclcpp/rclcpp.hpp"
#include "test_package/srv/square_number.hpp"
#include <memory>

class NodeA : public rclcpp::Node{

    public:
       NodeA() : Node("node_a"){
        service_ = this-> create_service<test_package::srv::SquareNumber>(
            "service_a", std::bind(&NodeA::handle_request, this, std::placeholders::_1,std::placeholders::_2));
        client_ = this-> create_client<test_package::srv::SquareNumber>("service_b");
        timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&NodeA::send_request, this));
       }


    private:
        void handle_request(const std::shared_ptr<test_package::srv::SquareNumber::Request> request,
                            std::shared_ptr<test_package::srv::SquareNumber::Response> response){
                                response-> squared_number = request-> number * request->number;
                                RCLCPP_INFO(this->get_logger(), "Node A get: %ld, and responsed with: %ld", request->number, response->squared_number);
        }

        void send_request(){
            auto request = std::make_shared<test_package::srv::SquareNumber::Request>();
            request->number = 5;

            auto future = client_->async_send_request(request, [this](rclcpp::Client<test_package::srv::SquareNumber>::SharedFuture result){
                RCLCPP_INFO(this->get_logger(), "Node A got response from Node B: %ld", result.get()->squared_number);
            });
        }

        rclcpp::Service<test_package::srv::SquareNumber>::SharedPtr service_;
        rclcpp::Client<test_package::srv::SquareNumber>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;



};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeA>());
    rclcpp::shutdown();
    return 0;
}