#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TurtleFollower : public rclcpp::Node
{
public:
    TurtleFollower()
    : Node("turtle_follower"),
      turtle_spawned_(false)
    {
        // Create a client to spawn a turtle
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer to check for the transformation and send velocity commands
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleFollower::on_timer, this));
    }

private:
     void on_timer()
{
    if (!turtle_spawned_) {
        spawn_turtle();
        return;
    }

    // Introduce a delay before checking for the transformation
    static auto last_check_time = this->get_clock()->now();
    auto current_time = this->get_clock()->now();
    
    // Check if enough time has passed since the last check
    if ((current_time - last_check_time).seconds() < 1.0) {
        return; // Wait for 1 second before checking again
    }
    last_check_time = current_time;

    // Check if the transformation is available
    if (tf_buffer_->canTransform("turtle2", "turtle1", current_time)) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            rclcpp::Time when = current_time - rclcpp::Duration(5, 0); // 5 seconds delay
            transform = tf_buffer_->lookupTransform("turtle2", "turtle1", when, 50ms);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // Create a velocity message to follow turtle1
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0 * sqrt(pow(transform.transform.translation.x, 2) + pow(transform.transform.translation.y, 2));
        msg.angular.z = atan2(transform.transform.translation.y, transform.transform.translation.x);
        publisher_->publish(msg);
    } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for transformation to become available...");
    }
}

    void spawn_turtle()
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 5.0; // Spawn position for turtle2
        request->y = 5.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // Call the spawn service
        using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (result->name == "turtle2") {
                turtle_spawned_ = true;
                RCLCPP_INFO(this->get_logger(), "Successfully spawned turtle2");
            }
        };
        spawner_->async_send_request(request, response_received_callback);
    }

    bool turtle_spawned_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleFollower>());
    rclcpp::shutdown();
    return 0;
}

