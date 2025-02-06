#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TurtleTFPublisher : public rclcpp::Node {
public:
    TurtleTFPublisher() : Node("turtle_tf_publisher") {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        publish_transform("world", "turtle1", 2.0, 2.0);
        publish_transform("world", "turtle2", 5.0, 5.0);
    }

private:
    void publish_transform(const std::string &parent, const std::string &child, double x, double y) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = parent;
        transform.child_frame_id = child;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        static_broadcaster_->sendTransform(transform);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleTFPublisher>());
    rclcpp::shutdown();
    return 0;
}
