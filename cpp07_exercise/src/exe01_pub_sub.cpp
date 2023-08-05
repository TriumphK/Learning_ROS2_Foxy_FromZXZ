#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ExePubSub: public rclcpp::Node
{
public:
    ExePubSub(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ExePubSub.");
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel", 1);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/t1/turtle1/pose", 1, std::bind(&ExePubSub::pose_cb, this, std::placeholders::_1));
    }
private:
    void pose_cb(const turtlesim::msg::Pose::ConstSharedPtr pose)
    {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = -(pose->angular_velocity);
        twist.linear.x = pose->linear_velocity;
        twist_pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ExePubSub>("exe_pubsub"));
    rclcpp::shutdown();
    return 0;
}