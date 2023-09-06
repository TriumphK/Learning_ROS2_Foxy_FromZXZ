#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ExeTFListener: public rclcpp::Node
{
public:
    ExeTFListener(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ExeTFListener.");
        this->declare_parameter("father_frame", "turtle2");
        this->declare_parameter("child_frame", "turtle1");
        father_frame = this->get_parameter("father_frame").as_string();
        child_frame = this->get_parameter("child_frame").as_string();

        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + father_frame + "/cmd_vel", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&ExeTFListener::on_timer, this));
    }

private:
    void on_timer()
    {
        try
        {
            geometry_msgs::msg::Twist twist;
            auto ts = buffer_->lookupTransform(father_frame, child_frame, tf2::TimePointZero);
            // RCLCPP_INFO(this->get_logger(), "%s to %s is: (%.2f,%.2f,%.2f).", ts.child_frame_id.c_str(), 
            //                                                                   ts.header.frame_id.c_str(),
            //                                                                   ts.transform.translation.x,
            //                                                                   ts.transform.translation.y,
            //                                                                   ts.transform.translation.z);
            twist.linear.x = 0.5 * sqrt(pow(ts.transform.translation.x, 2) + pow(ts.transform.translation.y, 2));
            twist.angular.z = 1.0 * atan2(ts.transform.translation.y, ts.transform.translation.x);
            twist_pub_->publish(twist);
        }
        catch(const tf2::LookupException& e)
        {
            // std::cerr << e.what() << '\n';
            RCLCPP_INFO(this->get_logger(), "Wrong: %s", e.what());
        }
    }

    std::string father_frame;
    std::string child_frame;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ExeTFListener>("tf_listener"));
    rclcpp::shutdown();
    return 0;
}
