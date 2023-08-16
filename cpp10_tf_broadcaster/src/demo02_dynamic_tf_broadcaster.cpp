#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class DynamicTF: public rclcpp::Node
{
public:
    DynamicTF(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a DynamicTF.");
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&DynamicTF::do_cb, this, std::placeholders::_1));
    }

private:
    void do_cb(const turtlesim::msg::Pose::ConstSharedPtr pose)
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "turtle1";
        ts.transform.translation.x = pose->x;
        ts.transform.translation.y = pose->y;
        ts.transform.translation.z = 0.0;

        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, pose->theta);
        ts.transform.rotation.x = qtn.getX();
        ts.transform.rotation.y = qtn.getY();
        ts.transform.rotation.z = qtn.getZ();
        ts.transform.rotation.w = qtn.getW();

        broadcaster_->sendTransform(ts);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DynamicTF>("dynamic_tf"));
    rclcpp::shutdown();
    return 0;
}