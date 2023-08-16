#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTFBroadcaster: public rclcpp::Node
{
public:
    StaticTFBroadcaster(const std::string& name, char *argv[]):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a StaticTFBroadcaster.");
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        pub_static_tf(argv);
    }

private:
    void pub_static_tf(char *argv[])
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = argv[7];
        transform.child_frame_id = argv[8];
        transform.transform.translation.x = atof(argv[1]);
        transform.transform.translation.y = atof(argv[2]);
        transform.transform.translation.z = atof(argv[3]);

        tf2::Quaternion qtn;
        qtn.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
        transform.transform.rotation.x = qtn.getX();
        transform.transform.rotation.y = qtn.getY();
        transform.transform.rotation.z = qtn.getZ();
        transform.transform.rotation.w = qtn.getW();

        broadcaster_->sendTransform(transform);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char *argv[])
{
    if (argc != 9)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid nums!!!");
        return 1;
    }
    
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StaticTFBroadcaster>("static_tf_bc", argv));
    rclcpp::shutdown();
    return 0;
}