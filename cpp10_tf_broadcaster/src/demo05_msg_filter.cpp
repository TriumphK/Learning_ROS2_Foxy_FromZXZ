#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class PointListener: public rclcpp::Node
{
public:
    PointListener(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a PointListener.");
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // CreateTimerROS(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base, rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers)
        timer_ = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        point_sub_.subscribe(this, "point");
        // MessageFilter(
        //  F & f, 
        //  BufferT & buffer, 
        //  const std::string & target_frame, uint32_t queue_size,
        //  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
        //  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
        //  std::chrono::duration<TimeRepT, TimeT> buffer_timeout = std::chrono::duration<TimeRepT, TimeT>::max())
        filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
            point_sub_,
            *buffer_,
            "base_link",
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            1s
        );
        // registerCallback(void(T::*callback)(P), T* t)
        filter_->registerCallback(&PointListener::transform_point, this);
    }

private:
    void transform_point(const geometry_msgs::msg::PointStamped::ConstSharedPtr ps)
    {
        // #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
        auto out = buffer_->transform(*ps, "base_link");
        RCLCPP_INFO(this->get_logger(), "%s: (%.2f,%.2f,%.2f)", out.header.frame_id.c_str(),
                                                                out.point.x,
                                                                out.point.y,
                                                                out.point.z);
    }

    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_;

    message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PointListener>("point_listener"));
    rclcpp::shutdown();
    return 0;
}