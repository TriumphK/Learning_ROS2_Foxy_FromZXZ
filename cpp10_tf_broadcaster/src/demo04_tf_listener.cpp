#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TFListener: public rclcpp::Node
{
public:
    TFListener(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a TFListener.");
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        timer_ = this->create_wall_timer(1s, std::bind(&TFListener::on_timer, this));
    }

private:
    void on_timer()
    {
        // auto ts = buffer_->lookupTransform("camera", "laser", tf2::TimePointZero);
        // RCLCPP_INFO(this->get_logger(), "%s to %s is: (%.2f,%.2f,%.2f).", ts.child_frame_id.c_str(), 
        //                                                                     ts.header.frame_id.c_str(),
        //                                                                     ts.transform.translation.x,
        //                                                                     ts.transform.translation.y,
        //                                                                     ts.transform.translation.z);
        try
        {
            auto ts = buffer_->lookupTransform("camera", "laser", tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "%s to %s is: (%.2f,%.2f,%.2f).", ts.child_frame_id.c_str(), 
                                                                              ts.header.frame_id.c_str(),
                                                                              ts.transform.translation.x,
                                                                              ts.transform.translation.y,
                                                                              ts.transform.translation.z);
        }
        catch(const tf2::LookupException& e)
        {
            // std::cerr << e.what() << '\n';
            RCLCPP_INFO(this->get_logger(), "Wrong: %s", e.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TFListener>("tf_listener"));
    rclcpp::shutdown();
    return 0;
}