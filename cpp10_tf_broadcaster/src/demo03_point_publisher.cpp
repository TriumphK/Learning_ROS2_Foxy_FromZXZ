#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class PointPublisher: public rclcpp::Node
{
public:
    PointPublisher(const std::string& name):Node(name),x(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "This is a PointPublisher.");
        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&PointPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::PointStamped ps;
        ps.header.stamp = this->now();
        ps.header.frame_id = "laser";
        x += 0.05;
        ps.point.x = x;
        ps.point.y = 0.0;
        ps.point.z = 0.5;

        point_pub_->publish(ps);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PointPublisher>("point_publisher"));
    rclcpp::shutdown();
    return 0;
}