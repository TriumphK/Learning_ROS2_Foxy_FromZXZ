#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

class ExeServer: public rclcpp::Node
{
public:
    ExeServer(const std::string& name):Node(name),x(0.0),y(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ExeServer.");

        sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, 
                                                                std::bind(&ExeServer::pose_cb, this, std::placeholders::_1));
        server_ = this->create_service<base_interfaces_demo::srv::Distance>("distance", 
                                                                std::bind(&ExeServer::distance_cb, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void pose_cb(const turtlesim::msg::Pose::ConstSharedPtr pose)
    {
        x = pose->x;
        y = pose->y;
    }

    void distance_cb(base_interfaces_demo::srv::Distance::Request::SharedPtr request,
                     base_interfaces_demo::srv::Distance::Response::SharedPtr response)
    {
        float goal_x = request->x;
        float goal_y = request->y;
        float distance_x = goal_x - x; 
        float distance_y = goal_y - y;

        float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
        response->distance = distance;
        RCLCPP_INFO(this->get_logger(), "The distance is: %.2f.", distance);
    }

    float x, y;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Service<base_interfaces_demo::srv::Distance>::SharedPtr server_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ExeServer>("exe_server"));
    rclcpp::shutdown();
    return 0;
}