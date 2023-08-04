#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode(const std::string& name):Node(name, "t1_ns")
    {
        RCLCPP_INFO(this->get_logger(), "This is a description.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MyNode>("node_name"));
    rclcpp::shutdown();
    return 0;
}