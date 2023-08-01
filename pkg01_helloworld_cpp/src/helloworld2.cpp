#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode():Node("hello_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "hello_class_cpp!");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MyNode>();
    
    rclcpp::shutdown();
    return 0;
}
