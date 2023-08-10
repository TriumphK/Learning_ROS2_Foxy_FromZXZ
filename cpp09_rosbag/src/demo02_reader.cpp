#include "rclcpp/rclcpp.hpp"

class BagReader: public rclcpp::Node
{
public:
    BagReader(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a BagReader.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<BagReader>("bag_reader"));
    rclcpp::shutdown();
    return 0;
}