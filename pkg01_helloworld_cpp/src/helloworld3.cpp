#include "rclcpp/rclcpp.hpp"

/*
    目前实现了ROS2中的单进程内的多节点生成，预期目的是能够实现高效通信
    主要参照文档：https://cloud.tencent.com/developer/article/2200676
*/

class MyNode: public rclcpp::Node
{
public:
    MyNode(const std::string & name, const std::string & output):Node(name) //Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        RCLCPP_INFO(this->get_logger(), output);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node1 = std::make_shared<MyNode>("test123", "++++++++ hello_123 +++++++");
    auto node2 = std::make_shared<MyNode>("test678", "++++++++ hello_678 +++++++");
    
    rclcpp::shutdown();
    return 0;
}
