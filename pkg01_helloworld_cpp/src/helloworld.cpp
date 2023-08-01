#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("hello_vscode_cpp");
    RCLCPP_INFO(node->get_logger(),"---hello_vscode_cpp!---");

    rclcpp::shutdown();
    return 0;
}
