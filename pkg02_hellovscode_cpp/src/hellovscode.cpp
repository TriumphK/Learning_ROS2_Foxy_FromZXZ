#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hellovscode_cpp_node");

  RCLCPP_INFO(node->get_logger(), "hello vscode!");

  rclcpp::shutdown();
  
  return 0;
}
