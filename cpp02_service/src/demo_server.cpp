#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/message.hpp"

class Server: public rclcpp::Node
{
public:
    Server(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a server.");
        service_ = this->create_service<base_interfaces_demo::srv::Message>("addints", std::bind(&Server::add, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void add(const base_interfaces_demo::srv::Message::Request::SharedPtr req, const base_interfaces_demo::srv::Message::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Request is: %d,%d",req->num1,req->num2);
        res->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(), "Response is: %d",res->sum);
    }

    rclcpp::Service<base_interfaces_demo::srv::Message>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Server>("node_server"));
    rclcpp::shutdown();
    return 0;
}