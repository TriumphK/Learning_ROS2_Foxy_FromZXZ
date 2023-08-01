#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

class AddIntsServer: public rclcpp::Node
{
public:
    AddIntsServer():Node("addints_server_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Service Server created!");
        server_ = this->create_service<base_interfaces_demo::srv::AddInts>("add_ints",std::bind(&AddIntsServer::add,this,std::placeholders::_1,std::placeholders::_2));
    }
private:
    rclcpp::Service<base_interfaces_demo::srv::AddInts>::SharedPtr server_;
    void add(const base_interfaces_demo::srv::AddInts::Request::SharedPtr req, const base_interfaces_demo::srv::AddInts::Response::SharedPtr res)
    {
        res->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", req->num1,req->num2,res->sum);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<AddIntsServer>());
    rclcpp::shutdown();
    return 0;
}
