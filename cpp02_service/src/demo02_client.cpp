#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using namespace std::chrono_literals;

class AddIntsClient: public rclcpp::Node
{
public:
    AddIntsClient():Node("addints_client_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Service Client created!");
        client_ = this->create_client<base_interfaces_demo::srv::AddInts>("add_ints");
    }

    bool connect_server()
    {
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---Quit---");
                return false;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---Waiting---");
        }
        return true;
    }
    //rclcpp::Client<base_interfaces_demo::srv::AddInts>::SharedFuture 
    //async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request> request)
    rclcpp::Client<base_interfaces_demo::srv::AddInts>::SharedFuture send_request(int num1, int num2)
    {
        auto request = std::make_shared<base_interfaces_demo::srv::AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<base_interfaces_demo::srv::AddInts>::SharedPtr client_;
};

int main(int argc, char *argv[])
{

    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Need three argc...");
        return 1;
    }

    rclcpp::init(argc,argv);
    auto client = std::make_shared<AddIntsClient>();

    bool flag = client->connect_server();
    if (!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Connect link failed!!!");
        return 0;
    }
    
    auto future = client->send_request(atoi(argv[1]),atoi(argv[2]));
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "SUCCESS!!!");
        RCLCPP_INFO(client->get_logger(), "sum = %d",future.get()->sum);
    }
    else
    {
        RCLCPP_INFO(client->get_logger(), "FAILED!!!");
    }
    

    rclcpp::shutdown();
    return 0;
}
