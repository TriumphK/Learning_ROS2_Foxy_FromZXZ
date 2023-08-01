#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/message.hpp"

using namespace std::chrono_literals;

class Client: public rclcpp::Node
{
public:
    Client(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a client.");
        client_ = this->create_client<base_interfaces_demo::srv::Message>("addints");
    }

    bool connect_server()
    {
        while (!client_->wait_for_service(1s))
        {
            // CTRL + C detect
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Force Quit.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connecting ---");
        }
        return true;
    }

    rclcpp::Client<base_interfaces_demo::srv::Message>::SharedFuture Send_request(int num1, int num2)
    {
        auto request = std::make_shared<base_interfaces_demo::srv::Message::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<base_interfaces_demo::srv::Message>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);

    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Need TWO NUM!!!");
        return 1;
    }

    // rclcpp::spin(std::make_shared<Client>("node_client"));
    auto client = std::make_shared<Client>("node_client");

    //connecting +++++++
    bool flag = client->connect_server();
    if (!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed Connecting! -----");
        return 0;
    }
    auto future = client->Send_request(atoi(argv[1]), atoi(argv[2]));
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "Success! Sum = %d", future.get()->sum);
    }
    else
    {
        RCLCPP_INFO(client->get_logger(), "Failed ---");
    }
    

    rclcpp::shutdown();
    return 0;
}