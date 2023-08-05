#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using namespace std::chrono_literals;

class ExeClient: public rclcpp::Node
{
public:
    ExeClient(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ExeClient.");
        client_ = this->create_client<base_interfaces_demo::srv::Distance>("distance");
    }

    bool connect_server()
    {
        while (!client_->wait_for_service(2s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---XXX@@@XXX---");
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "---Connecting---");
            return false;
        }
        
        return true;
    }

    rclcpp::Client<base_interfaces_demo::srv::Distance>::SharedFuture send_goal(float x, float y, float theta)
    {
        // std::shared_future<std::shared_ptr<base_interfaces_demo::srv::Distance_Response>>
        // async_send_request(std::shared_ptr<base_interfaces_demo::srv::Distance_Request> request)
        auto request = std::make_shared<base_interfaces_demo::srv::Distance_Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<base_interfaces_demo::srv::Distance>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Give: x y theta Please!");
        return 1;
    }

    float goal_x = atof(argv[1]);
    float goal_y = atof(argv[2]);
    float goal_theta = atof(argv[3]);
    
    rclcpp::init(argc,argv);
    auto client = std::make_shared<ExeClient>("exe_client");
    bool flag = client->connect_server();
    if (!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---Connected Failed!!!---");
        return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---Connected Success!!!---");

    auto future = client->send_goal(goal_x, goal_y, goal_theta);
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "Success! Distance = %.2f", future.get()->distance);
    }
    else
    {
        RCLCPP_INFO(client->get_logger(), "Failed ---");
    }

    rclcpp::shutdown();
    return 0;
}