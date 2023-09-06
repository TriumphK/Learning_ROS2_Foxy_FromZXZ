#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class Spawn: public rclcpp::Node
{
public:
    Spawn(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a Spawn.");
        this->declare_parameter("x", 3.0);
        this->declare_parameter("y", 3.0);
        this->declare_parameter("theta", 1.57);
        this->declare_parameter("turtle_name", "turtle2");
        x = this->get_parameter("x").as_double();
        y = this->get_parameter("y").as_double();
        theta = this->get_parameter("theta").as_double();
        turtle_name = this->get_parameter("turtle_name").as_string();

        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    }

    bool connect_server()
    {
        while (!spawn_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--- Force Quit ---");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "--- connecting ---");
        }
        return true;
    }

    // rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture async_send_request(std::shared_ptr<turtlesim::srv::Spawn_Request> request)
    rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture send_request()
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = turtle_name;
        return spawn_client_->async_send_request(request);
    }

private:
    double_t x, y, theta;
    std::string turtle_name;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto client = std::make_shared<Spawn>("spawn_new");
    bool flag = client->connect_server();
    if (!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--- Connect Failed ---");
        return 1;
    }

    auto response = client->send_request();
    if (rclcpp::spin_until_future_complete(client, response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "--- Success ---");
        // make sure the name of the turtle is different
        std::string name = response.get()->name;
        if (name.empty())
        {
            RCLCPP_INFO(client->get_logger(), "--- Failed: Name is the same. ---");
        }
        
    }
    else
    {
        RCLCPP_INFO(client->get_logger(), "--- Failed ---");
    }
    
    
    rclcpp::shutdown();
    return 0;
}