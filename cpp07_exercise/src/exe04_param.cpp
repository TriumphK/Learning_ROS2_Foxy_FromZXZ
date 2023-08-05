#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DynaParam: public rclcpp::Node
{
public:
    DynaParam(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a DynaParam.");
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
    }

    bool connect_server()
    {
        while (!param_client_->wait_for_service(3s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--- Force Quit ---");
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "---Connecting---");
        }
        
        return true;
    }

    void update_param()
    {
        // background_r [0 -255]
        int red = param_client_->get_parameter<int>("background_r");
        int flag = 1;
        rclcpp::Rate rate(30.0);
        while (rclcpp::ok())
        {
            if (red < 0 || red > 255)
            {
                flag *= -1;
            }
            red += 5 * flag;
            param_client_->set_parameters({rclcpp::Parameter("background_r", red)});
            
            rate.sleep();
        }
    }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto client = std::make_shared<DynaParam>("dyna_param");
    if (!client->connect_server())
    {
        return 1;
    }
    client->update_param();

    rclcpp::shutdown();
    return 0;
}