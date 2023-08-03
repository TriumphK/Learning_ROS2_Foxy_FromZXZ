#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient: public rclcpp::Node
{
public:
    ParamClient(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ParamClient.");
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "param_server");
    }
    
    bool connect_server()
    {
        while (!param_client_->wait_for_service(2s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "--- Connecting ---");
        }
        
        return true;
    }

    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "------ Get Param ------");
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        RCLCPP_INFO(this->get_logger(), "car_name %s", car_name.c_str());
        double width = param_client_->get_parameter<double>("width");
        RCLCPP_INFO(this->get_logger(), "width %.2f", width);

        auto params = param_client_->get_parameters({"car_name", "wheel"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "(%s == %s)", param.get_name().c_str(), param.value_to_string().c_str());;
        }
        
        RCLCPP_INFO(this->get_logger(), "Y or N? (car_name)  %d", param_client_->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "Y or N? (length)      %d", param_client_->has_parameter("length"));
    }

    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "------ Update Param ------");
        param_client_->set_parameters({rclcpp::Parameter("car_name", "xiaoli"),
                                       rclcpp::Parameter("wheel", 3),
                                       rclcpp::Parameter("length", 1.2) //rclcpp::NodeOptions().allow_undeclared_parameters(true)
        });
    }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto client = std::make_shared<ParamClient>("param_client");
    bool flag = client->connect_server();
    if (!flag)
    {
        return 0;
    }
    client->get_param();
    client->update_param();
    client->get_param();

    rclcpp::shutdown();
    return 0;
}