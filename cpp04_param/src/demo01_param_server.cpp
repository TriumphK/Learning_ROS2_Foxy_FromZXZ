#include "rclcpp/rclcpp.hpp"

class ParamServer: public rclcpp::Node
{
public:
    // rclcpp::Node::Node
    //      (const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    ParamServer(const std::string& name):Node(name, rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
        RCLCPP_INFO(this->get_logger(), "This is a ParamServer.");
    }

    void declare_param()
    {
        RCLCPP_INFO(this->get_logger(), "------ declare ------");
        this->declare_parameter("car_name", "laowang");
        this->declare_parameter("width", 2.34);
        this->declare_parameter("wheel", 5); //bei tao

        //rclcpp::NodeOptions().allow_undeclared_parameters(true)
        this->set_parameter(rclcpp::Parameter("age", 12));
    }

    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "------ get ------");
        auto name_get = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "Key=%s, Value=%s.", name_get.get_name().c_str(), name_get.as_string().c_str());
        auto params = this->get_parameters({"car_name", "width"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "(%s = %s)", param.get_name().c_str(), param.value_to_string().c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Y or N? (wheel) %d", this->has_parameter("wheel"));
        RCLCPP_INFO(this->get_logger(), "Y or N? (tree)  %d", this->has_parameter("tree"));
        ;
    }

    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "------ update ------");
        this->set_parameter(rclcpp::Parameter("width", 3.2));
    }

    void del_param()
    {
        RCLCPP_INFO(this->get_logger(), "------ del ------");
        // can not delete the declared param
        //this->undeclare_parameter("car_name");
        //RCLCPP_INFO(this->get_logger(), "Y or N? (\"car_name\")  %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "Before Y or N? (\"age\")  %d", this->has_parameter("age"));
        this->undeclare_parameter("age");
        RCLCPP_INFO(this->get_logger(), "After Y or N? (\"age\")  %d", this->has_parameter("age"));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ParamServer>("param_server");
    node->declare_param();
    node->get_param();
    node->update_param();
    node->del_param();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}