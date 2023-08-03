#include "rclcpp/rclcpp.hpp"

class MyParam: public rclcpp::Node
{
public:
    MyParam(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a param using.");
        rclcpp::Parameter p1("car_name", "tiger");
        rclcpp::Parameter p2("height", 1.68);
        rclcpp::Parameter p3("wheel", 4);

        //value
        RCLCPP_INFO(this->get_logger(), "car name = %s", p1.as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "height = %.2f", p2.as_double());
        RCLCPP_INFO(this->get_logger(), "wheel = %ld", p3.as_int());

        //key
        RCLCPP_INFO(this->get_logger(), "name = %s", p1.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "type = %s", p2.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "value2string = %s", p3.value_to_string().c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MyParam>("param_test"));
    rclcpp::shutdown();
    return 0;
}