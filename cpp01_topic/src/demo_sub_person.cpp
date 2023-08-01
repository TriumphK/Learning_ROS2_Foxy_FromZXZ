#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/person.hpp"

class SubPerson: public rclcpp::Node
{
public:
    SubPerson(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a sub of Person.");
        subscriber_ = this->create_subscription<base_interfaces_demo::msg::Person>("chatter_person", 10, std::bind(&SubPerson::do_cb, this, std::placeholders::_1));
    }
private:
    void do_cb(const base_interfaces_demo::msg::Person::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Sub Person Messgae: %s,%d,%.2f",msg->name.c_str(),msg->age,msg->height);
    }

    rclcpp::Subscription<base_interfaces_demo::msg::Person>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SubPerson>("node_sub_person"));
    rclcpp::shutdown();
    return 0;
}