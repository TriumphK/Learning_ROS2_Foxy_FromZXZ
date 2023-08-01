#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

class ListenerStu: public rclcpp::Node
{
public:
    ListenerStu():Node("listenerstu_node_cpp")
    {
        subscription_ = this->create_subscription<base_interfaces_demo::msg::Student>("chatter_stu",10,std::bind(&ListenerStu::do_cb,this,std::placeholders::_1));
    }
private:
    void do_cb(const base_interfaces_demo::msg::Student::SharedPtr stu)
    {
        RCLCPP_INFO(this->get_logger(), "From /chatter_stu: %s,%d,%f",
                                        stu->name.c_str(),stu->age,stu->height);
    }
    rclcpp::Subscription<base_interfaces_demo::msg::Student>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ListenerStu>());
    rclcpp::shutdown();
    return 0;
}