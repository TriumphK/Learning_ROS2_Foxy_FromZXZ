#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using namespace std::chrono_literals;

class TalkerStu: public rclcpp::Node
{
public:
    TalkerStu():Node("talkerstu_node_cpp"),age(0)
    {
        publisher_ = this->create_publisher<base_interfaces_demo::msg::Student>("chatter_stu",10);
        timer_ = this->create_wall_timer(500ms,std::bind(&TalkerStu::on_timer,this));
    }
private:
    void on_timer()
    {
        auto stu = base_interfaces_demo::msg::Student();
        stu.age = age;
        stu.height = 175.2;
        stu.name = "xiaoming";
        age++;
        publisher_->publish(stu);
        RCLCPP_INFO(this->get_logger(), "Student Info: name %s, age %d, height %f",
                                        stu.name.c_str(),stu.age,stu.height);
    }
    rclcpp::Publisher<base_interfaces_demo::msg::Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int age;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TalkerStu>());
    rclcpp::shutdown();
    return 0;
}