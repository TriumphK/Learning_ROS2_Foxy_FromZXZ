#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/person.hpp"

using namespace std::chrono_literals;

class PubPerson: public rclcpp::Node
{
public:
    PubPerson(const std::string& name):Node(name),age(0)
    {
        RCLCPP_INFO(this->get_logger(), "This is a pub of Person.");
        publisher_ = this->create_publisher<base_interfaces_demo::msg::Person>("chatter_person", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&PubPerson::on_timer, this));
    }
private:
    void on_timer()
    {
        auto person = base_interfaces_demo::msg::Person();
        person.name = "zhang";
        person.age = age;
        person.height = 172.3;
        RCLCPP_INFO(this->get_logger(), "Pub Person Message:%s,age=%d,height=%.2f",person.name.c_str(),person.age,person.height);
        publisher_->publish(person);
        age++;
    }

    int age;
    rclcpp::Publisher<base_interfaces_demo::msg::Person>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PubPerson>("node_pub_person"));
    rclcpp::shutdown();
    return 0;
}