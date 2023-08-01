#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Pub: public rclcpp::Node
{
public:
    Pub(const std::string& name):Node(name),count(0)
    {
        RCLCPP_INFO(this->get_logger(), "this is a pub.");
        publisher_ = this->create_publisher<std_msgs::msg::String>("liaotian", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&Pub::on_timer, this));
    }
private:
    void on_timer()
    {
        auto message = std_msgs::msg::String();
        message.data = "hello world!" + std::to_string(count++);
        RCLCPP_INFO(this->get_logger(), "Pub Message is: %s", message.data.c_str());
        publisher_->publish(message);
    }

    size_t count;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub>("demo_pub"));

    rclcpp::shutdown();
    
    return 0;
}
