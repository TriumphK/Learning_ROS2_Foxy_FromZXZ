#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TimerNode: public rclcpp::Node
{
public:
    TimerNode(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a timer node test.");
        //demo_rate();
        // demo_time();
        // demo_duration();
        demo_opt();
    }
private:
    void demo_rate()
    {
        rclcpp::Rate rate1(100ms); // time
        rclcpp::Rate rate2(1.0); // hz
        while (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "------ counting ------");
            rate1.sleep();
        } 
    }

    void demo_time()
    {
        rclcpp::Time t1(500000000L);
        rclcpp::Time t2(2, 500000000L);
        rclcpp::Time right_now = this->now();

        RCLCPP_INFO(this->get_logger(), "t1: s = %.2f, ns = %ld", t1.seconds(), t1.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "t2: s = %.2f, ns = %ld", t2.seconds(), t2.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "right_now: s = %.2f, ns = %ld", right_now.seconds(), right_now.nanoseconds());
    }

    void demo_duration()
    {
        rclcpp::Duration du1(2s);
        rclcpp::Duration du2(2, 500000000L);

        RCLCPP_INFO(this->get_logger(), "du1: s = %.2f, ns = %ld", du1.seconds(), du1.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "du2: s = %.2f, ns = %ld", du2.seconds(), du2.nanoseconds());
    }

    void demo_opt()
    {
        rclcpp::Time t1(10, 0);
        rclcpp::Time t2(30, 0);
        rclcpp::Duration du1(8, 0);
        rclcpp::Duration du2(15, 0);

        RCLCPP_INFO(this->get_logger(), "t1 >= t2: %d", t1 >= t2);
        RCLCPP_INFO(this->get_logger(), "t1 < t2: %d", t1 < t2);

        rclcpp::Duration du3 = t2 -t1;
        RCLCPP_INFO(this->get_logger(), "du3: s = %.2f.", du3.seconds());

        rclcpp::Time t3 = t2 + du2;
        rclcpp::Time t4 = t1 - du1;
        RCLCPP_INFO(this->get_logger(), "t3: s = %.2f.", t3.seconds());
        RCLCPP_INFO(this->get_logger(), "t4: s = %.2f.", t4.seconds());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TimerNode>("timer_node");

    rclcpp::shutdown();
    return 0;
}