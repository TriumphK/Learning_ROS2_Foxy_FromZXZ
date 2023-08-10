/* 
    THANKS: https://www.ncnynl.com/archives/202110/4623.html
    Now it can record by cpp-file using ros2bag successfully.
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

class BagWriter: public rclcpp::Node
{
public:
    BagWriter(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a BagWriter.");
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        writer_->open(storage_options_, converter_options_);
        // void create_topic(const rosbag2_storage::TopicMetadata & topic_with_type);
        writer_->create_topic(topic_with_type_);

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10, std::bind(&BagWriter::do_cb, this, std::placeholders::_1)); 
    }

private:
    // void do_cb(geometry_msgs::msg::Twist::SharedPtr twist)
    // {
    //     message->topic_name = "/turtle1/cmd_vel";
    //     message->time_stamp = this->now().nanoseconds();
    //     message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
        
    //     writer_->write(message);
    // }
    void do_cb(std::shared_ptr<rclcpp::SerializedMessage> twist)
    {
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t,
            [this](rcutils_uint8_array_t *msg) {
                auto fini_return = rcutils_uint8_array_fini(msg);
                delete msg;
                if (fini_return != RCUTILS_RET_OK) {
                    RCLCPP_ERROR(get_logger(),
                    "Failed to destroy serialized message %s", rcutils_get_error_string().str);
                }
            }
        );
        *bag_message->serialized_data = twist->release_rcl_serialized_message();
        bag_message->topic_name = "/turtle1/cmd_vel";
        if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
            RCLCPP_ERROR(get_logger(), "Error getting current time: %s", rcutils_get_error_string().str);
        }

        writer_->write(bag_message);
        RCLCPP_INFO(this->get_logger(), "---++++++---.");
    }
    
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr twist_sub_;
    const rosbag2_cpp::StorageOptions storage_options_{"src/cpp09_rosbag/new", "sqlite3"};
    const rosbag2_cpp::ConverterOptions converter_options_{rmw_get_serialization_format(), rmw_get_serialization_format()};
    const rosbag2_storage::TopicMetadata topic_with_type_{"/turtle1/cmd_vel", "geometry_msgs/msg/Twist", rmw_get_serialization_format(), ""};
    // std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<BagWriter>("bag_writer"));
    rclcpp::shutdown();
    return 0;
}
