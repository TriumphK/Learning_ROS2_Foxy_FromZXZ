#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ActionServer: public rclcpp::Node
{
public:
    ActionServer(const std::string& name):Node(name),x(0.0),y(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ActionServer.");
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&ActionServer::pose_cb, this, std::placeholders::_1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        /* 
            rclcpp_action::Server<ActionT>::SharedPtr create_server<ActionT, NodeT>
            (NodeT node, const std::string &name, 
            rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
            rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
            rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
            const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
            rclcpp::CallbackGroup::SharedPtr group = nullptr)
         */
        action_server_ = rclcpp_action::create_server<base_interfaces_demo::action::Nav>(this, 
                                                                                         "nav",
                                                                                         std::bind(&ActionServer::handle_goal, this, std::placeholders::_1,std::placeholders::_2),
                                                                                         std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
                                                                                         std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    void pose_cb(const turtlesim::msg::Pose::ConstSharedPtr pose)
    {
        x = pose->x;
        y = pose->y;
    }

    //GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const base_interfaces_demo::action::Nav::Goal> goal)
    {
        (void)uuid;
        if (goal->goal_x < 0 || goal->goal_x > 11.08 || goal->goal_y < 0 || goal->goal_y > 11.08)
        {
            RCLCPP_INFO(this->get_logger(), "X Y of goal is invalid!!!");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Target point effective.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    //CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Nav>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), " --- Cancel --- ");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Nav>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), " --- Main --- ");
        auto feedback = std::make_shared<base_interfaces_demo::action::Nav::Feedback>();
        auto result = std::make_shared<base_interfaces_demo::action::Nav::Result>();
        geometry_msgs::msg::Twist twist;

        rclcpp::Rate rate(1.0);
        while (true)
        {
            // handle_cancel --- accept
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                return;
            }
            
            float goal_x = goal_handle->get_goal()->goal_x;
            float goal_y = goal_handle->get_goal()->goal_y;

            float distance_x = goal_x - x;
            float distance_y = goal_y - y;
            float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
            feedback->distance = distance;
            goal_handle->publish_feedback(feedback);

            float scale = 0.5;
            float linear_x = scale * distance_x;
            float linear_y = scale * distance_y;
            twist.linear.x = linear_x;
            twist.linear.y = linear_y;
            twist_pub_->publish(twist);
            
            // when over
            if (distance <= 0.05)
            {
                break;
            }

            rate.sleep();
        }

        if (rclcpp::ok())
        {
            result->turtle_x = x;
            result->turtle_y = y;
            goal_handle->succeed(result);
        }
    }

    //void (std::shared_ptr<ServerGoalHandle<ActionT>>)
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Nav>> goal_handle)
    {
        std::thread(std::bind(&ActionServer::execute, this, goal_handle)).detach();
    }

    float x, y;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp_action::Server<base_interfaces_demo::action::Nav>::SharedPtr action_server_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ActionServer>("action_server"));
    rclcpp::shutdown();
    return 0;
}