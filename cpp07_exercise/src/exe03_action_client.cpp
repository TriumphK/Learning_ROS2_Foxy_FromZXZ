#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using namespace std::chrono_literals;

class ActionClient: public rclcpp::Node
{
public:
    ActionClient(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ActionClient.");
        // rclcpp_action::Client<ActionT>::SharedPtr create_client<ActionT, NodeT>(
        //    NodeT node, const std::string &name, rclcpp::CallbackGroup::SharedPtr group = nullptr)
        action_client_ = rclcpp_action::create_client<base_interfaces_demo::action::Nav>(this, "nav");
    }

    void send_goal(float x, float y, float theta)
    {
        if (!action_client_->wait_for_action_server(5s))
        {
            RCLCPP_INFO(this->get_logger(), "Over Time---");
            return;
        }

        // std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>::SharedPtr> async_send_goal
        //  (const base_interfaces_demo::action::Nav::Goal &goal, 
        //  const rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions &options)
        base_interfaces_demo::action::Nav::Goal goal;
        rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions options;

        goal.goal_x = x;
        goal.goal_y = y;
        goal.goal_theta = theta;

        // SendGoalOptions(): goal_response_callback(nullptr), feedback_callback(nullptr), result_callback(nullptr)
        // std::function<void (std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>>>)>
        options.goal_response_callback = std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
        // std::function<void (std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>>, std::shared_ptr<const base_interfaces_demo::action::Nav_Feedback>)>
        options.feedback_callback = std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        // std::function<void (const rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>::WrappedResult &result)>
        options.result_callback = std::bind(&ActionClient::result_callback, this, std::placeholders::_1);
        
        action_client_->async_send_goal(goal, options);
    }

private:
    void goal_response_callback(std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>>> goal_handle)
    {
        if (!goal_handle.get())
        {
            RCLCPP_INFO(this->get_logger(), "REJECTED");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "ACCEPTED");
        }  
    }

    void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>> goal_handle, std::shared_ptr<const base_interfaces_demo::action::Nav::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Current: ---%.2f---", feedback->distance);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Success. NOW is: (%.2f,%.2f,%.2f)", result.result->turtle_x, result.result->turtle_y, result.result->turtle_theta);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Failed++++----");
        } 
    }

    rclcpp_action::Client<base_interfaces_demo::action::Nav>::SharedPtr action_client_;
};

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Please Input the effective goal.");
        return 1;
    }
    
    rclcpp::init(argc,argv);
    auto client = std::make_shared<ActionClient>("action_client");
    client->send_goal(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    rclcpp::spin(client);

    rclcpp::shutdown();
    return 0;
}