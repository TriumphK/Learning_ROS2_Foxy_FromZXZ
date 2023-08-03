#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using namespace std::chrono_literals;

class ProgressActionClient: public rclcpp::Node
{
public:
    ProgressActionClient(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ProgressActionClient.");
        //rclcpp_action::Client<ActionT>::SharedPtr create_client<ActionT, NodeT>
        //(NodeT node,
        //const std::string &name, 
        //rclcpp::CallbackGroup::SharedPtr group = nullptr)
        client_ = rclcpp_action::create_client<base_interfaces_demo::action::Progress>(this, "get_sum");
    }

    void send_goal(int num)
    {
        if (!client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Long Time Waste...");
            return;
        }
        //std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr> 
        //async_send_goal(
        //    const base_interfaces_demo::action::Progress::Goal &goal,
        //    const rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions &options)
        auto goal = base_interfaces_demo::action::Progress::Goal();
        rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions option;
        option.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, std::placeholders::_1);
        option.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        option.result_callback = std::bind(&ProgressActionClient::result_callback, this, std::placeholders::_1);
        goal.num = num;
        auto future = client_->async_send_goal(goal, option);
    }

private:
    //using GoalHandle = ClientGoalHandle<ActionT>;
    //using GoalResponseCallback = std::function<void (std::shared_future<typename GoalHandle::SharedPtr>)>;
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr> goal_handle)
    {
        if (!goal_handle.get())
        {
            RCLCPP_INFO(this->get_logger(), "Request Rejected!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Request Accepted!");
        }
        
    }

    //FeedbackCallback =std::function<void (typename ClientGoalHandle<ActionT>::SharedPtr, const std::shared_ptr<const Feedback>)>;
    void feedback_callback(rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr goal_handle, const std::shared_ptr<const base_interfaces_demo::action::Progress::Feedback> feedback)
    {
        (void)goal_handle;
        double progress = feedback->progress;
        RCLCPP_INFO(this->get_logger(), "Current: ---%.2f---", progress);
    }

    //ResultCallback = std::function<void (const WrappedResult & result)>;
    //using WrappedResult = typename GoalHandle::WrappedResult;
    void result_callback(const rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::WrappedResult & result)
    {
        // result.code
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Success. Result is: %d", result.result->sum);
        }
        else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(), "Aborted.");
        }
        else if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(), "Cancled.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Unknown.");
        }
        
        
        
    }

    rclcpp_action::Client<base_interfaces_demo::action::Progress>::SharedPtr client_;
    
};

int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Need one int number.");
        return 1;
    }

    rclcpp::init(argc,argv);
    auto node = std::make_shared<ProgressActionClient>("action_client");
    node->send_goal(atoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}