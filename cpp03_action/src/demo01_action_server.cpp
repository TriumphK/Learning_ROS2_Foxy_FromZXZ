#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

class ProgressActionServer: public rclcpp::Node
{
public:
    ProgressActionServer(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "This is a ProgressActionServer.");
        /* 
          rclcpp_action::Server<ActionT>::SharedPtr 
          create_server<ActionT, NodeT>(NodeT node, const std::string &name, 
          rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
          rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
          rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted,
         */
        server_ = rclcpp_action::create_server<base_interfaces_demo::action::Progress>(this, "get_sum",
                                                                                        std::bind(&ProgressActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                                                        std::bind(&ProgressActionServer::handle_cancel, this, std::placeholders::_1),
                                                                                        std::bind(&ProgressActionServer::handle_accepted, this, std::placeholders::_1));
    }
private:
/* 
    using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
 */

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const base_interfaces_demo::action::Progress::Goal> goal)
    {
        (void)uuid;
        if (goal->num <= 1)
        {
            RCLCPP_INFO(this->get_logger(), " --- NEED NUM >1 ---  !!!");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Progress>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), " --- Cancel --- ");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Progress>> goal_handle)
    {
        //feedback
        //void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
        //goal_handle->publish_feedback();
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<base_interfaces_demo::action::Progress::Feedback>();
        auto result = std::make_shared<base_interfaces_demo::action::Progress::Result>();

        rclcpp::Rate rate(2.0);
        for (int i = 1; i <= num; i++)
        {
            sum += i;
            double progress = i / (double)num;
            feedback->progress = progress;

            RCLCPP_INFO(this->get_logger(), "Feedback: %.2f", progress);
            goal_handle->publish_feedback(feedback);
            
            if (goal_handle->is_canceling())
            {
                //void canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
                result->sum = sum;
                goal_handle->canceled(result);
                return;
            }

            rate.sleep();
        }
        
        //result
        //void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
        //goal_handle->succeed();
        if (rclcpp::ok())
        {
            result->sum = sum;
            RCLCPP_INFO(this->get_logger(), "Result: %d", sum);
            goal_handle->succeed(result);
        }
        
    }

    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Progress>> goal_handle)
    {
        // new thread
        std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle)).detach();
    }

    rclcpp_action::Server<base_interfaces_demo::action::Progress>::SharedPtr server_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ProgressActionServer>("action_server"));
    rclcpp::shutdown();
    return 0;
}