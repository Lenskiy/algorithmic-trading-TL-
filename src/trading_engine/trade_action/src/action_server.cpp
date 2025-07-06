#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "system_interface/action/progress.hpp"
#include "system_interface/action/other_action.hpp"
#include <mutex>
#include <memory>

class ActionServer : public rclcpp::Node
{
public:
    explicit ActionServer(const std::string& name)
    : Node(name),
      progress_server(rclcpp_action::create_server<system_interface::action::Progress>(
        this,
        "progress_action",
        std::bind(&ActionServer::handle_progress_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ActionServer::handle_progress_cancel, this, std::placeholders::_1),
        std::bind(&ActionServer::handle_progress_accepted, this, std::placeholders::_1)
      )),
      other_server(rclcpp_action::create_server<system_interface::action::OtherAction>(
        this,
        "other_action",
        std::bind(&ActionServer::handle_other_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ActionServer::handle_other_cancel, this, std::placeholders::_1),
        std::bind(&ActionServer::handle_other_accepted, this, std::placeholders::_1)
      ))
    {
        RCLCPP_INFO(this->get_logger(), "ActionServer [%s] is running.", name.c_str());
    }
    

private:
    rclcpp_action::Server<system_interface::action::Progress>::SharedPtr progress_server;
    rclcpp_action::Server<system_interface::action::OtherAction>::SharedPtr other_server;
    std::mutex mutex_;

    rclcpp_action::GoalResponse handle_progress_goal(
        // The GoalUUID is used to distinguish between different goals,
        // especially when multiple clients are interacting with the same action server 
        // and potentially sending multiple, concurrent requests.
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const system_interface::action::Progress::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with data: %d", goal->num);
        if (goal->num <= 1)
        {
            RCLCPP_INFO(this->get_logger(), "Data must be greater than 1, rejecting goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_progress_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<system_interface::action::Progress>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<system_interface::action::Progress>> goal_handle)
    {
        auto feedback = std::make_shared<system_interface::action::Progress::Feedback>();
        auto result = std::make_shared<system_interface::action::Progress::Result>();
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        rclcpp::Rate rate(1);

        for (int i = 1; i <= num; i++)
        {
            sum += i;
            feedback->progress = static_cast<double>(i) / num;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Providing feedback: %.2f%% complete", feedback->progress * 100);

            if (goal_handle->is_canceling())
            {
                result->sum = sum;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled with partial sum: %ld", result->sum);
                return;
            }
            rate.sleep();
        }

        if (rclcpp::ok())
        {
            result->sum = sum;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded with total sum: %ld", result->sum);
        }
    }

    void handle_progress_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<system_interface::action::Progress>> goal_handle)
    {
        auto execute_function = [this, goal_handle]() { this->execute(goal_handle); };
        std::thread(execute_function).detach(); // Consider using a thread pool or other managed threading
    }


    // Handlers for 'OtherAction' action
    rclcpp_action::GoalResponse handle_other_goal(
        const rclcpp_action::GoalUUID &uuid2,
        std::shared_ptr<const system_interface::action::OtherAction::Goal> goal_other)
    {
        RCLCPP_INFO(this->get_logger(), "(Others)Received goal request with data: %ld", goal_other->num);
        if (goal_other->num <= 1)
        {
            RCLCPP_INFO(this->get_logger(), "(Others)Data must be greater than 1, rejecting goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_other_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<system_interface::action::OtherAction>>goal_other_handle)
    {
        RCLCPP_INFO(this->get_logger(), "(Others)Received request to cancel goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute_other(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<system_interface::action::OtherAction>> goal_other_handle)
    {
        auto feedback = std::make_shared<system_interface::action::OtherAction::Feedback>();
        auto result = std::make_shared<system_interface::action::OtherAction::Result>();
        int num = goal_other_handle->get_goal()->num;
        int sum = 0;
        rclcpp::Rate rate(1);

        for (int i = 1; i <= num; i++)
        {
            sum += i;
            feedback->progress = static_cast<double>(i) / num;
            goal_other_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "(Others)Providing feedback: %.2f%% complete", feedback->progress * 100);

            if (goal_other_handle->is_canceling())
            {
                result->sum = sum;
                goal_other_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "(Others)Goal canceled with partial sum: %ld", result->sum);
                return;
            }
            rate.sleep();
        }

        if (rclcpp::ok())
        {
            result->sum = sum;
            goal_other_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "(Others)Goal succeeded with total sum: %ld", result->sum);
        }
    }

    void handle_other_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<system_interface::action::OtherAction>> goal_other_handle)
    {
        auto execute_function_other = [this, goal_other_handle]() { this->execute_other(goal_other_handle); };
        std::thread(execute_function_other).detach(); // Consider using a thread pool or other managed threading
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionServer>("ActionServer");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}