#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "system_interface/action/progress.hpp" 
#include "system_interface/action/other_action.hpp"

template<typename ActionType>
class ActionClient : public rclcpp::Node {
public:

    ActionClient(const std::string& name, const std::string& action_name, int goal_num_init)
    : Node(name), client(rclcpp_action::create_client<ActionType>(this, action_name)), 
      goal_num(goal_num_init)
    {
        RCLCPP_INFO(this->get_logger(), "ActionClient [%s] is running.", action_name.c_str());
        send_goal(goal_num_init);
    }

    void send_goal(int num)
    {
        // Ensure the action server is available
        if (!client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Setup the goal message with correct goal type
        auto goal = typename ActionType::Goal();
        goal.num = num; 

        // Define goal options with correct ActionType
        auto options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
        options.goal_response_callback = std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
        options.feedback_callback = std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        options.result_callback = std::bind(&ActionClient::result_callback, this, std::placeholders::_1);

        // Send the goal
        auto goal_handle_future = client->async_send_goal(goal, options);
        if (!goal_handle_future.valid()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
        }
    }

    // Generalized callbacks using ActionType
    void goal_response_callback(const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr& /* goal_handle */,
        const std::shared_ptr<const typename ActionType::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current progress: %.2f%%", feedback->progress * 100);
    }

    void result_callback(const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "The end result is: %ld", result.result->sum);  // Adjust accessing sum as per ActionType
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown result status");
                break;
        }
    }

private:
    typename rclcpp_action::Client<ActionType>::SharedPtr client;
    int goal_num;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    std::vector<std::shared_ptr<rclcpp::Node>> clients;
    
    clients.push_back(std::make_shared<ActionClient<system_interface::action::Progress>>("Client1", "progress_action", 10));
    clients.push_back(std::make_shared<ActionClient<system_interface::action::Progress>>("Client2", "progress_action", 20));
    clients.push_back(std::make_shared<ActionClient<system_interface::action::OtherAction>>("Client3", "other_action", 11));

    
    // Add all nodes to the executor
    for (auto& client : clients) {
        executor->add_node(client);
    }

    // Use the multi-threaded executor to spin
    executor->spin();
    rclcpp::shutdown();
    return 0;
}

