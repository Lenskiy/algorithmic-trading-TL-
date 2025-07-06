#include "rclcpp/rclcpp.hpp"
#include "system_interface/msg/order.hpp"
#include "system_interface/msg/log.hpp"

class Accounting : public rclcpp::Node {
public:
    Accounting(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "The node has successfully launched : %s", name.c_str());

        callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        auto order_subscriber_options = rclcpp::SubscriptionOptions();
        order_subscriber_options.callback_group = callback_group_reentrant;

        auto confirmation_subscriber_options = rclcpp::SubscriptionOptions();
        confirmation_subscriber_options.callback_group = callback_group_reentrant;

        auto ce_subscriber_options = rclcpp::SubscriptionOptions();
        ce_subscriber_options.callback_group = callback_group_reentrant;

        order_subscriber = this->create_subscription<system_interface::msg::Order>(
            "agent_pool_order",
            100,
            std::bind(&Accounting::order_callback, this, std::placeholders::_1),
            order_subscriber_options
        );

        confirmation_subscriber = this->create_subscription<system_interface::msg::Order>(
            "approved_orders",
            100,
            std::bind(&Accounting::confirmation_callback, this, std::placeholders::_1),
            confirmation_subscriber_options
        );

        ce_subscriber = this->create_subscription<system_interface::msg::Order>(
            "ce_order",
            100,
            std::bind(&Accounting::ce_callback, this, std::placeholders::_1),
            ce_subscriber_options
        );

        order_publisher = this->create_publisher<system_interface::msg::Order>("store_order", 100);
        confirmation_publisher = this->create_publisher<system_interface::msg::Order>("store_order", 100);
        ce_publisher = this->create_publisher<system_interface::msg::Order>("store_order", 100);

        //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Accounting::update_callback, this), callback_group_reentrant);
    } 

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant;
    
    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr order_subscriber;
    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr confirmation_subscriber;
    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr ce_subscriber;

    rclcpp::Publisher<system_interface::msg::Order>::SharedPtr order_publisher;
    rclcpp::Publisher<system_interface::msg::Order>::SharedPtr confirmation_publisher;
    rclcpp::Publisher<system_interface::msg::Order>::SharedPtr ce_publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    void order_callback(const system_interface::msg::Order::SharedPtr order) {
        RCLCPP_INFO(this->get_logger(), "Received order");
        
        order_publisher->publish(*order);

        RCLCPP_INFO(this->get_logger(), "Orders from agent_pool have been sent to topic store_order");
    }

    void confirmation_callback(const system_interface::msg::Order::SharedPtr confirmation) {
        RCLCPP_INFO(this->get_logger(), "Received confirmation");
        
        order_publisher->publish(*confirmation);

        RCLCPP_INFO(this->get_logger(), "Orders from action_approval have been sent to topic store_order");
    }

    void ce_callback(const system_interface::msg::Order::SharedPtr ce_order) {
        RCLCPP_INFO(this->get_logger(), "Received ce_order");

        order_publisher->publish(*ce_order);

        RCLCPP_INFO(this->get_logger(), "Orders from ce have been sent to topic store_order");
    }

};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Accounting>("accounting");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
