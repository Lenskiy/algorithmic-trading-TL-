#include "rclcpp/rclcpp.hpp"
#include "system_interface/msg/log.hpp"
#include "system_interface/msg/order.hpp"
#include "system_interface/msg/log_array.hpp"

class Logging : public rclcpp::Node{
public:
    Logging(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "The node has successfully launched : %s", name.c_str());

        callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        auto agent_pool_topic_logging_subscriber_options = rclcpp::SubscriptionOptions();
        agent_pool_topic_logging_subscriber_options.callback_group = callback_group_reentrant;

        auto approved_orders_subscriber_options = rclcpp::SubscriptionOptions();
        approved_orders_subscriber_options.callback_group = callback_group_reentrant; 

        auto logging_subscriber_options = rclcpp::SubscriptionOptions();
        logging_subscriber_options.callback_group = callback_group_reentrant; 

        //agent pool
        agent_pool_topic_logging_subscriber = this->create_subscription<system_interface::msg::Order>(
            "agent_pool_order",
            100,
            std::bind(&Logging::agent_pool_topic_logging_callback, this, std::placeholders::_1),
            agent_pool_topic_logging_subscriber_options
        );

        //action approval
        approved_orders_subscriber = this->create_subscription<system_interface::msg::Order>(
            "approved_orders",
            100,
            std::bind(&Logging::approved_orders_callback, this, std::placeholders::_1),
            approved_orders_subscriber_options
        );

        //service
        logging_subscriber = this->create_subscription<system_interface::msg::Log>(
            "logging",
            100,
            std::bind(&Logging::logging_callback, this, std::placeholders::_1),
            logging_subscriber_options
        );

        log_publisher = this->create_publisher<system_interface::msg::LogArray>("store_log", 100);

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Logging::publish_log_callback, this), callback_group_reentrant);
    }


private:
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant;

    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr agent_pool_topic_logging_subscriber;
    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr approved_orders_subscriber;

    rclcpp::Subscription<system_interface::msg::Log>::SharedPtr logging_subscriber;
    
    rclcpp::Publisher<system_interface::msg::LogArray>::SharedPtr log_publisher;

    system_interface::msg::LogArray log_array;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mtx_log_array;

    void agent_pool_topic_logging_callback(system_interface::msg::Order::SharedPtr order_info){
        system_interface::msg::Log tmp_log;

        auto now = std::chrono::system_clock::now();
        int timestamp = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
        tmp_log.time = timestamp;

        tmp_log.event = "agent_pool_order";

        std::string tmp_order_info = "symbol: " + order_info->symbol
                                   + ", exchange: " + order_info->exchange
                                   + ", price: " + std::to_string(order_info->price)
                                   + ", size: " + std::to_string(order_info->size)
                                   + ", side: " + order_info->side
                                   + ", time: " + std::to_string(order_info->time);
        tmp_log.info = tmp_order_info;
        
        std::lock_guard<std::mutex> guard(mtx_log_array);

        log_array.log_array.push_back(tmp_log);

        RCLCPP_INFO(this->get_logger(), "The log has been processed into the appropriate format and temporarily stored in log_array.");
    }

    void approved_orders_callback(system_interface::msg::Order::SharedPtr order_info) {
        system_interface::msg::Log tmp_log;

        auto now = std::chrono::system_clock::now();
        int timestamp = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
        tmp_log.time = timestamp;

        tmp_log.event = "approved_orders";

        std::string tmp_order_info = "symbol: " + order_info->symbol
                                   + ", exchange: " + order_info->exchange
                                   + ", price: " + std::to_string(order_info->price)
                                   + ", size: " + std::to_string(order_info->size)
                                   + ", side: " + order_info->side
                                   + ", time: " + std::to_string(order_info->time);
        tmp_log.info = tmp_order_info;
        
        std::lock_guard<std::mutex> guard(mtx_log_array);

        log_array.log_array.push_back(tmp_log);

        RCLCPP_INFO(this->get_logger(), "The log has been processed into the appropriate format and temporarily stored in log_array.");
    }

    void logging_callback(system_interface::msg::Log::SharedPtr log) {
        std::lock_guard<std::mutex> guard(mtx_log_array);

        log_array.log_array.push_back(*log);

        RCLCPP_INFO(this->get_logger(), "The log has been processed into the appropriate format and temporarily stored in log_array.");
    }

    void publish_log_callback() {
        std::lock_guard<std::mutex> guard(mtx_log_array);

        if(!log_array.log_array.empty()){
            log_publisher->publish(log_array);
        }

        log_array.log_array.clear();
        
        RCLCPP_INFO(this->get_logger(), "Published log_array");
    }

};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Logging>("logging");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}