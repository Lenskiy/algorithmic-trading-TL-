#include "rclcpp/rclcpp.hpp"
#include "system_interface/msg/order.hpp"
#include <chrono>

class TestControlExternal : public rclcpp::Node {
public:
    TestControlExternal(const std::string& name) : Node(name), approved_orders_count_(0) {
        subscription_ = this->create_subscription<system_interface::msg::Order>(
            "approved_orders", 10, 
            std::bind(&TestControlExternal::order_callback, this, std::placeholders::_1));

    
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TestControlExternal::print_stats, this));

        RCLCPP_INFO(this->get_logger(), "TestControlExternal started. Listening for approved orders...");
    }

private:
    void order_callback(const system_interface::msg::Order::SharedPtr msg) {
        approved_orders_count_++;
        RCLCPP_INFO(this->get_logger(), "Received order: %s %s %f %f %s %ld",
            msg->symbol.c_str(), msg->exchange.c_str(),
            msg->price, msg->size,
            msg->side.c_str(), msg->time);

    }

    void print_stats() {
        RCLCPP_INFO(this->get_logger(), "Total approved orders received: %d", approved_orders_count_);
    }

    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    int approved_orders_count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestControlExternal>("test_control_external");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}