#include "rclcpp/rclcpp.hpp"
#include "system_interface/msg/order.hpp"
#include <random>
#include <chrono>

class TestAgentPool : public rclcpp::Node {
public:
    TestAgentPool(const std::string& name) : Node(name), total_published(0) {
        order_publisher_ = this->create_publisher<system_interface::msg::Order>("incoming_orders", 10);
        
        timer_ = this->create_wall_timer(
            // change timer duration here
            std::chrono::milliseconds(10),
            std::bind(&TestAgentPool::publish_random_order, this));
        
        RCLCPP_INFO(this->get_logger(), "TestAgentPool started. Publishing orders every 0.5 seconds.");
    }

private:
    int total_published;
    void publish_random_order() {
        auto order = system_interface::msg::Order();
        
        // generate random order
        order.symbol = generate_random_symbol();
        order.exchange = generate_random_exchange();
        order.price = generate_random_price();
        order.size = generate_random_size();
        order.side = generate_random_side();
        order.time = static_cast<int32_t>(this->now().seconds());

        order_publisher_->publish(order);
        total_published++;
        
        RCLCPP_INFO(this->get_logger(), "Publishing order: %s %s %f %f %s",
            order.symbol.c_str(), order.exchange.c_str(),
            order.price, order.size,
            order.side.c_str());

        RCLCPP_WARN(this->get_logger(), "Total orders published: %d", total_published);
    }

    std::string generate_random_symbol() {
        static const std::vector<std::string> symbols = {"AAPL", "GOOGL", "MSFT", "AMZN", "FB"};
        return symbols[rand() % symbols.size()];
    }

    std::string generate_random_exchange() {
        static const std::vector<std::string> exchanges = {"NYSE", "NASDAQ", "LSE", "TSE"};
        return exchanges[rand() % exchanges.size()];
    }

    float generate_random_price() {
        return 50.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 950.0f));
    }

    float generate_random_size() {
        return 1.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 99.0f));
    }

    std::string generate_random_side() {
        if(rand() % 2 == 0) {
            return "buy";
        }else{
            return "sell";
        }
    }

    rclcpp::Publisher<system_interface::msg::Order>::SharedPtr order_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestAgentPool>("test_agent_pool");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}