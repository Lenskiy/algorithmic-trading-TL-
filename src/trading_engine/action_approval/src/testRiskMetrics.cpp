#include "rclcpp/rclcpp.hpp"
#include "system_interface/srv/order_risk.hpp"
#include <chrono>
#include <random>

class TestRiskMetrics : public rclcpp::Node {
public:
    TestRiskMetrics(const std::string& name) : Node(name), total_completed_(0) {
        service_ = this->create_service<system_interface::srv::OrderRisk>(
            "order_risk_assessment",
            std::bind(&TestRiskMetrics::handle_order_risk, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "TestRiskMetrics started. Waiting for OrderRisk requests...");
    }

private:
    void handle_order_risk(
        const std::shared_ptr<system_interface::srv::OrderRisk::Request> request,
        std::shared_ptr<system_interface::srv::OrderRisk::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received OrderRisk request for symbol: %s", request->symbol.c_str());




        // set sleep time here
        // std::this_thread::sleep_for(std::chrono::seconds(2));



        
        
        // generate random risk level
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 3.0);
        response->value_at_risk = static_cast<float>(dis(gen));

        RCLCPP_INFO(this->get_logger(), "Risk level for symbol %s: %.2f", request->symbol.c_str(), response->value_at_risk);
        total_completed_++;
        print_stats();
    }

    void print_stats() {
        RCLCPP_INFO(this->get_logger(), "Total OrderRisk requests completed: %d", total_completed_);
    }

    rclcpp::Service<system_interface::srv::OrderRisk>::SharedPtr service_;

    int total_completed_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestRiskMetrics>("test_risk_metrics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}