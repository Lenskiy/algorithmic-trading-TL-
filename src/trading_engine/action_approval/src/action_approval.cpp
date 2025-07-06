#include "rclcpp/rclcpp.hpp"
#include "system_interface/msg/order.hpp"
#include "system_interface/srv/order_risk.hpp"

#include <unordered_map>
#include <queue>
#include <mutex>
#include <chrono>
#include <vector>
#include <algorithm>
#include <atomic>

class ActionApproval : public rclcpp::Node {
public:
    ActionApproval(const std::string& name)
        : Node(name),
          total_received_(0),
          total_sent_(0),
          total_approved_(0),
          total_rejected_(0),
          max_concurrent_requests_(10),
          current_requests_(0),
          order_sequence_(0) {

        order_subscription_ = this->create_subscription<system_interface::msg::Order>(
            "incoming_orders", 100,
            std::bind(&ActionApproval::order_callback, this, std::placeholders::_1));

        risk_client_ = this->create_client<system_interface::srv::OrderRisk>("order_risk");
        order_publisher_ = this->create_publisher<system_interface::msg::Order>("approved_orders", 10);

        // Timer to print statistics every 10 seconds
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&ActionApproval::print_stats, this));

        // Timer to process the order queue every 100 milliseconds
        process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ActionApproval::process_queue, this));

        RCLCPP_INFO(this->get_logger(), "Action Approval Node Started");
    }

private:
    struct OrderInfo {
        system_interface::msg::Order order;
        enum class Status { PENDING, PROCESSING, COMPLETED, REJECTED } status;
        rclcpp::Time request_time;  // Timestamp when the order was received
        int retry_count;            // Number of retries attempted
    };

    // Callback function when a new order is received
    void order_callback(const system_interface::msg::Order::SharedPtr msg) {
        total_received_++;
        uint64_t sequence_number = order_sequence_++;  // Get a unique sequence number
        // Create a unique order ID using symbol, time, and sequence number
        std::string order_id = msg->symbol + "_" + std::to_string(msg->time) + "_" + std::to_string(sequence_number);

        std::lock_guard<std::mutex> lock(orders_mutex_);  // Lock the mutex for thread safety
        if (orders_.size() >= 10000) {
            // If the orders map is full, drop the order and log a warning
            RCLCPP_WARN(this->get_logger(), "Order queue full, dropping order: %s", order_id.c_str());
            return;
        }
        // Add the new order to the orders map and push its ID onto the processing queue
        orders_[order_id] = OrderInfo{*msg, OrderInfo::Status::PENDING, this->now(), 0};
        order_queue_.push(order_id);

        // Log that the order was received
        RCLCPP_INFO(this->get_logger(), "Received order: %s, Queue size: %zu", order_id.c_str(), order_queue_.size());
    }

    // Function to process orders from the queue
    void process_queue() {
        std::lock_guard<std::mutex> lock(orders_mutex_);  // Lock the mutex for thread safety
        // Process orders while the queue is not empty and the max concurrent requests limit is not reached
        while (!order_queue_.empty() && current_requests_ < max_concurrent_requests_) {
            std::string order_id = order_queue_.front();  // Get the next order ID from the queue
            order_queue_.pop();                           // Remove it from the queue
            process_order(order_id);                      // Process the order
        }
    }

    // Function to process a single order
    void process_order(const std::string& order_id) {
        auto order = orders_.find(order_id);  // Find the order in the orders map
        if (order == orders_.end()) {
            // If not found, log an error (this should not happen)
            RCLCPP_ERROR(this->get_logger(), "Order %s not found in map. This should not happen!", order_id.c_str());
            return;
        }

        auto& order_info = order->second;
        // Create a request for the risk assessment service
        auto request = std::make_shared<system_interface::srv::OrderRisk::Request>();
        request->symbol = order_info.order.symbol;
        request->exchange = order_info.order.exchange;
        request->price = order_info.order.price;
        request->size = order_info.order.size;

        // Convert side from string to bool if necessary
        if (order_info.order.side == "buy") {
            request->side = true;
        } else if (order_info.order.side == "sell") {
            request->side = false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid side in order: %s", order_info.order.side.c_str());
            // Handle the invalid side case as needed
            order_info.status = OrderInfo::Status::REJECTED;
            total_rejected_++;
            orders_.erase(order_id);
            return;
        }

        request->time = order_info.order.time;

        // Update the order status and request time
        order_info.status = OrderInfo::Status::PROCESSING;
        order_info.request_time = this->now();

        current_requests_++;  // Increment the number of current requests
        // Send the risk assessment request asynchronously
        auto future = risk_client_->async_send_request(
            request, [this, order_id](rclcpp::Client<system_interface::srv::OrderRisk>::SharedFuture future) {
                handle_risk_response(order_id, future);  // Handle the response when it's ready
                current_requests_--;                    // Decrement the number of current requests
            });
        total_sent_++;  // Increment the total number of sent requests
    }

    // Helper function to check if an order ID is still in the queue
    bool is_order_in_queue(const std::string& order_id) {
        std::queue<std::string> temp = order_queue_;  // Copy the queue to a temporary one
        while (!temp.empty()) {
            if (temp.front() == order_id) {
                return true;  // Order ID found in the queue
            }
            temp.pop();
        }
        return false;  // Order ID not found
    }

    // Function to handle the response from the risk assessment service
    void handle_risk_response(const std::string& order_id,
                              rclcpp::Client<system_interface::srv::OrderRisk>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(orders_mutex_);  // Lock the mutex for thread safety

        auto order_it = orders_.find(order_id);  // Find the order in the orders map
        if (order_it == orders_.end()) {
            if (is_order_in_queue(order_id)) {
                // If the order is still in the queue but not in the map, log a warning
                RCLCPP_WARN(this->get_logger(), "Order %s is still in queue, but not in map. This is unexpected.", order_id.c_str());
                return;
            }
            // If the order is not found at all
            RCLCPP_ERROR(this->get_logger(), "Order not found and not in queue: %s. This should not happen!", order_id.c_str());
            return;
        }

        auto& order_info = order_it->second;

        // Check if the future is ready (non-blocking)
        if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            handle_timeout(order_id);  // Handle the timeout scenario
            return;
        }

        auto response = future.get();

        // RCLCPP_INFO(this->get_logger(), "Risk level for order %s: %.2f", order_id.c_str(), response->risk_level);

        if (response->value_at_risk <= 1.0f) {
            // If the risk level is acceptable, publish the order
            order_publisher_->publish(order_info.order);
            order_info.status = OrderInfo::Status::COMPLETED;
            total_approved_++;  // Increment the total number of approved orders
            RCLCPP_INFO(this->get_logger(), "Order approved and published: %s", order_id.c_str());
        } else {
            // If the risk level is too high, reject the order
            order_info.status = OrderInfo::Status::REJECTED;
            total_rejected_++;  // Increment the total number of rejected orders
            RCLCPP_INFO(this->get_logger(), "Order rejected due to high risk: %s", order_id.c_str());
        }
        orders_.erase(order_it);  // Remove the order from the orders map
    }

    // Function to handle timeouts in risk assessment
    void handle_timeout(const std::string& order_id) {
        auto it = orders_.find(order_id);  // Find the order in the orders map
        if (it == orders_.end()) {
            // If not found, log an error (this should not happen)
            RCLCPP_ERROR(this->get_logger(), "Order %s not found during timeout handling. This should not happen!", order_id.c_str());
            return;
        }

        auto& order_info = it->second;
        // Log a warning about the timeout
        RCLCPP_WARN(this->get_logger(), "Risk assessment for order %s timed out. Retry count: %d",
                    order_id.c_str(), order_info.retry_count);

        if (order_info.retry_count < 3) {
            // If the retry count is less than 3, increment it and requeue the order
            order_info.retry_count++;
            order_queue_.push(order_id);  // Requeue the order for retry
            RCLCPP_INFO(this->get_logger(), "Order %s requeued for retry.", order_id.c_str());
        } else {
            // If the maximum retries have been reached, reject the order
            RCLCPP_ERROR(this->get_logger(), "Max retries reached for order %s. Rejecting.", order_id.c_str());
            order_info.status = OrderInfo::Status::REJECTED;
            total_rejected_++;  // Increment the total number of rejected orders
            orders_.erase(it);  // Remove the order from the orders map
        }
    }

    // Function to print current statistics
    void print_stats() {
        std::lock_guard<std::mutex> lock(orders_mutex_);  // Lock the mutex for thread safety
        int pending_count = 0;
        int processing_count = 0;
        // Iterate through the orders to count pending and processing ones
        for (const auto& [_, order_info] : orders_) {
            if (order_info.status == OrderInfo::Status::PENDING) {
                pending_count++;
            } else if (order_info.status == OrderInfo::Status::PROCESSING) {
                processing_count++;
            }
        }

        // Log the current statistics
        RCLCPP_INFO(this->get_logger(),
                    "Current statistics:\n"
                    "  PENDING orders: %d\n"
                    "  PROCESSING orders: %d\n"
                    "  Total RECEIVED orders: %d\n"
                    "  Total SENT orders: %d\n"
                    "  Total APPROVED orders: %d\n"
                    "  Total REJECTED orders: %d\n"
                    "  Current queue size: %zu\n"
                    "  Current map size: %zu",
                    pending_count, processing_count, total_received_, total_sent_,
                    total_approved_, total_rejected_, order_queue_.size(), orders_.size());
    }

    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr order_subscription_;
    rclcpp::Client<system_interface::srv::OrderRisk>::SharedPtr risk_client_;
    rclcpp::Publisher<system_interface::msg::Order>::SharedPtr order_publisher_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr process_timer_;

    std::unordered_map<std::string, OrderInfo> orders_;
    std::queue<std::string> order_queue_;
    std::mutex orders_mutex_;
    int total_received_;
    int total_sent_;
    int total_approved_;
    int total_rejected_;
    int max_concurrent_requests_;
    std::atomic<int> current_requests_;
    std::atomic<uint64_t> order_sequence_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionApproval>("action_approval");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
