#include <rclcpp/rclcpp.hpp>
#include <system_interface/msg/template_info.hpp>
#include <system_interface/msg/price.hpp>
#include <chrono>
#include <deque>
#include <mutex>
#include <ctime>
#include <iomanip>
#include <sstream>

class OHLCVProcessorNode : public rclcpp::Node {
public:
    OHLCVProcessorNode() : Node("ohlcv_processor") {
        subscriber_ = this->create_subscription<system_interface::msg::TemplateInfo>(
            "market_data", 10, std::bind(&OHLCVProcessorNode::market_data_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<system_interface::msg::Price>("gateway_price_topic", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10), std::bind(&OHLCVProcessorNode::publish_ohlcv, this));
        
        RCLCPP_INFO(this->get_logger(), "OHLCV Processor Node initialized");
    }

private:
    rclcpp::Subscription<system_interface::msg::TemplateInfo>::SharedPtr subscriber_;
    rclcpp::Publisher<system_interface::msg::Price>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mutex data_mutex_;
    std::deque<system_interface::msg::TemplateInfo> price_history_;
    
    double open_price_ = 0.0;
    double high_price_ = 0.0;
    double low_price_ = std::numeric_limits<double>::max();
    double close_price_ = 0.0;
    double volume_ = 0.0;
    int64_t start_time_ = 0;
    std::string current_symbol_;
    std::string current_exchange_ = "bitmex"; 

    bool is_initialized_;
    rclcpp::Time period_start_time_;

    void market_data_callback(const system_interface::msg::TemplateInfo::SharedPtr msg) {
        // Checking data validity
        if (msg->time == "1970-01-01T00:00:00.000000000Z" || (msg->bidprice == 0 && msg->askprice == 0)) {
            RCLCPP_WARN(this->get_logger(), "Received invalid data, skipping...");
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        double mid_price = (msg->bidprice + msg->askprice) / 2.0;

        if (!is_initialized_) {
            initialize_period(mid_price, msg->symbol);
        }
        
        update_ohlcv(mid_price, msg->bidsize, msg->asksize);
        
        price_history_.push_back(*msg);
        
        RCLCPP_INFO(this->get_logger(), "Updated OHLCV: high=%f, low=%f, close=%f, volume=%f",
                    high_price_, low_price_, close_price_, volume_);
        
        RCLCPP_INFO(this->get_logger(), "Price history size: %zu", price_history_.size());
    }

    void initialize_period(double price, const std::string& symbol) {
        period_start_time_ = this->now();
        start_time_ = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        open_price_ = price;
        high_price_ = price;
        low_price_ = price;
        close_price_ = price;
        current_symbol_ = symbol;
        volume_ = 0.0;
        is_initialized_ = true;
        price_history_.clear();  // Clear the price history at the start of each new period
        RCLCPP_INFO(this->get_logger(), "Initialized new period: open_price=%f, symbol=%s",
                    open_price_, current_symbol_.c_str());
    }

    void update_ohlcv(double price, double bid_size, double ask_size) {
        high_price_ = std::max(high_price_, price);
        low_price_ = std::min(low_price_, price);
        close_price_ = price;
        volume_ += (bid_size + ask_size) / 2.0;
    }

    void publish_ohlcv() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!is_initialized_ || price_history_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No data available to publish OHLCV");
            return;
        }
        
        system_interface::msg::Price ohlcv_data;
        ohlcv_data.time = static_cast<int32_t>(start_time_);
        ohlcv_data.frequency = "10 seconds";
        ohlcv_data.symbol = current_symbol_;
        ohlcv_data.exchange = current_exchange_;
        ohlcv_data.open = static_cast<float>(open_price_);
        ohlcv_data.high = static_cast<float>(high_price_);
        ohlcv_data.low = static_cast<float>(low_price_);
        ohlcv_data.close = static_cast<float>(close_price_);
        ohlcv_data.volume = static_cast<float>(volume_);
        
        publisher_->publish(ohlcv_data);
        
        RCLCPP_INFO(this->get_logger(),
            "Published OHLCV: symbol=%s, time=%d, frequency=%s, exchange=%s, open=%f, high=%f, low=%f, close=%f, volume=%f",
            ohlcv_data.symbol.c_str(), ohlcv_data.time, ohlcv_data.frequency.c_str(), ohlcv_data.exchange.c_str(),
            ohlcv_data.open, ohlcv_data.high, ohlcv_data.low, ohlcv_data.close, ohlcv_data.volume);
        
        // Start a new period
        initialize_period(close_price_, current_symbol_);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OHLCVProcessorNode>());
    rclcpp::shutdown();
    return 0;
}