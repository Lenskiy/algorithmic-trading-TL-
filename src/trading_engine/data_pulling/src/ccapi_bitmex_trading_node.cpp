#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "ccapi_cpp/ccapi_session.h"
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

namespace ccapi {
Logger* Logger::logger = nullptr;  // This line is needed.

class MyEventHandler : public EventHandler {
 public:
  bool processEvent(const Event& event, Session* session) override {
    std::cout << "Received an event:\n" + event.toStringPretty(2, 2) << std::endl;
    return true;
  }
};
} /* namespace ccapi */

using json = nlohmann::json;
using namespace std::chrono_literals;

class CCAPIBitMEXTradingNode : public rclcpp::Node {
public:
    CCAPIBitMEXTradingNode() : Node("ccapi_bitmex_trading_node"), stop_flag_(false) {
        this->declare_parameter("api_key", "");
        this->declare_parameter("api_secret", "");

        api_key_ = this->get_parameter("api_key").as_string();
        api_secret_ = this->get_parameter("api_secret").as_string();

        if (api_key_.empty() || api_secret_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "API key or secret is not set");
            return;
        }

        // Set environment variables for API credentials
        setenv("BITMEX_API_KEY", api_key_.c_str(), 1);
        setenv("BITMEX_API_SECRET", api_secret_.c_str(), 1);

        // Set up CCAPI session
        sessionOptions_ = std::make_unique<ccapi::SessionOptions>();
        sessionConfigs_ = std::make_unique<ccapi::SessionConfigs>();
        eventHandler_ = std::make_unique<ccapi::MyEventHandler>();
        session_ = std::make_unique<ccapi::Session>(*sessionOptions_, *sessionConfigs_, eventHandler_.get());

        // Create a subscription to receive trading commands
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "trading_commands", 10, std::bind(&CCAPIBitMEXTradingNode::command_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CCAPI BitMEX Trading Node initialized");
    }

    ~CCAPIBitMEXTradingNode() {
        stop_flag_ = true;
        if (session_) {
            session_->stop();
        }
    }

private:
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());
        try {
            json command = json::parse(msg->data);
            if (command["action"] == "place_order") {
                place_order(command["symbol"], command["side"], command["quantity"], command["price"]);
            } else if (command["action"] == "get_account_balance") {
                get_account_balance();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown command: %s", command["action"].get<std::string>().c_str());
            }
        } catch (json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing command: %s", e.what());
        }
    }

    void place_order(const std::string& symbol, const std::string& side, double quantity, double price) {
        ccapi::Request request(ccapi::Request::Operation::CREATE_ORDER, "bitmex", symbol);
        request.appendParam({
            {"side", side},
            {"orderQty", std::to_string(quantity)},
            {"price", std::to_string(price)},
            {"ordType", "Limit"}
        });
        session_->sendRequest(request);
    }

    void get_account_balance() {
        ccapi::Request request(ccapi::Request::Operation::GET_ACCOUNT_BALANCES, "bitmex");
        session_->sendRequest(request);
    }

    std::unique_ptr<ccapi::SessionOptions> sessionOptions_;
    std::unique_ptr<ccapi::SessionConfigs> sessionConfigs_;
    std::unique_ptr<ccapi::MyEventHandler> eventHandler_;
    std::unique_ptr<ccapi::Session> session_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string api_key_;
    std::string api_secret_;
    std::atomic<bool> stop_flag_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CCAPIBitMEXTradingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}