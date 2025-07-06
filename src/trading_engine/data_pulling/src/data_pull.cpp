#include <cstdlib>
#include <iostream>
#include <memory>
#include "extractData.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <future>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ccapi_cpp/ccapi_session.h"
#include "system_interface/msg/tick_data.hpp"

using namespace std::chrono_literals;
using ::ccapi::Event;
using ::ccapi::EventDispatcher;
using ::ccapi::Session;
using ::ccapi::SessionConfigs;
using ::ccapi::SessionOptions;
using ::ccapi::Subscription;
using ::ccapi::toString;

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode(const std::string& name) 
        : Node(name)
    {
        publisher_ = this->create_publisher<system_interface::msg::TickData>("gateway_tick_data", 10);
        timer_ = this->create_wall_timer(5s, std::bind(&PublisherNode::fetch_and_publish_data, this));

        SessionOptions sessionOptions;
        SessionConfigs sessionConfigs;
        session_ = std::make_unique<Session>(sessionOptions, sessionConfigs);

        Subscription bitmexSubscription("bitmex", "XBTUSD", "TRADE");
        session_->subscribe(bitmexSubscription);
    }

    ~PublisherNode() {
        if (session_) {
            session_->stop();
        }
    }

private:
    rclcpp::Publisher<system_interface::msg::TickData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<Session> session_;

    void fetch_and_publish_data() {
        RCLCPP_INFO(this->get_logger(), "Fetching and publishing data");

        std::vector<Event> eventList = session_->getEventQueue().purge();
        RCLCPP_INFO(this->get_logger(), "Received %zu events", eventList.size());

        for (const auto& event : eventList) {
            try {
                RCLCPP_INFO(this->get_logger(), "Handling Event: %s", toString(event).c_str());
                std::string data = toString(event);

                if (data.find("type = MARKET_DATA_EVENTS_TRADE") != std::string::npos) {
                    system_interface::msg::TickData tick_data;
                    
                    tick_data.symbol = getSymbol(data);
                    tick_data.exchange = "bitmex";
                    tick_data.time = getTime(data);
                    tick_data.is_buy = getIsBuy(data);
                    tick_data.price = getPrice(data);
                    tick_data.size = getSize(data);

                    publisher_->publish(tick_data);

                    RCLCPP_INFO(this->get_logger(), 
                        "Published: symbol=%s, exchange=%s, is_buy=%s, price=%f, size=%f, time=%ld",
                        tick_data.symbol.c_str(), tick_data.exchange.c_str(), tick_data.is_buy ? "true" : "false", 
                        tick_data.price, tick_data.size, tick_data.time);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "An exception occurred while processing an event: %s", e.what());
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>("market_data_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}