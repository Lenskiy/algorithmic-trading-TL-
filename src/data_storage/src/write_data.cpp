#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include "system_interface/msg/tick_data.hpp"
#include "TickDataParquet.h"
#include <queue>
#include <mutex>
#include <vector>
#include <atomic>

using namespace std::chrono_literals;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string name) : Node(name), data_writer_("trades.parquet"), is_shutting_down_(false)
    {
        RCLCPP_INFO(this->get_logger(), "data storaging node is running.");
        subscription_ = this->create_subscription<system_interface::msg::TickData>(
            "gateway_tick_data", 10, std::bind(&SubscriberNode::sub_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1s, std::bind(&SubscriberNode::write_data, this));
    }

    ~SubscriberNode()
    {
        is_shutting_down_ = true;
        RCLCPP_INFO(this->get_logger(), "Performing final write in destructor.");
        write_data();
    }

private:
    rclcpp::Subscription<system_interface::msg::TickData>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    TickDataParquet data_writer_;
    std::queue<system_interface::msg::TickData> tick_data_queue_;
    std::mutex queue_mutex_;
    std::atomic<bool> is_shutting_down_;

    void sub_callback(const system_interface::msg::TickData::SharedPtr tick_data)
    {
        if (is_shutting_down_)
            return;

        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // filter zero data
        if (tick_data->time == 0)
            return;

        tick_data_queue_.push(*tick_data); // 将 TickData 直接加入队列
        RCLCPP_INFO(this->get_logger(), "Parsed data: symbol=%s, exchange=%s, is_buy=%s, price=%f, size=%f, time=%ld",
                    tick_data->symbol.c_str(), tick_data->exchange.c_str(), tick_data->is_buy ? "true" : "false",
                    tick_data->price, tick_data->size, tick_data->time);
    }

    void write_data()
    {
        std::vector<system_interface::msg::TickData> tick_data_vec;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            while (!tick_data_queue_.empty())
            {
                tick_data_vec.push_back(tick_data_queue_.front());
                tick_data_queue_.pop();
            }
        }

        if (!tick_data_vec.empty())
        {
            data_writer_.writeData(tick_data_vec); // 假设 DataWriter 支持 TickData 的直接写入
            RCLCPP_INFO(this->get_logger(), "Wrote %zu TickData to Parquet file.", tick_data_vec.size());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>("data_storage");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
