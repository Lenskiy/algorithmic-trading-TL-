#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <atomic>
#include "std_msgs/msg/string.hpp"
#include "system_interface/msg/template_info.hpp"
#include "system_interface/msg/tick_data.hpp"
#include "system_interface/srv/get_historical_tick_datas.hpp"
#include "TickDataParquet.h"

using namespace std::chrono_literals;

class DataServerNode : public rclcpp::Node
{
public:
    DataServerNode(std::string name) : Node(name), data_reader_("trades.parquet"), is_shutting_down_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Local data access server is running.");
        data_service_ = this->create_service<system_interface::srv::GetHistoricalTickDatas>(
            "get_historical_tick_datas",
            std::bind(&DataServerNode::handle_request,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));
    }

    ~DataServerNode()
    {
        is_shutting_down_ = true;
        RCLCPP_INFO(this->get_logger(), "Data access server is closed.");
    }

private:
    rclcpp::Service<system_interface::srv::GetHistoricalTickDatas>::SharedPtr data_service_;
    TickDataParquet data_reader_;
    std::atomic<bool> is_shutting_down_;

    void handle_request(const std::shared_ptr<system_interface::srv::GetHistoricalTickDatas::Request> request,
                        std::shared_ptr<system_interface::srv::GetHistoricalTickDatas::Response> response)
    {
        std::vector<system_interface::msg::TickData> trades = data_reader_.readData(); // orderbook data
        response->tick_datas = trades;
        for (size_t i = 0; i < trades.size(); ++i) // 使用 trades.size() 获取 vector 的大小
        {
            const auto &tick_data = trades[i]; // 获取第 i 个 TickData 对象
            RCLCPP_INFO(this->get_logger(), "Parsed data: symbol=%s, exchange=%s, is_buy=%s, price=%f, size=%f, time=%ld",
                        tick_data.symbol.c_str(), tick_data.exchange.c_str(), tick_data.is_buy ? "true" : "false",
                        tick_data.price, tick_data.size, tick_data.time);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataServerNode>("data_storage");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}