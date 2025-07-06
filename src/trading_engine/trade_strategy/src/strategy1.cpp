#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <arrow/array.h>
#include <arrow/builder.h>
#include <arrow/type.h>
#include <memory>
#include <arrow/api.h>
#include <parquet/arrow/writer.h>
#include "system_interface/msg/template_info.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "strategy1 node is running.");
        // 3. Create a subscriber
        subscription_ = this->create_subscription<system_interface::msg::TemplateInfo>("string_msg", 10,
                                                                          std::bind(&SubscriberNode::sub_callback, this, std::placeholders::_1));
    }

private:
    // 1.Declare subscribers
    rclcpp::Subscription<system_interface::msg::TemplateInfo>::SharedPtr subscription_;
    // 2.Subscriber callback function
    void sub_callback(const system_interface::msg::TemplateInfo::SharedPtr msgs)
    {   
        std::string example_symbol = msgs->symbol;
        double example_bid_size = msgs->bidsize;
        double example_bid_price = msgs->bidprice;
        double example_ask_size = msgs->asksize;
        double example_ask_price = msgs->askprice;

        RCLCPP_INFO(this->get_logger(), "Receiving name: %s, bidsize: %f, bidprice: %f,asksize: %f, askprice: %f",
                    example_symbol.c_str(),
                    example_bid_size,
                    example_bid_price,
                    example_ask_size,
                    example_ask_price);
        }
    
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>("strategy1");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}