#include "rclcpp/rclcpp.hpp"

#include "system_interface/srv/order_risk.hpp"
#include "system_interface/srv/calculate_price_risk.hpp"
#include "system_interface/srv/calculate_liquidity_risk.hpp"
#include "system_interface/srv/calculate_correlation.hpp"
#include "system_interface/srv/get_historical_prices.hpp"
#include "system_interface/srv/get_historical_tick_datas.hpp"
#include "risk_calculation.cpp"
#include <memory>

class RiskServer : public rclcpp::Node
{
public:
    RiskServer(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Risk server node is created.");


        // client to access data storage
        data_client = this->create_client<system_interface::srv::GetHistoricalTickDatas>(
            "get_historical_tick_datas");

        // risk service with re-entry callback group
        risk_cal_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        risk_service = this->create_service<system_interface::srv::OrderRisk>(
            "order_risk", 
            std::bind(&RiskServer::accept_order_risk, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2)
        );

        var_service = this->create_service<system_interface::srv::CalculatePriceRisk>(
            "calculate_VaR", 
            std::bind(&RiskServer::handle_VaR, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

        liquidity_service = this->create_service<system_interface::srv::CalculateLiquidityRisk>(
            "calculate_liquidity", 
            std::bind(&RiskServer::handle_liqduity, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

        correlation_service = this->create_service<system_interface::srv::CalculateCorrelation>(
            "calculate_correlation", 
            std::bind(&RiskServer::handle_correlation, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));            
    }


private:
    RiskCalculation risk_;
    rclcpp::CallbackGroup::SharedPtr risk_cal_callback_group;
    // Data storage client
    rclcpp::Client<system_interface::srv::GetHistoricalTickDatas>::SharedPtr data_client;
    // Declare the server
    rclcpp::Service<system_interface::srv::OrderRisk>::SharedPtr risk_service;
    rclcpp::Service<system_interface::srv::CalculatePriceRisk>::SharedPtr var_service;
    rclcpp::Service<system_interface::srv::CalculateLiquidityRisk>::SharedPtr liquidity_service;
    rclcpp::Service<system_interface::srv::CalculateCorrelation>::SharedPtr correlation_service;

    void accept_order_risk(
        const std::shared_ptr<system_interface::srv::OrderRisk::Request> risk_request,
         std::shared_ptr<system_interface::srv::OrderRisk::Response> risk_response){
            // Message input
            RCLCPP_INFO(this->get_logger(), 
            "Received risk metrics request for symbol: %s", risk_request->symbol);

            // Request data
            auto data_request = std::make_shared<system_interface::srv::GetHistoricalTickDatas::Request>();
            data_request->symbol = risk_request->symbol;
            data_request->exchange = risk_request->exchange;
            data_request->end_time = risk_request->time - 1;  
                // allow the historical data to be in the data storage
            data_request->start_time = risk_request->time - 100;
            // data_request->frequency = "1min";

            // use lambda to send risk response based on data response
            auto callback = [this, risk_response]
            (const rclcpp::Client<system_interface::srv::GetHistoricalTickDatas>::SharedFuture data_response) {
                this->historical_data_call_back(data_response, risk_response);
                };

            data_client->async_send_request(data_request, callback);
         }

    void historical_data_call_back(const rclcpp::Client<system_interface::srv::GetHistoricalTickDatas>::SharedFuture data_response,
    const std::shared_ptr<system_interface::srv::OrderRisk::Response> risk_response) {
        auto cal = new RiskCalculation();
        auto data = data_response.get();
        
        std::vector<float> prices;
        for (const auto& price_msg : data->tick_datas) {
            prices.push_back(price_msg.price); // Extract the double value from the message
        }
        
        // value at risk   
        risk_response->value_at_risk = cal->var_historical(prices);

    }

    /*
    SERVER for calculating VaR (value at risk)
    */    
   void handle_VaR(
         const std::shared_ptr<system_interface::srv::CalculatePriceRisk::Request> request,
         std::shared_ptr<system_interface::srv::CalculatePriceRisk::Response> response) {
            // send a request to data storage and calculate the result as response
         }
    
    /*
    SERVER for liquidity risk
    */
    void handle_liqduity(
        const std::shared_ptr<system_interface::srv::CalculateLiquidityRisk::Request> request,
         std::shared_ptr<system_interface::srv::CalculateLiquidityRisk::Response> response) {

         }

    /*
    SERVER for capital risk
    */
    void handle_correlation(
        const std::shared_ptr<system_interface::srv::CalculateCorrelation::Request> request,
         std::shared_ptr<system_interface::srv::CalculateCorrelation::Response> response) {

         }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RiskServer>("risk_server");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}