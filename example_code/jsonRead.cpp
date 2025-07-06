#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
using namespace std;
// Define a Trade structure
struct Trade {
    std::string trade_date;       
    std::string trade_time;          
    int64_t execution_timestamp;     
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    // constructor 
    Trade(std::string date, std::string tie, int64_t timestamp, int trader, std::string symbol, int qty, double pr) :
        trade_date(date), trade_time(time),m execution_timestamp(timestamp),
        trader_id(trader), asset_symbol(symbol), quantity(qty), price(pr) {}
};

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    nlohmann::json j;
    std::ifstream file("large_trades.json");
    file >> j;

    for (const auto& item : j) {
        Trade trade(
            item["trade_date"].get<std::string>(),
            item["trade_time"].get<std::string>(),
            item["execution_timestamp"].get<int64_t>(),
            item["trader_id"].get<int>(),
            item["asset_symbol"].get<std::string>(),
            item["quantity"].get<int>(),
            item["price"].get<double>()
        );
        // Process trade or print it out
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "JSON read time: " << elapsed.count() << " seconds." << std::endl;

    return 0;
}
