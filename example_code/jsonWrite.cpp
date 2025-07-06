#include <nlohmann/json.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
struct Trade {
    std::string trade_date;       
    std::string trade_time;          
    int64_t execution_timestamp;     
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    Trade(std::string date, std::string time, int64_t timestamp, int trader, std::string symbol, int qty, double pr) :
        trade_date(date), trade_time(time), execution_timestamp(timestamp),
        trader_id(trader), asset_symbol(symbol), quantity(qty), price(pr) {}
};

std::vector<Trade> getTrades() {
    std::vector<Trade> trades;
    for (int i = 0; i < 2000; ++i) { 
        trades.emplace_back("2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75);
    }
    return trades;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Trade> trades = getTrades();
    nlohmann::json j;

    for (const auto& trade : trades) {
        nlohmann::json item;
        item["trade_date"] = trade.trade_date;
        item["trade_time"] = trade.trade_time;
        item["execution_timestamp"] = trade.execution_timestamp;
        item["trader_id"] = trade.trader_id;
        item["asset_symbol"] = trade.asset_symbol;
        item["quantity"] = trade.quantity;
        item["price"] = trade.price;
        j.push_back(item);
    }

    std::ofstream file("large_trades.json");
    file << j.dump(4);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "JSON write time: " << elapsed.count() << " seconds." << std::endl;

    return 0;
}
