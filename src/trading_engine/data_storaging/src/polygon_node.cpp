#include <rclcpp/rclcpp.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <system_interface/srv/get_historical_prices.hpp>
#include <system_interface/msg/price.hpp>
#include <vector>
#include <string>
#include <regex>

using json = nlohmann::json;
using std::placeholders::_1;
using std::placeholders::_2;

std::string convert_frequency(const std::string &frequency) {
    // Regular expression to match the frequency input (e.g., 10s, 3min, 2h, 1d)
    std::regex frequency_regex(R"((\d+)(s|min|h|d))");
    std::smatch match;

    if (std::regex_match(frequency, match, frequency_regex)) {
        std::string number = match[1];   // The numeric part (e.g., "10", "3")
        std::string unit = match[2];     // The unit part (e.g., "s", "min", "h", "d")

        // Convert to the format required by the API
        if (unit == "s") {
            return number + "/second";
        } else if (unit == "min") {
            return number + "/minute";
        } else if (unit == "h") {
            return number + "/hour";
        } else if (unit == "d") {
            return number + "/day";
        }
    }

    // If the input doesn't match expected formats, return empty or default value
    return "";
}

class PolygonServiceNode : public rclcpp::Node {
public:
    PolygonServiceNode() : Node("polygon_service_node") {
        // Create the service named "get_historical_prices"
        service_ = this->create_service<system_interface::srv::GetHistoricalPrices>(
            "get_historical_prices", std::bind(&PolygonServiceNode::handle_service_request, this, _1, _2));

        // Initialize CURL globally (for making HTTP requests)
        curl_global_init(CURL_GLOBAL_DEFAULT);

        RCLCPP_INFO(this->get_logger(), "Polygon Service Node is ready.");
    }

    ~PolygonServiceNode() {
        curl_global_cleanup();
        RCLCPP_INFO(this->get_logger(), "Polygon Service Node shutting down.");
    }

private:
    // Callback function to handle incoming service requests
    void handle_service_request(
        const std::shared_ptr<system_interface::srv::GetHistoricalPrices::Request> request,
        std::shared_ptr<system_interface::srv::GetHistoricalPrices::Response> response) {
        
        std::string stock_code = request->symbol;
        std::string market_code = request->exchange;
        int64_t start_time = request->start_time;
        int64_t end_time = request->end_time;
        std::string frequency = request->frequency;

        RCLCPP_INFO(this->get_logger(), "Fetching historical prices for stock: %s", stock_code.c_str());

        // Fetch the stock data from Polygon API
        std::string api_response = fetch_stock_data(stock_code, market_code, start_time, end_time, frequency, "re7qTQPWTdM5s9UyjGYZ2p1H1K87ie04");

        if (!api_response.empty()) {
            // Parse and populate the response with Price data
            parse_and_fill_prices(api_response, stock_code, market_code, frequency, response->prices);
            RCLCPP_INFO(this->get_logger(), "Successfully fetched historical prices for stock: %s", stock_code.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to fetch data for stock: %s", stock_code.c_str());
        }
    }

    std::string fetch_stock_data(const std::string &stock_code, const std::string &market_code, int64_t start_time, int64_t end_time, const std::string &frequency, const std::string &api_key) {
        CURL *curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
            return "";
        }

        // Convert frequency to the correct API format
        std::string formatted_frequency = convert_frequency(frequency);
        if (formatted_frequency.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid frequency format");
            return "";
        }

        std::string url = "https://api.polygon.io/v2/aggs/ticker/" + stock_code +
                          "/range/" + formatted_frequency + "/" + std::to_string(start_time) + "/" +
                          std::to_string(end_time) + "?apiKey=" + api_key;
        RCLCPP_INFO(this->get_logger(), "Fetching url: %s", url.c_str());

        std::string readBuffer;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
            curl_easy_cleanup(curl);
            return "";
        }

        curl_easy_cleanup(curl);
        return readBuffer;
    }

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    void parse_and_fill_prices(const std::string &api_response, const std::string &stock_code, const std::string &market_code, const std::string &frequency, std::vector<system_interface::msg::Price> &prices) {
        try {
            json j = json::parse(api_response);
            if (j.contains("results")) {
                auto results = j["results"];
                for (const auto &result : results) {
                    system_interface::msg::Price price;
                    price.symbol = stock_code;
                    price.exchange = market_code;
                    price.open = result["o"].get<double>();
                    price.close = result["c"].get<double>();
                    price.high = result["h"].get<double>();
                    price.low = result["l"].get<double>();
                    price.time = result["t"].get<int64_t>();
                    // price.count = result["n"].get<double>();
                    price.volume = result["v"].get<double>();
                    price.frequency = frequency;

                    prices.push_back(price);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unexpected API response: %s", api_response.c_str());
            }
        } catch (json::parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        }
    }

    rclcpp::Service<system_interface::srv::GetHistoricalPrices>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolygonServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
