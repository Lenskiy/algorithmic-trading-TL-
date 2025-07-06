#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>

using json = nlohmann::json;
using namespace std::chrono_literals;

class AlphaVantageNode : public rclcpp::Node {
public:
    AlphaVantageNode() : Node("alpha_vantage_node") {
        this->declare_parameter("api_key", "");
        this->declare_parameter("symbols", std::vector<std::string>{"IBM"});
        this->declare_parameter("update_interval", 60.0);  // Default to 60 seconds due to API rate limits

        api_key_ = this->get_parameter("api_key").as_string();
        symbols_ = this->get_parameter("symbols").as_string_array();
        update_interval_ = this->get_parameter("update_interval").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::String>("stock_data", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(update_interval_),
            std::bind(&AlphaVantageNode::timer_callback, this));

        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_ = curl_easy_init();

        RCLCPP_INFO(this->get_logger(), "Alpha Vantage Node initialized with API key: %s...", api_key_.substr(0, 5).c_str());
        RCLCPP_INFO(this->get_logger(), "Symbols to track: %s", join(symbols_, ", ").c_str());
        RCLCPP_INFO(this->get_logger(), "Update interval: %.2f seconds", update_interval_);
    }

    ~AlphaVantageNode() {
        curl_easy_cleanup(curl_);
        curl_global_cleanup();
        RCLCPP_INFO(this->get_logger(), "Alpha Vantage Node shutting down");
    }

private:
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
        for (const auto& symbol : symbols_) {
            RCLCPP_INFO(this->get_logger(), "Fetching data for symbol: %s", symbol.c_str());
            fetch_stock_data(symbol);
            std::this_thread::sleep_for(12s);  // Sleep to respect rate limit (5 calls per minute)
        }
    }

    void fetch_stock_data(const std::string& symbol) {
    if(curl_) {
        std::string readBuffer;
        std::string url = "https://www.alphavantage.co/query?function=TIME_SERIES_INTRADAY&symbol=" + symbol + "&interval=1min&apikey=" + api_key_;
        
        RCLCPP_INFO(this->get_logger(), "Sending request to URL: %s", url.c_str());
        
        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);

        CURLcode res = curl_easy_perform(curl_);
        if(res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
        } else {
            try {
                json j = json::parse(readBuffer);
                if (j.contains("Time Series (1min)")) {
                    auto timeSeries = j["Time Series (1min)"];
                    auto it = timeSeries.begin();  // Get the most recent data point
                    std::string timestamp = it.key();
                    auto data = it.value();
                    
                    auto message = std_msgs::msg::String();
                    message.data = 
                        "stock_code: " + symbol + "\n" +
                        "market_code: US\n" +  // Assuming US market, adjust if needed
                        "open: " + data["1. open"].get<std::string>() + "\n" +
                        "high: " + data["2. high"].get<std::string>() + "\n" +
                        "low: " + data["3. low"].get<std::string>() + "\n" +
                        "close: " + data["4. close"].get<std::string>() + "\n" +
                        "time: " + timestamp + "\n" +
                        "count: " + data["5. volume"].get<std::string>() + "\n" +
                        "frequency: 1min\n";
                    
                    // Calculate amount (total value traded)
                    float close = std::stof(data["4. close"].get<std::string>());
                    float volume = std::stof(data["5. volume"].get<std::string>());
                    float amount = close * volume;
                    message.data += "amount: " + std::to_string(amount) + "\n";
                    
                    publisher_->publish(message);
                    RCLCPP_INFO(this->get_logger(), "Published data for %s at %s", symbol.c_str(), timestamp.c_str());
                } else if (j.contains("Note")) {
                    RCLCPP_WARN(this->get_logger(), "API call limit reached: %s", j["Note"].get<std::string>().c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Unexpected API response: %s", readBuffer.c_str());
                }
            } catch (json::parse_error& e) {
                RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "CURL not initialized");
    }
}

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string*)userp)->append((char*)contents, size * nmemb);
        return size * nmemb;
    }

    std::string join(const std::vector<std::string>& vec, const std::string& delim) {
        std::string result;
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i > 0) result += delim;
            result += vec[i];
        }
        return result;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    CURL *curl_;
    std::string api_key_;
    std::vector<std::string> symbols_;
    double update_interval_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlphaVantageNode>();
    RCLCPP_INFO(node->get_logger(), "Starting Alpha Vantage Node");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down Alpha Vantage Node");
    rclcpp::shutdown();
    return 0;
}