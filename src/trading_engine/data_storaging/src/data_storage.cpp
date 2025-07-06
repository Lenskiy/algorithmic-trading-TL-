/*
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include "dataWrite.h"  // Include the custom data writer for Parquet
#include <fstream>
#include <chrono>
#include <regex>
#include <ctime>

using json = nlohmann::json;
using namespace std::chrono_literals;

class PolygonNode : public rclcpp::Node {
public:
    PolygonNode() : Node("polygon_node"), data_writer_("output.parquet") {  // Initialize DataWriter with Parquet file name
        // Declare the parameters for the Polygon API request
        this->declare_parameter("stock_code", "AAPL");
        this->declare_parameter("market_code", "US");  // Optional, can be ignored
        this->declare_parameter("start_time", 1609459200);  // Default: Jan 1, 2021 (Unix timestamp)
        this->declare_parameter("end_time", 1612137600);    // Default: Jan 31, 2021 (Unix timestamp)
        this->declare_parameter("frequency", "5minute");    // Default to "5minute"
        this->declare_parameter("use_millisecond_timestamp", false);  // Default is false (use formatted date)
        this->declare_parameter("api_key", "re7qTQPWTdM5s9UyjGYZ2p1H1K87ie04");

        // Fetch the parameters
        stock_code_ = this->get_parameter("stock_code").as_string();
        market_code_ = this->get_parameter("market_code").as_string();  // Can be ignored if unnecessary
        start_time_ = this->get_parameter("start_time").as_int();
        end_time_ = this->get_parameter("end_time").as_int();
        frequency_ = this->get_parameter("frequency").as_string();
        use_millisecond_timestamp_ = this->get_parameter("use_millisecond_timestamp").as_bool();
        api_key_ = this->get_parameter("api_key").as_string();

        // Initialize the CURL handler
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_ = curl_easy_init();

        timer_ = this->create_wall_timer(60s, std::bind(&PolygonNode::fetch_market_data, this));

        RCLCPP_INFO(this->get_logger(), "Polygon Node initialized with Stock Code: %s", stock_code_.c_str());
    }

    ~PolygonNode() {
        curl_easy_cleanup(curl_);
        curl_global_cleanup();
        RCLCPP_INFO(this->get_logger(), "Polygon Node shutting down");
    }

private:
    void fetch_market_data() {
        if (!curl_) {
            RCLCPP_ERROR(this->get_logger(), "CURL not initialized");
            return;
        }

        std::string url = build_url();
        if (url.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build the URL.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Fetching market data from URL: %s", url.c_str());

        std::string readBuffer;
        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);

        CURLcode res = curl_easy_perform(curl_);
        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
            return;
        }

        process_and_save_data(readBuffer);
    }

    std::string build_url() {
        // Parse frequency to get time size and time unit
        int time_size;
        std::string time_unit;
        if (!parse_frequency(frequency_, time_size, time_unit)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid frequency format: %s", frequency_.c_str());
            return "";
        }

        // Map time unit to appropriate Polygon API timespan value
        std::string timespan;
        if (time_unit == "second") {
            timespan = "second";  // Correctly handle "second" as a valid timespan
        } else if (time_unit == "minute") {
            timespan = "minute";
        } else if (time_unit == "hour") {
            timespan = "hour";
        } else if (time_unit == "day") {
            timespan = "day";
        } else if (time_unit == "week") {
            timespan = "week";
        } else if (time_unit == "month") {
            timespan = "month";
        } else if (time_unit == "year") {
            timespan = "year";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid time unit: %s", time_unit.c_str());
            return "";
        }

        // Construct the URL based on whether to use millisecond timestamp or formatted date
        std::string start_time_str, end_time_str;
        if (use_millisecond_timestamp_) {
            start_time_str = std::to_string(start_time_ * 1000);  // Convert to milliseconds
            end_time_str = std::to_string(end_time_ * 1000);      // Convert to milliseconds
        } else {
            start_time_str = convert_unix_to_date(start_time_);
            end_time_str = convert_unix_to_date(end_time_);
        }

        std::string url = "https://api.polygon.io/v2/aggs/ticker/" + stock_code_ +
                          "/range/" + std::to_string(time_size) + "/" + timespan + "/" +
                          start_time_str + "/" + end_time_str +
                          "?apiKey=" + api_key_;

        // Print the constructed URL
        RCLCPP_INFO(this->get_logger(), "Constructed URL: %s", url.c_str());

        return url;
    }

    // Parse frequency string into time size (int) and time unit (string)
    bool parse_frequency(const std::string &frequency, int &time_size, std::string &time_unit) {
        std::regex freq_regex(R"((\d+)([a-zA-Z]+))");
        std::smatch match;
        if (std::regex_match(frequency, match, freq_regex)) {
            time_size = std::stoi(match[1].str());
            time_unit = match[2].str();
            return true;
        }
        return false;
    }

    // Helper function to convert Unix timestamp to date in YYYY-MM-DD format
    std::string convert_unix_to_date(int unix_timestamp) const {
        std::time_t t = unix_timestamp;
        std::tm *ptm = std::gmtime(&t);
        char buffer[11];  // YYYY-MM-DD format
        std::strftime(buffer, 11, "%Y-%m-%d", ptm);
        return std::string(buffer);
    }

    void process_and_save_data(const std::string &response) {
    try {
        json j = json::parse(response);

        if (j.contains("results")) {
            auto results = j["results"];
            RCLCPP_INFO(this->get_logger(), "Number of results: %ld", results.size());

            // Create a vector to hold Price data
            std::vector<Price> prices;

            // Extract and format results
            for (auto &result : results) {
                Price price(
                    stock_code_,               // Stock code
                    market_code_,              // Market code
                    result["o"].get<double>(),  // Open price
                    result["c"].get<double>(),  // Close price
                    result["h"].get<double>(),  // High price
                    result["l"].get<double>(),  // Low price
                    result["t"].get<int64_t>(), // Timestamp
                    result["n"].get<double>(),  // Number of trades (count)
                    result["v"].get<double>(),  // Volume (amount)
                    frequency_                 // Frequency
                );

                // Print the price data to the terminal
                RCLCPP_INFO(this->get_logger(),
                    "Stock: %s | Open: %.2f | Close: %.2f | High: %.2f | Low: %.2f | Time: %lld | Trades: %.2f | Volume: %.2f",
                    price.stock_code.c_str(),
                    price.open,
                    price.close,
                    price.high,
                    price.low,
                    static_cast<long long>(price.time),
                    price.count,
                    price.amount
                );

                // Add the price data to the vector for Parquet writing
                prices.push_back(price);
            }

            // Write the price data to a Parquet file
            //data_writer_.writeData(prices);  // Modify this function to handle the Price structure
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unexpected API response: %s", response.c_str());
        }
    } catch (json::parse_error &e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
    }
}


    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    DataWriter data_writer_;  // DataWriter object for writing to Parquet file
    rclcpp::TimerBase::SharedPtr timer_;
    CURL *curl_;
    std::string stock_code_;
    std::string market_code_;  // Optional, might not be used for this API
    int start_time_;  // Unix timestamp
    int end_time_;    // Unix timestamp
    std::string frequency_;  // Frequency (e.g., "5minute", "2week", etc.)
    bool use_millisecond_timestamp_;  // Flag to use millisecond timestamp or not
    std::string api_key_;  // API key for Polygon
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolygonNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/