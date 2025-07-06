#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <websocketpp/common/asio.hpp>
#include <websocketpp/common/functional.hpp>

using json = nlohmann::json;

typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class PolygonWebSocketNode : public rclcpp::Node {
public:
    PolygonWebSocketNode() : Node("polygon_websocket_node") {
        this->declare_parameter("api_key", "");
        this->declare_parameter("symbols", std::vector<std::string>{"AAPL"});

        api_key_ = this->get_parameter("api_key").as_string();
        symbols_ = this->get_parameter("symbols").as_string_array();

        publisher_ = this->create_publisher<std_msgs::msg::String>("stock_data", 10);

        c.clear_access_channels(websocketpp::log::alevel::all);
        c.set_access_channels(websocketpp::log::alevel::connect);
        c.set_access_channels(websocketpp::log::alevel::disconnect);
        c.set_access_channels(websocketpp::log::alevel::app);
        c.clear_error_channels(websocketpp::log::elevel::all);
        c.set_error_channels(websocketpp::log::elevel::fatal);

        c.init_asio();

        c.set_tls_init_handler(bind(&PolygonWebSocketNode::on_tls_init, this, ::_1));
        c.set_message_handler(bind(&PolygonWebSocketNode::on_message, this, ::_1, ::_2));
        c.set_open_handler(bind(&PolygonWebSocketNode::on_open, this, ::_1));
        c.set_close_handler(bind(&PolygonWebSocketNode::on_close, this, ::_1));
        c.set_fail_handler(bind(&PolygonWebSocketNode::on_fail, this, ::_1));

        websocketpp::lib::error_code ec;
        connection = c.get_connection("wss://delayed.polygon.io/stocks", ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Could not create connection: %s", ec.message().c_str());
            return;
        }

        c.connect(connection);

        websocket_thread_ = std::thread([this]() { c.run(); });
    }

    ~PolygonWebSocketNode() {
        c.close(connection, websocketpp::close::status::normal, "Closing connection");
        if (websocket_thread_.joinable()) {
            websocket_thread_.join();
        }
    }

private:
    websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> on_tls_init(websocketpp::connection_hdl) {
        auto ctx = websocketpp::lib::make_shared<websocketpp::lib::asio::ssl::context>(websocketpp::lib::asio::ssl::context::sslv23);
        ctx->set_options(websocketpp::lib::asio::ssl::context::default_workarounds |
                         websocketpp::lib::asio::ssl::context::no_sslv2 |
                         websocketpp::lib::asio::ssl::context::no_sslv3 |
                         websocketpp::lib::asio::ssl::context::single_dh_use);
        return ctx;
    }

    void on_open(websocketpp::connection_hdl hdl) {
        RCLCPP_INFO(this->get_logger(), "Connection opened. Authenticating and subscribing...");
        authenticate();
    }

    void on_close(websocketpp::connection_hdl hdl) {
        RCLCPP_INFO(this->get_logger(), "Connection closed");
    }

    void on_fail(websocketpp::connection_hdl hdl) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
    }

    void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
        try {
            json j = json::parse(msg->get_payload());
            RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->get_payload().c_str());
            if (j.contains("ev") && j["ev"] == "T") {
                auto trade = j;
                auto message = std_msgs::msg::String();
                message.data = 
                    "stock_code: " + trade["sym"].get<std::string>() + "\n" +
                    "price: " + std::to_string(trade["p"].get<double>()) + "\n" +
                    "size: " + std::to_string(trade["s"].get<int>()) + "\n" +
                    "timestamp: " + std::to_string(trade["t"].get<long long>()) + "\n";
                
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published data for %s", trade["sym"].get<std::string>().c_str());
            } else if (j.contains("status")) {
                RCLCPP_INFO(this->get_logger(), "Received status message: %s", j["status"].get<std::string>().c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing message: %s", e.what());
        }
    }

    void authenticate() {
        json auth_message = {
            {"action", "auth"},
            {"params", api_key_}
        };
        connection->send(auth_message.dump());
        RCLCPP_INFO(this->get_logger(), "Sent authentication message: %s", auth_message.dump().c_str());
        
        std::string params = custom_join(symbols_, ",T.");
        json subscribe_message = {
            {"action", "subscribe"},
            {"params", "T." + params}
        };
        connection->send(subscribe_message.dump());
        RCLCPP_INFO(this->get_logger(), "Sent subscription message: %s", subscribe_message.dump().c_str());
    }

    // Custom join function
    template<typename Container>
    std::string custom_join(const Container& container, const std::string& delim) {
        std::ostringstream oss;
        auto it = container.begin();
        if (it != container.end()) {
            oss << *it++;
        }
        while (it != container.end()) {
            oss << delim << *it++;
        }
        return oss.str();
    }

    client c;
    client::connection_ptr connection;
    std::thread websocket_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string api_key_;
    std::vector<std::string> symbols_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolygonWebSocketNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}