#include "rclcpp/rclcpp.hpp"
#include "system_interface/msg/order.hpp"
#include "system_interface/msg/log_array.hpp"
#include "system_interface/srv/sql_query.hpp"
#include <mysql/mysql.h>

class DatabaseTest : public rclcpp::Node {
public:
    DatabaseTest(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "The node has successfully launched : %s", name.c_str());

        connect_to_database();

        callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        auto store_order_subscriber_options = rclcpp::SubscriptionOptions();
        store_order_subscriber_options.callback_group = callback_group_reentrant;

        auto store_log_subscriber_options = rclcpp::SubscriptionOptions();
        store_log_subscriber_options.callback_group = callback_group_reentrant;

        store_order_subscriber = this->create_subscription<system_interface::msg::Order>(
            "store_order",
            1000,
            std::bind(&DatabaseTest::store_order_callback, this, std::placeholders::_1),
            store_order_subscriber_options
        );

        store_log_subscriber = this->create_subscription<system_interface::msg::LogArray>(
            "store_log",
            100,
            std::bind(&DatabaseTest::store_log_callback, this, std::placeholders::_1),
            store_log_subscriber_options
        );

        sql_service_ = this->create_service<system_interface::srv::SqlQuery>(
            "database_query",
            std::bind(&DatabaseTest::handle_query_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_default,
            callback_group_reentrant
        );

    }

    ~DatabaseTest() {
        if (conn != nullptr) {
            mysql_close(conn);
            RCLCPP_INFO(this->get_logger(), "MySQL connection closed.");
        }
    }

private:
    rclcpp::Subscription<system_interface::msg::Order>::SharedPtr store_order_subscriber;
    rclcpp::Subscription<system_interface::msg::LogArray>::SharedPtr store_log_subscriber;

    rclcpp::Service<system_interface::srv::SqlQuery>::SharedPtr sql_service_;

    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant;

    MYSQL *conn;

    void connect_to_database() {
        conn = mysql_init(nullptr);
        if (conn == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "mysql_init() failed.");
            return;
        }

        if (mysql_real_connect(conn, "localhost", "root", "Cjm.0610", 
                               "database_test", 0, nullptr, 0) == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "MySQL connection failed: %s", mysql_error(conn));
            mysql_close(conn);
            conn = nullptr;
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "MySQL connection successful.");
        }
    }

    void store_order_callback(const system_interface::msg::Order::SharedPtr order) {
        RCLCPP_INFO(this->get_logger(), "Received order");

        if (conn == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No MySQL connection.");
            return;
        }

        std::stringstream ss;
        ss << "INSERT INTO orders (symbol, exchange, price, size, side, time) "
           << "VALUES ('" << order->symbol << "', '"
           << order->exchange << "', "
           << order->price << ", "
           << order->size << ", '"
           << order->side << "', "
           << order->time << ")";

        std::string query = ss.str();

        if (mysql_query(conn, query.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "MySQL query failed: %s", mysql_error(conn));
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "Order inserted into database.");
        }
    }

    void store_log_callback(const system_interface::msg::LogArray::SharedPtr logs) {
        RCLCPP_INFO(this->get_logger(), "Received log_array");

        if (conn == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No MySQL connection.");
            return;
        }

        for (const auto& log : logs->log_array) {
            std::stringstream ss;
            ss << "INSERT INTO logs (time, event, info) "
               << "VALUES (" << log.time << ", '"
               << log.event << "', '"
               << log.info << "')";

            std::string query = ss.str();

            if (mysql_query(conn, query.c_str())) {
                RCLCPP_ERROR(this->get_logger(), "MySQL query failed: %s", mysql_error(conn));
            } 
            else {
                RCLCPP_INFO(this->get_logger(), "Log inserted into database.");
            }
        }
    }

    void handle_query_request(
        const std::shared_ptr<system_interface::srv::SqlQuery::Request> request,
        std::shared_ptr<system_interface::srv::SqlQuery::Response> response) {
        
        std::string query = request->query;

        if (conn == nullptr) {
            response->result.push_back("Database connection not available.");
            return;
        }

        if (mysql_query(conn, query.c_str())) {
            response->result.push_back("MySQL query failed: " + std::string(mysql_error(conn)));
            return;
        }

        MYSQL_RES *res = mysql_store_result(conn);
        if (res == nullptr) {
            response->result.push_back("No result: " + std::string(mysql_error(conn)));
            return;
        }

        int num_fields = mysql_num_fields(res);
        MYSQL_ROW row;
        
        while ((row = mysql_fetch_row(res))) {
            std::stringstream ss;
            for (int i = 0; i < num_fields; i++) {
                ss << (row[i] ? row[i] : "NULL") << "\t";
            }
            response->result.push_back(ss.str()); 
        }

        mysql_free_result(res);
    }
};



int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DatabaseTest>("database_test");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

 