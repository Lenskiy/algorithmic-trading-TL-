#include "rclcpp/rclcpp.hpp"
#include "system_interface/srv/sql_query.hpp"
#include "system_interface/srv/run_backtesting.hpp"

class CommandLineInterface : public rclcpp::Node {
public:
    CommandLineInterface(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "The node has successfully launched : %s", name.c_str());
        
        cli_thread_ = std::thread(&CommandLineInterface::handle_cli_input, this);

        client_ = this->create_client<system_interface::srv::SqlQuery>("database_query");
        run_backtesting_client = this->create_client<system_interface::srv::RunBacktesting>("run_backtesting");
    }

    ~CommandLineInterface() {
        if (cli_thread_.joinable()) {
            cli_thread_.join();
        }
    }

private:
    std::thread cli_thread_;

    rclcpp::Client<system_interface::srv::SqlQuery>::SharedPtr client_;
    rclcpp::Client<system_interface::srv::RunBacktesting>::SharedPtr run_backtesting_client;

    void handle_cli_input() {
        while (rclcpp::ok()) {
            std::string input;
            std::getline(std::cin, input);

            if (input == "exit") {
                std::cout << "Exiting CLI..." << std::endl;
                rclcpp::shutdown();
                break;
            } 
            else if (input.find("database_test") != std::string::npos) {
                std::string query = input.substr(input.find(" ") + 1);
                send_query(query);
            }
            else if (input.find("run_backtesting") != std::string::npos) {
                std::string query = input.substr(input.find(" ") + 1);
                std::istringstream stream(query);
                std::string word;
                std::vector<std::string> words;

                while (stream >> word) {
                    words.push_back(word);
                }

                if(words.size() == 2) {
                    send_query_backtesting(words);
                }
                else {
                    std::cout << "Unknown command: " << input << std::endl;
                }
            } 
            else {
                std::cout << "Unknown command: " << input << std::endl;
            }
        }
    }

    void send_query(const std::string &query) {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }

        auto request = std::make_shared<system_interface::srv::SqlQuery::Request>();
        request->query = query;

        auto result = client_->async_send_request(
            request, 
            std::bind(&CommandLineInterface::sql_query_result_callback, this, std::placeholders::_1)
        );
    }

    void sql_query_result_callback(rclcpp::Client<system_interface::srv::SqlQuery>::SharedFuture sql_query_result_future) {
        auto response = sql_query_result_future.get();
        for (const auto& line : response->result) {
            std::cout << line << std::endl; 
        }
    }

    void send_query_backtesting(const std::vector<std::string> &query) {
        while (!run_backtesting_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }

        auto request = std::make_shared<system_interface::srv::RunBacktesting::Request>();
        request->bt_type = query[0];
        request->bt_params_json = query[1];

        auto result = run_backtesting_client->async_send_request(
            request, 
            std::bind(&CommandLineInterface::run_backtesting_result_callback, this, std::placeholders::_1)
        );
    }

    void run_backtesting_result_callback(rclcpp::Client<system_interface::srv::RunBacktesting>::SharedFuture run_backtesting_result_future) {
        auto response = run_backtesting_result_future.get();

        std::cout << response->done << std::endl;
        std::cout << response->reason << std::endl;
    }

};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CommandLineInterface>("command_line_interface");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

