#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <iostream>
#include <string>

using std::to_string;
using std::string;
namespace ssl = boost::asio::ssl;             
using tcp = boost::asio::ip::tcp;               
namespace websocket = boost::beast::websocket; 

int main() {
    try {
        string host = "www.bitmex.com";
        string port = "443";
        string text = R"({"op": "subscribe", "args": ["trade:XBTUSD"]})";


        boost::asio::io_context ioc;

        ssl::context ctx{ssl::context::tlsv12_client};

        ctx.set_default_verify_paths();

        tcp::resolver resolver{ioc};
        websocket::stream<boost::asio::ssl::stream<tcp::socket>> ws{ioc, ctx};

        auto const results = resolver.resolve(host, port);
        auto ep = boost::asio::connect(ws.next_layer().next_layer(), results);

        if(!SSL_set_tlsext_host_name(ws.next_layer().native_handle(), host.c_str()))
        {
            boost::system::error_code ec{static_cast<int>(::ERR_get_error()), boost::asio::error::get_ssl_category()};
            throw boost::system::system_error{ec};
        }

        host += ':' + to_string(ep.port());

        ws.next_layer().handshake(ssl::stream_base::client);

        ws.set_option(websocket::stream_base::decorator(
            [](websocket::request_type& req) {
                req.set(boost::beast::http::field::user_agent,
                    string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-client-coro");
            }));

        ws.handshake(host, "/realtime");

        ws.write(boost::asio::buffer(string(text)));

        boost::beast::multi_buffer buffer;
        while (true) { 
            try {
                ws.read(buffer);
                string message = boost::beast::buffers_to_string(buffer.data());
                std::cout << "Received: " << message << std::endl;
                buffer.consume(buffer.size());
            } catch (std::exception const& e) {
                std::cerr << "Read Error: " << e.what() << std::endl;
                break; 
            }
        }
    } catch (std::exception const& e) {
        std::cerr << "Connection Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
