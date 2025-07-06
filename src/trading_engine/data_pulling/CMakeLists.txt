cmake_minimum_required(VERSION 3.8)
project(data_pulling)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT APPLE AND NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Check for CCAPI_PATH environment variable
if(DEFINED ENV{CCAPI_PATH})
    set(CCAPI_PROJECT_DIR $ENV{CCAPI_PATH})
    message(STATUS "CCAPI_PROJECT_DIR: ${CCAPI_PROJECT_DIR}")
else()
    message(WARNING "CCAPI_PATH environment variable is not set. Please set it to the CCAPI installation directory or modify the CMakeLists.txt manually to set the CCAPI_PROJECT_DIR.")
    set(CCAPI_PROJECT_DIR "/path/to/your/ccapi")  # Default or placeholder path
endif()

# Check for RAPIDJSON_PATH environment variable
if(DEFINED ENV{RAPIDJSON_PATH})
    set(RAPIDJSON_PATH $ENV{RAPIDJSON_PATH})
    message(STATUS "RAPIDJSON_PATH: ${RAPIDJSON_PATH}")
else()
    message(WARNING "RAPIDJSON_PATH environment variable is not set. Please set it to the RapidJSON installation directory or modify the CMakeLists.txt manually to set the RAPIDJSON_PATH.")
    set(RAPIDJSON_PATH "/path/to/your/rapidjson")  # Default or placeholder path
endif()

# Set the Apache Arrow installation path
set(CMAKE_PREFIX_PATH "/usr/local")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(system_interface REQUIRED)
find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)
find_package(websocketpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_compile_options(-I${CCAPI_PROJECT_DIR}/include)


link_libraries(
    OpenSSL::Crypto
    OpenSSL::SSL
)

# Update include directories
include_directories(
    ${CCAPI_PROJECT_DIR}/include
    ${CCAPI_PROJECT_DIR}/include/ccapi_cpp
    ${ARROW_INCLUDE_DIR}
    ${PARQUET_INCLUDE_DIR}
    ${OPENSSL_INCLUDE_DIR}
    ${CURL_INCLUDE_DIRS}  # Added for Alpha Vantage node
    ${RAPIDJSON_PATH}/include
    ${websocketpp_INCLUDE_DIRS}

)

include_directories(include)

add_compile_options(-Wno-deprecated -Wno-nonnull -Wno-deprecated-declarations)

# Enable multiple exchanges
add_compile_definitions(CCAPI_ENABLE_SERVICE_MARKET_DATA)
add_compile_definitions(CCAPI_ENABLE_SERVICE_EXECUTION_MANAGEMENT)
add_compile_definitions(CCAPI_ENABLE_EXCHANGE_BINANCE)
add_compile_definitions(CCAPI_ENABLE_EXCHANGE_BITMEX)

# If you're using C++17 or later, you might also want to add:
add_compile_definitions(CCAPI_USE_BOOST_THREAD=0)

# Declare executables
add_executable(data_pull src/data_pull.cpp)
add_executable(alpha_vantage_node src/alpha_vantage_node.cpp)
add_executable(polygon_websocket_node src/polygon_websocket_node.cpp)
add_executable(ccapi_bitmex_trading_node src/ccapi_bitmex_trading_node.cpp)
add_executable(ohlcv_processor src/ohlcv_processor.cpp)

# Declare dependencies of executables
ament_target_dependencies(data_pull rclcpp std_msgs system_interface)
ament_target_dependencies(alpha_vantage_node rclcpp std_msgs)
ament_target_dependencies(polygon_websocket_node rclcpp std_msgs)
ament_target_dependencies(ccapi_bitmex_trading_node rclcpp std_msgs)
ament_target_dependencies(ohlcv_processor rclcpp std_msgs system_interface)

ament_export_include_directories(include)


# Link libraries with executables
target_link_libraries(data_pull
    ${ARROW_SHARED_LIB}
    ${PARQUET_SHARED_LIB}
)

target_link_libraries(alpha_vantage_node
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

target_link_libraries(polygon_websocket_node
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    ${OPENSSL_LIBRARIES}
)

target_link_libraries(ccapi_bitmex_trading_node
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    ${OPENSSL_LIBRARIES}
)

target_include_directories(data_pull PRIVATE
    ${CCAPI_PROJECT_DIR}/include
    ${CCAPI_PROJECT_DIR}/include/ccapi_cpp
)

target_include_directories(data_pull PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(alpha_vantage_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(polygon_websocket_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(ccapi_bitmex_trading_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(ohlcv_processor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the executables
install(TARGETS 
    data_pull
    alpha_vantage_node
    polygon_websocket_node
    ccapi_bitmex_trading_node
    ohlcv_processor

    DESTINATION lib/${PROJECT_NAME}
)

# Package configuration
ament_package()