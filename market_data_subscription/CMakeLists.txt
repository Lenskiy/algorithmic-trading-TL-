cmake_minimum_required(VERSION 3.14)
set(NAME market_data_subscription)
project(${NAME})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT APPLE AND NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

if(NOT CCAPI_LEGACY_USE_WEBSOCKETPP)
  message(STATUS "Using Boost Beast for WebSocket")
  include(ExternalProject)
  if(NOT BOOST_INCLUDE_DIR)
    ExternalProject_Add(boost
      URL https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz
      SOURCE_DIR "${CMAKE_BINARY_DIR}/boost"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      TEST_COMMAND ""
    )
    set(BOOST_INCLUDE_DIR "${CMAKE_BINARY_DIR}/boost")
  endif()

  if(NOT RAPIDJSON_INCLUDE_DIR)
    ExternalProject_Add(rapidjson
      URL https://github.com/Tencent/rapidjson/archive/refs/tags/v1.1.0.tar.gz
      SOURCE_DIR "${CMAKE_BINARY_DIR}/rapidjson"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      TEST_COMMAND ""
    )
    set(RAPIDJSON_INCLUDE_DIR "${CMAKE_BINARY_DIR}/rapidjson/include")
  endif()

  if(NOT HFFIX_INCLUDE_DIR)
    ExternalProject_Add(hffix
      URL https://github.com/jamesdbrock/hffix/archive/refs/tags/v1.3.0.tar.gz
      SOURCE_DIR "${CMAKE_BINARY_DIR}/hffix"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      TEST_COMMAND ""
    )
    set(HFFIX_INCLUDE_DIR "${CMAKE_BINARY_DIR}/hffix/include")
  endif()

else()
  message(STATUS "Using legacy WebSocket++")
  set(RAPIDJSON_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/rapidjson/include)
  set(WEBSOCKETPP_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/websocketpp)
  set(BOOST_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/boost)
  set(HFFIX_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/hffix/include)
endif() 

include_directories(
  ${CCAPI_PROJECT_DIR}/include
  ${BOOST_INCLUDE_DIR}
  ${RAPIDJSON_INCLUDE_DIR}
  ${HFFIX_INCLUDE_DIR}
  ${OPENSSL_INCLUDE_DIR}
)

find_package(OpenSSL REQUIRED)
link_libraries(
  OpenSSL::Crypto
  OpenSSL::SSL
)

add_compile_options(-Wno-deprecated -Wno-nonnull -Wno-deprecated-declarations)

get_filename_component(CCAPI_PROJECT_DIR ../ ABSOLUTE)
message(STATUS "CCAPI_PROJECT_DIR: ${CCAPI_PROJECT_DIR}")
# Enable multiple exchanges
add_compile_definitions(CCAPI_ENABLE_SERVICE_MARKET_DATA)
add_compile_definitions(CCAPI_ENABLE_EXCHANGE_BINANCE)
add_compile_definitions(CCAPI_ENABLE_EXCHANGE_BITMEX)

add_executable(${NAME} main.cpp)

# Ensure the external projects are built before your executable
add_dependencies(${NAME} boost rapidjson)
include_directories($ENV{HOME}/Desktop/ccapi/include)