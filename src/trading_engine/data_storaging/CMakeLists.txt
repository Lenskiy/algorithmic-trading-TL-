cmake_minimum_required(VERSION 3.8)
project(data_storaging)

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

# Set the Apache Arrow and Parquet installation paths
set(CMAKE_PREFIX_PATH "/usr/local")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(system_interface REQUIRED)
find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)


# Link OpenSSL libraries
link_libraries(
  OpenSSL::Crypto
  OpenSSL::SSL
)

# Include directories
include_directories(${ARROW_INCLUDE_DIR})
include_directories(${PARQUET_INCLUDE_DIR})
include_directories(include)

add_compile_options(-Wno-deprecated -Wno-nonnull -Wno-deprecated-declarations)

# Linter settings
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Declare executable for polygon_node
add_executable(polygon_node src/polygon_node.cpp)

# Declare dependencies for the polygon_node executable
ament_target_dependencies(polygon_node rclcpp std_msgs system_interface rosidl_default_runtime)

# Link required libraries (CURL, nlohmann_json, Arrow, Parquet)
target_link_libraries(polygon_node
  CURL::libcurl
  nlohmann_json::nlohmann_json
  ${ARROW_SHARED_LIB}
  ${PARQUET_SHARED_LIB}
)

# Include directories for polygon_node
target_include_directories(polygon_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the polygon_node executable
install(TARGETS polygon_node
  DESTINATION lib/${PROJECT_NAME})

# Package configuration
ament_export_dependencies(rosidl_default_runtime)
ament_package()
