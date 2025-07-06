cmake_minimum_required(VERSION 3.8)
project(data_storage)

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



# Declare executable
add_executable(write_data src/write_data.cpp)
add_executable(read_data src/read_data.cpp)

# Declare dependencies of executable
ament_target_dependencies(write_data rclcpp std_msgs system_interface)
ament_target_dependencies(read_data rclcpp std_msgs system_interface)
ament_export_include_directories(include)

# Link libraries with executable
target_link_libraries(write_data ${ARROW_SHARED_LIB} ${PARQUET_SHARED_LIB})
target_include_directories(write_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(read_data ${ARROW_SHARED_LIB} ${PARQUET_SHARED_LIB})
target_include_directories(read_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



# Install the executable
install(TARGETS write_data DESTINATION lib/${PROJECT_NAME})
install(TARGETS read_data DESTINATION lib/${PROJECT_NAME})

# Package configuration
ament_package()