cmake_minimum_required(VERSION 3.8)
project(trade_strategy)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(system_interface REQUIRED)
find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)

add_executable(strategy1 src/strategy1.cpp)
add_executable(strategy2 src/strategy2.cpp)

target_include_directories(strategy1 PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${system_interface_INCLUDE_DIRS}
  ${Arrow_INCLUDE_DIRS}
  ${Parquet_INCLUDE_DIRS}
)

target_include_directories(strategy2 PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${system_interface_INCLUDE_DIRS}
  ${Arrow_INCLUDE_DIRS}
  ${Parquet_INCLUDE_DIRS}
)

target_link_libraries(strategy1
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${system_interface_LIBRARIES}
  ${Arrow_LIBRARIES}
  ${Parquet_LIBRARIES}
)

target_link_libraries(strategy2
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${system_interface_LIBRARIES}
  ${Arrow_LIBRARIES}
  ${Parquet_LIBRARIES}
)

ament_target_dependencies(strategy1
  rclcpp
  std_msgs
  system_interface
  Arrow
  Parquet
)

ament_target_dependencies(strategy2
  rclcpp
  std_msgs
  system_interface
  Arrow
  Parquet
)

install(TARGETS
  strategy1
  strategy2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()