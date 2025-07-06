cmake_minimum_required(VERSION 3.8)
project(accounting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(system_interface REQUIRED)

include_directories(${ARROW_INCLUDE_DIR})

# build accounting
add_executable(accounting src/accounting.cpp)

target_link_libraries(accounting)

target_include_directories(accounting PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(accounting PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  accounting
  "rclcpp"
  "system_interface"
)


install(TARGETS accounting
  DESTINATION lib/${PROJECT_NAME})


# build logging
add_executable(logging src/logging)

target_link_libraries(logging)

target_include_directories(logging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(logging PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  logging
  "rclcpp"
  "system_interface"
)


install(TARGETS logging
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
