cmake_minimum_required(VERSION 3.8)
project(system_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/TemplateInfo.msg"
"msg/Log.msg"
"msg/LogArray.msg"
"srv/AddTwoInt.srv"
"srv/SqlQuery.srv"
"msg/Price.msg"
"msg/Order.msg"
"msg/TickData.msg"
"srv/GetHistoricalPrices.srv"
"srv/GetHistoricalTickDatas.srv"
"srv/RunBacktesting.srv"
"srv/SetAgent.srv"
"srv/CloseAgent.srv"
"srv/OrderRisk.srv"
"srv/CalculateCorrelation.srv"
"srv/CalculateLiquidityRisk.srv"
"srv/CalculatePriceRisk.srv"
"action/Progress.action"
"action/OtherAction.action"
DEPENDENCIES std_msgs
)

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