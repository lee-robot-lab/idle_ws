cmake_minimum_required(VERSION 3.8)
project(can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(can_bridge_node src/can_bridge_node.cpp)
target_compile_features(can_bridge_node PUBLIC cxx_std_17)
target_include_directories(can_bridge_node PRIVATE include)
ament_target_dependencies(can_bridge_node rclcpp msgs)

install(TARGETS
  can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tx_tick_budget test/test_tx_tick_budget.cpp)
  target_compile_features(test_tx_tick_budget PUBLIC cxx_std_17)
  target_include_directories(test_tx_tick_budget PRIVATE include)

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
