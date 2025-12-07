cmake_minimum_required(VERSION 3.8)
project(navigate_action_client_pkg)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(navigate_action_client_cpp src/navigate_action_client.cpp)
ament_target_dependencies(navigate_action_client_cpp rclcpp rclcpp_action example_interfaces)

install(TARGETS
  navigate_action_client_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp rclcpp_action example_interfaces)

ament_package()
