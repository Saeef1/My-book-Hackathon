cmake_minimum_required(VERSION 3.8)
project(subscriber_pkg)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(subscriber_node_cpp src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node_cpp rclcpp std_msgs)

install(TARGETS
  subscriber_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs)

ament_package()
