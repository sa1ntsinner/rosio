cmake_minimum_required(VERSION 3.8)
project(lx_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(lx_controller src/lx_controller.cpp)
ament_target_dependencies(lx_controller rclcpp)

install(TARGETS lx_controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
