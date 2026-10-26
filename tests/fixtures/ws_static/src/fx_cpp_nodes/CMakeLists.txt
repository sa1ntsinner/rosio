cmake_minimum_required(VERSION 3.10)
project(fx_cpp_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fx_msgs REQUIRED)

set(node_deps rclcpp rclcpp_components std_msgs fx_msgs)

add_executable(fx_alpha_node src/fx_alpha_main.cpp)
ament_target_dependencies(fx_alpha_node ${node_deps})

add_library(fx_beta_component SHARED src/fx_beta_node.cpp)
ament_target_dependencies(fx_beta_component rclcpp rclcpp_components fx_msgs)
rclcpp_components_register_nodes(fx_beta_component "fx_cpp_nodes::BetaNode")

add_executable(fx_beta_node src/fx_beta_main.cpp)
target_link_libraries(fx_beta_node fx_beta_component)
ament_target_dependencies(fx_beta_node ${node_deps})

add_library(fx_gamma_component SHARED src/fx_gamma_node.cpp)
ament_target_dependencies(fx_gamma_component rclcpp rclcpp_components)
rclcpp_components_register_node(fx_gamma_component
  PLUGIN "fx_cpp_nodes::GammaNode"
  EXECUTABLE fx_gamma_node)

install(TARGETS
  fx_alpha_node fx_beta_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fx_beta_component fx_gamma_component
  LIBRARY DESTINATION lib)

ament_package()
