cmake_minimum_required(VERSION 3.10)
project(cx_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cx_core REQUIRED)

set(common_deps
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs visualization_msgs)

include_directories(include)

add_executable(cx_literal_node src/cx_literal_endpoints.cpp)
ament_target_dependencies(cx_literal_node ${common_deps})

add_executable(cx_param_node src/cx_param_endpoints.cpp)
ament_target_dependencies(cx_param_node ${common_deps})

add_executable(cx_tf_node src/cx_tf_node.cpp)
ament_target_dependencies(cx_tf_node ${common_deps} tf2_ros)

add_executable(cx_lifecycle_node src/cx_lifecycle_node.cpp)
ament_target_dependencies(cx_lifecycle_node ${common_deps} rclcpp_lifecycle tf2_ros)

add_executable(cx_wrappers_node src/cx_wrappers.cpp)
ament_target_dependencies(cx_wrappers_node ${common_deps} diagnostic_updater message_filters)

add_library(cx_component SHARED src/cx_component_node.cpp)
ament_target_dependencies(cx_component rclcpp rclcpp_components visualization_msgs)
rclcpp_components_register_node(cx_component
  PLUGIN "cx_nodes::MarkerComponent"
  EXECUTABLE cx_marker_node)

add_library(cx_nodes_layers SHARED src/cx_layer_plugin.cpp)
ament_target_dependencies(cx_nodes_layers rclcpp pluginlib sensor_msgs cx_core)
pluginlib_export_plugin_description_file(cx_core cx_layers_plugin.xml)

install(TARGETS
  cx_literal_node cx_param_node cx_tf_node cx_lifecycle_node cx_wrappers_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS cx_component cx_nodes_layers
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()
