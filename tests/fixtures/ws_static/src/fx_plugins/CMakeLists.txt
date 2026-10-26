cmake_minimum_required(VERSION 3.10)
project(fx_plugins)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(fx_plugin_layers SHARED
  src/decay_layer.cpp
  src/static_layer.cpp)
ament_target_dependencies(fx_plugin_layers rclcpp pluginlib)

pluginlib_export_plugin_description_file(fx_core fx_layers_plugin.xml)
# fx_tools_plugin.xml is exported here only, never in package.xml.
pluginlib_export_plugin_description_file(fx_tools fx_tools_plugin.xml)

install(TARGETS fx_plugin_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES fx_layers_plugin.xml fx_tools_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
