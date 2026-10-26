project(argument_nodes)
include_directories(include)
add_executable(header_default src/header_main.cpp)
add_executable(caller src/caller.cpp)
add_executable(ambiguous src/ambiguous.cpp)
add_library(talker_component SHARED src/component.cpp)
rclcpp_components_register_node(talker_component
  PLUGIN "argument_nodes::Talker" EXECUTABLE component_talker)
install(TARGETS caller ambiguous DESTINATION lib/${PROJECT_NAME})
