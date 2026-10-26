cmake_minimum_required(VERSION 3.10)
project(fx_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fx_other_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FxPoint.msg"
  "msg/FxState.msg"
  "msg/FxArray.msg"
  "srv/FxConfigure.srv"
  "action/FxDock.action"
  DEPENDENCIES builtin_interfaces std_msgs fx_other_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
