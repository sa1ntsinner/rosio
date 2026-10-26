cmake_minimum_required(VERSION 3.10)
project(fx_desc)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf xacro meshes DESTINATION share/${PROJECT_NAME})

ament_package()
