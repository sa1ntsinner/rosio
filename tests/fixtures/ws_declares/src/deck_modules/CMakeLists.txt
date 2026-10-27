cmake_minimum_required(VERSION 3.8)
project(deck_modules)
add_library(deck_modules SHARED src/stop.cpp src/slow.cpp)
pluginlib_export_plugin_description_file(deck plugins.xml)
