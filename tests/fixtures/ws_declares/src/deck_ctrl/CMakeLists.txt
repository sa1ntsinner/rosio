cmake_minimum_required(VERSION 3.8)
project(deck_ctrl)
add_library(deck_ctrl src/controller.cpp)
target_include_directories(deck_ctrl PUBLIC include)
