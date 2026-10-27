cmake_minimum_required(VERSION 3.8)
project(deck_fit)
add_library(deck_fit src/fitter.cpp)
target_include_directories(deck_fit PUBLIC include)
