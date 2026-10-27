cmake_minimum_required(VERSION 3.8)
project(deck)
generate_parameter_library(${PROJECT_NAME}_parameters
  param/deck_parameters.yaml
)
add_executable(deck src/main.cpp src/deck.cpp src/optimizer.cpp)
target_include_directories(deck PUBLIC include)
target_link_libraries(deck ${PROJECT_NAME}_parameters)
install(TARGETS deck DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
