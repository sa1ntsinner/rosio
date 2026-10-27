cmake_minimum_required(VERSION 3.8)
project(helped)
add_library(${PROJECT_NAME}_lib
  src/configurable.cpp
  src/driver.cpp
  src/host.cpp
  src/imu.cpp
  src/odometry.cpp
  src/stray.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include)
add_executable(pair src/main.cpp)
target_link_libraries(pair ${PROJECT_NAME}_lib)
install(TARGETS pair DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
