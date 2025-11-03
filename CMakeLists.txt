cmake_minimum_required(VERSION 3.10)
project(known_size_rect_ght_scan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(known_size_rect_ght_scan_node src/known_size_rect_ght_scan_node.cpp)
ament_target_dependencies(known_size_rect_ght_scan_node
  rclcpp sensor_msgs visualization_msgs geometry_msgs)

install(TARGETS known_size_rect_ght_scan_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

