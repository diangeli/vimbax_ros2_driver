cmake_minimum_required(VERSION 3.16)
project(camera_recorder LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(recorder_component SHARED
  src/bag_uri.cpp
  src/recorder_node.cpp)
target_include_directories(recorder_component PUBLIC include)
ament_target_dependencies(recorder_component
  rclcpp rclcpp_components rosbag2_cpp rosbag2_storage sensor_msgs)
rclcpp_components_register_node(recorder_component
  PLUGIN "camera_recorder::RecorderNode"
  EXECUTABLE recorder_node)

install(TARGETS recorder_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bounded_queue test/test_bounded_queue.cpp)
  target_include_directories(test_bounded_queue PRIVATE include)
  ament_add_gtest(test_bag_uri test/test_bag_uri.cpp src/bag_uri.cpp)
  target_include_directories(test_bag_uri PRIVATE include)
endif()

ament_package()
