cmake_minimum_required(VERSION 2.8.3)
project(avx_ros_package)

# Compile C++11
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation mavros_msgs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## receiver node
add_executable(receiver_node src/receiver_node.cpp include/receiver_node.h)
target_link_libraries(receiver_node ${catkin_LIBRARIES})
