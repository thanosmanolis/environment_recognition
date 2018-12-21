cmake_minimum_required(VERSION 2.8.3)
project(env_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES env_recognition
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(recognition 
  src/recognition_node.cpp
  src/recognition.cpp)
target_link_libraries(recognition 
  ${catkin_LIBRARIES})
add_dependencies(recognition 
  ${catkin_EXPORTED_TARGETS})
