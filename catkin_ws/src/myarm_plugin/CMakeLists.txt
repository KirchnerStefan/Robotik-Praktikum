cmake_minimum_required(VERSION 2.8.3)
project(myarm_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

# Find Gazebo

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(myarm_plugin src/myarm_plugin.cc)
target_link_libraries(myarm_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

catkin_package(
  DEPENDS
	roscpp
	std_msgs
	gazebo_ros
)

