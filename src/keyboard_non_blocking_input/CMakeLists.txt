cmake_minimum_required(VERSION 2.8.3)
project(keyboard_non_blocking_input)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES keyboard_non_blocking_input
  CATKIN_DEPENDS 
	roscpp 
	rospy 
	std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

add_executable(keyboard_non_blocking_input_node src/keyboard_non_blocking_input_node.cpp)
target_link_libraries(keyboard_non_blocking_input_node ${catkin_LIBRARIES})
add_dependencies(keyboard_non_blocking_input_node std_msgs)
