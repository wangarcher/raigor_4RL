cmake_minimum_required(VERSION 2.8.3)
project(raigor_pid_control)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf_conversions
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall")


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cpr_closed_loop_controller
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


add_executable(pid_controller_node
                ${SRC_DIR}/pid_controller_node.cpp
                ${SRC_DIR}/PIDController.cpp
                ${INCLUDE_DIR}/PIDController.h)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pid_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(cpr_record_demo_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#
#add_dependencies(cpr_ee_state_splitter_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
target_link_libraries(pid_controller_node
   ${catkin_LIBRARIES}
)

