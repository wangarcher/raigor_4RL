cmake_minimum_required(VERSION 3.0.2)
project(tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  geometric_shapes
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  roscpp
  std_msgs
  nav_msgs
)

catkin_package(
)


include_directories(
  include
  include "/usr/local/include/eigen3"
  ${catkin_INCLUDE_DIRS}
)


add_executable(
  tracker 
  src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES})