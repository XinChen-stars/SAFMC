cmake_minimum_required(VERSION 3.0.2)
project(simtools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simtools
#  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(real_gazebo_pose src/real_gazebo_pose.cpp)

target_link_libraries(real_gazebo_pose
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
