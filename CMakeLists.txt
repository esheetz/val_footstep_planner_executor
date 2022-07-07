#---------------------------------------------------------------------------
# Dependencies on ROS and IHMC controller_msgs
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(val_footstep_planner_executor)

set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  sensor_msgs
  std_msgs
  controller_msgs
  message_generation
)

#------------------------------------------------------------------------
#     ROS Setup
#------------------------------------------------------------------------
add_subdirectory (srv)

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf geometry_msgs sensor_msgs std_msgs controller_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

#---------------------------------------------------------------------------
# Add subdirectories
#---------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/nodes")
include_directories ("${PROJECT_SOURCE_DIR}/srv")
include_directories (${catkin_INCLUDE_DIRS})
add_subdirectory (nodes)
