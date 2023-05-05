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
  val_safety_exception_reporter
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
  CATKIN_DEPENDS roscpp rospy tf geometry_msgs sensor_msgs std_msgs controller_msgs val_safety_exception_reporter
)
include_directories(${catkin_INCLUDE_DIRS})

#------------------------------------------------------------------------
#     Find ROS Services
#     Note:
#         ${CATKIN_DEVEL_PREFIX} is devel/.private/${PROJECT_NAME}
#         services are in devel/include
#         also works: ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#------------------------------------------------------------------------
set(SERVICE_DIR ${CATKIN_DEVEL_PREFIX}/../../${CATKIN_GLOBAL_INCLUDE_DESTINATION})
message("")
message("Service directory: ${SERVICE_DIR}")
message("")
include_directories ("${SERVICE_DIR}")

#------------------------------------------------------------------------
#     Include Directories
#------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/nodes")
include_directories ("${PROJECT_SOURCE_DIR}/srv")

#---------------------------------------------------------------------------
#     Directory Selection
#---------------------------------------------------------------------------
add_subdirectory (nodes)
