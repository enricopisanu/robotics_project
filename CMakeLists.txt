cmake_minimum_required(VERSION 3.0.2)
project(robotics_hw1)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_FLAGS "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  message_filters
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  tf2
  tf2_ros 
)

find_package(yaml-cpp REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   OdometryCustom.msg
   MotorSpeed.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SetOdom.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs   
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
  
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES robotics_hw1
 CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  message_runtime 
  nav_msgs
  message_filters
 DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(baseline_calculation_node src/baseline_calculation_node.cpp)
add_dependencies(baseline_calculation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(baseline_calculation_node ${catkin_LIBRARIES})

add_executable(speed_publisher_node src/publish_speed.cpp)
add_dependencies(speed_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_publisher_node ${catkin_LIBRARIES})

add_executable(odometry_node src/odometry_node.cpp)
add_dependencies(odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_node 
  ${catkin_LIBRARIES}
  ${ROS_LIB_DIR}   
  gflags glog
  yaml-cpp)