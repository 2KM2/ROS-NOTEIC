cmake_minimum_required(VERSION 3.0.2)
project(LaserUndistortion)

## ROS Noetic supports C++14.  This project uses C++14 value initialization
## and smart-pointer helpers in the ICP scaffold.
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  champion_nav_msgs
  pcl_ros
  pcl_conversions
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common filters registration)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_icp_scaffold
  CATKIN_DEPENDS roscpp tf sensor_msgs champion_nav_msgs pcl_ros pcl_conversions
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

message(STATUS "PCL include directories: ${PCL_INCLUDE_DIRS}")

## The scaffold keeps registration state, odometry priors, quality gates, and
## point-cloud preprocessing outside the student ICP solver TODO.
add_library(${PROJECT_NAME}_icp_scaffold
  src/icp_odometry.cpp
)

add_dependencies(${PROJECT_NAME}_icp_scaffold ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_icp_scaffold
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Existing homework node.  Its name stays unchanged so the launch file works.
add_executable(${PROJECT_NAME}_node src/LidarMotionUndistortion.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_icp_scaffold
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_icp_scaffold
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install public scaffold interfaces for later ICP / point-to-line extensions.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Install launch files and the RViz configuration with the package.
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_LaserUndistortion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
