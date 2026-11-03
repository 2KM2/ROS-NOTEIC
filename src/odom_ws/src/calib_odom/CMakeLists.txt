cmake_minimum_required(VERSION 3.0.2)
project(calib_odom)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CSM REQUIRED csm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_core
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    message_filters
    nav_core
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CSM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${CSM_LIBRARY_DIRS})

add_executable(calib_odom_node
  src/Odom_Calib.cpp
  src/main.cpp
)

add_dependencies(calib_odom_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(calib_odom_node
  ${catkin_LIBRARIES}
  ${CSM_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(odom_calib_test
    test/Odom_Calib_test.cpp
    src/Odom_Calib.cpp
  )
  if(TARGET odom_calib_test)
    target_link_libraries(odom_calib_test
      gtest_main
      ${catkin_LIBRARIES}
    )
  endif()
endif()
