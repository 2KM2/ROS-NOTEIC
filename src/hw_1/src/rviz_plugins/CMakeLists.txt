cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rviz
  multi_map_server
  roscpp
  nav_msgs
  tf
  pluginlib
)

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
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rviz multi_map_server roscpp nav_msgs tf pluginlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
#find_package(Qt5 COMPONENTS QtCore QtGui REQUIRED)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt5_wrap_cpp(MOC_FILES
  src/goal_tool.h
  src/probmap_display.h
  src/aerialmap_display.h
  src/multi_probmap_display.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/probmap_display.cpp
  src/aerialmap_display.cpp
  src/multi_probmap_display.cpp
  ${MOC_FILES}
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

# multi_map_server 的消息头文件是生成的,必须先生成再编译本包
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# media/ 和 icons/ 这两个目录本包里没有,install(DIRECTORY) 对不存在的目录会让
# `catkin_make install` 直接报 CMake Error,所以加存在性判断。
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/media)
  install(DIRECTORY media/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/icons)
  install(DIRECTORY icons/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
endif()
