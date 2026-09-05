cmake_minimum_required(VERSION 3.0.2)
project(grid_search_practice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  visualization_msgs
  astar_tutorial
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_search_core
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs astar_tutorial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 算法核心 + 地图辅助：只依赖 STL 和 astar_tutorial 的 GridMap2D
add_library(grid_search_core
  src/helpers.cpp
  src/your_search.cpp
)
target_link_libraries(grid_search_core ${catkin_LIBRARIES})

add_executable(practice_rviz_node src/rviz_node.cpp)
target_link_libraries(practice_rviz_node grid_search_core ${catkin_LIBRARIES})

add_executable(practice_keyboard src/keyboard.cpp)
target_link_libraries(practice_keyboard ${catkin_LIBRARIES})

add_executable(practice_selfcheck src/selfcheck.cpp)
target_link_libraries(practice_selfcheck grid_search_core ${catkin_LIBRARIES})

install(TARGETS grid_search_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS practice_rviz_node practice_keyboard practice_selfcheck
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
