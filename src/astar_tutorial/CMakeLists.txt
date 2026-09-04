cmake_minimum_required(VERSION 3.0.2)
project(astar_tutorial)

# C++17：算法核心只用 STL，不牵扯 PCL，所以这里不受 grid_path_searcher 那个
# "PCL 1.10 必须 c++14" 的限制。
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
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astar_core
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 算法核心：只依赖 STL，不链接任何 ROS 库。
# 所以它也可以脱离 catkin 单独编译（见 README 的"不用 ROS 也能跑"一节）。
# ---------------------------------------------------------------------------
add_library(astar_core
  src/grid_map_2d.cpp
  src/astar.cpp
  src/trace_replayer.cpp
)

# ---------------------------------------------------------------------------
# rviz 可视化节点
# ---------------------------------------------------------------------------
add_executable(astar_rviz_node src/astar_rviz_node.cpp)
target_link_libraries(astar_rviz_node astar_core ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# 键盘遥控器（只用 roscpp，不用算法库）
# ---------------------------------------------------------------------------
add_executable(astar_keyboard src/astar_keyboard.cpp)
target_link_libraries(astar_keyboard ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# 纯终端 demo 和自检：不链 ROS
# ---------------------------------------------------------------------------
add_executable(astar_console_demo src/astar_console_demo.cpp)
target_link_libraries(astar_console_demo astar_core)

add_executable(astar_selfcheck src/astar_selfcheck.cpp)
target_link_libraries(astar_selfcheck astar_core)

install(TARGETS astar_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS astar_rviz_node astar_keyboard astar_console_demo astar_selfcheck
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
