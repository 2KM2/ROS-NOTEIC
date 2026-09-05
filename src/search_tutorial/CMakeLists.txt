cmake_minimum_required(VERSION 3.0.2)
project(search_tutorial)

# C++17：算法核心只用 STL，不牵扯 PCL，所以不受 grid_path_searcher 那个
# "PCL 1.10 必须 c++14" 的限制。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# astar_tutorial 是**编译期**依赖：
#   * 栅格地图 GridMap2D 直接复用它的那份，不重复实现一遍；
#   * console_demo 的对比表和 selfcheck 的交叉验证要拿它的 A* 当参照物
#     （A*(h≡0) 就是 Dijkstra，两份独立实现给出同一个代价才敢说结果是最优的）。
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
  LIBRARIES search_core
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs astar_tutorial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 算法核心：只依赖 STL + astar_tutorial 的 astar_core（为了 GridMap2D），
# 不链接任何 ROS 库。所以它也可以脱离 catkin 单独编译，见 README 的
# "不用 ROS 也能跑"一节。
# ---------------------------------------------------------------------------
add_library(search_core
  src/graph_search.cpp
  src/trace_replayer.cpp
)
target_link_libraries(search_core ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# rviz 可视化节点：在 rviz 里一步一步看 BFS / DFS / Dijkstra
# ---------------------------------------------------------------------------
add_executable(search_rviz_node src/search_rviz_node.cpp)
target_link_libraries(search_rviz_node search_core ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# 键盘遥控器（只用 roscpp，不碰算法库）
# ---------------------------------------------------------------------------
add_executable(search_keyboard src/search_keyboard.cpp)
target_link_libraries(search_keyboard ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# 纯终端 demo 和自检：不需要 roscore
# ---------------------------------------------------------------------------
add_executable(search_console_demo src/search_console_demo.cpp)
target_link_libraries(search_console_demo search_core ${catkin_LIBRARIES})

add_executable(search_selfcheck src/search_selfcheck.cpp)
target_link_libraries(search_selfcheck search_core ${catkin_LIBRARIES})

install(TARGETS search_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS search_rviz_node search_keyboard search_console_demo search_selfcheck
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
