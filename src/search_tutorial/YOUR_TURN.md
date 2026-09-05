# 你来写：BFS / DFS / Dijkstra

RViz、键盘、地图、trace 回放都还在。被挖空的只有两处算法核心。

## 写哪里

1. `include/search_tutorial/graph_search.h` 里的 `Frontier::pop()`  
   三个算法唯一的取出规则。函数里有被 `#if 0` 包住的参考实现。

2. `src/graph_search.cpp` 里的 `GraphSearch::search()`  
   输入检查已经写好。从「练习 2」注释开始把主循环写回来。  
   **同一文件后半**有完整参考实现（`#if 0`），对着仿写。

先写 `pop()`，再写主循环。头文件顶部的 13 行伪代码就是提纲。

## 怎么验证

```bash
# 容器里
cd /workspace && catkin_make -j$(nproc)

# 终端单步（算法写对之前会显示无路径，这是正常的）
rosrun search_tutorial search_console_demo --algo bfs --map tiny --step

# 写完后交叉验证
rosrun search_tutorial search_selfcheck 300

# RViz 单步
roslaunch search_tutorial search_demo.launch auto_play:=false \
    map_width:=40 map_height:=40 map_type:=random
# 另一个终端
rosrun search_tutorial search_keyboard
```

按 `1/2/3` 切换 BFS / DFS / Dijkstra，按 `n` 一步一步看。
