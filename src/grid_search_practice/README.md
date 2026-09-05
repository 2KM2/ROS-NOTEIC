# grid_search_practice

40×40 栅格（每格 1 m）练习包。地图、邻居、代价、RViz 单步回放已经写好，
**BFS / DFS / Dijkstra / A\* 四个独立函数留给你写**。

## 你写哪里

`src/your_search.cpp` 里的四个函数。每个函数的注释就是提纲。

辅助函数（直接调用）：

- `neighbors(map, x, y, opt)` 可走邻居
- `edgeCost(x0, y0, x1, y1)` 直角 1，斜边 √2
- `heuristic(...)` 4 邻域曼哈顿，8 邻域 octile
- `reconstructPath(parent, map, start_id, goal_id)`

每扩展一格，往 `out.steps` 里记一步（`current` / `open` / `closed`），RViz 才能回放。

## 编译 / 跑

容器里：

```bash
cd /workspace && catkin_make -j$(nproc)
source devel/setup.bash

# 辅助函数自检（不测四个算法）
rosrun grid_search_practice practice_selfcheck

# RViz
roslaunch grid_search_practice practice.launch

# 另一个终端
rosrun grid_search_practice practice_keyboard
```

按 `n` 单步。算法没写时只显示地图和起终点（紫球/青绿球），这是正常的。

默认：起点 `(0,0)`，终点 `(39,39)`，45 个随机障碍，4 邻域。
按 `1/2/3/4` 切算法，`v` 切 8 邻域。

颜色含义（蓝=CLOSED，绿=OPEN，红=当前，橙=路径）见 [doc/rviz-colors.md](doc/rviz-colors.md)。
