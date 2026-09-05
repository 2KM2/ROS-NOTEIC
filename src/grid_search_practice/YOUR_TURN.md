# 你来写：BFS / DFS / Dijkstra / A*

文件：`src/your_search.cpp`

建议顺序：BFS → DFS → Dijkstra → A*。

A* 几乎就是 Dijkstra，取出规则从 `g` 改成 `g+h`。

## 怎么知道写对了

```bash
roslaunch grid_search_practice practice.launch auto_play:=false
rosrun grid_search_practice practice_keyboard
```

- BFS：一圈圈往外推（4 邻域是菱形）
- DFS：一条线扎到底
- Dijkstra：4 邻域下应和 BFS 同形同代价；8 邻域下代价应 ≤ BFS
- A*：代价应和 Dijkstra 一样，扩展格子通常更少

按 `c` 看四算法对比表。
