# 图搜索算法系统学习资料包

本资料包面向初学者，目标是把 **BFS、DFS、Uniform Cost Search、Greedy Best-First Search、A\*** 串成一个完整体系。

## 建议学习顺序

1. `notes/00_图搜索总览.md`
2. `notes/01_BFS_广度优先搜索.md`
3. `notes/02_DFS_深度优先搜索.md`
4. `notes/03_UCS_一致代价搜索.md`
5. `notes/04_Greedy_贪心最佳优先搜索.md`
6. `notes/05_AStar_A星搜索.md`
7. `notes/06_五种算法对比总结.md`
8. 阅读 `cpp/` 中对应 C++ 代码

## 核心主线

所有这些算法都可以理解为：

```text
准备一个 Container
↓
放入起点
↓
不断从 Container 中取出一个节点
↓
访问它
↓
扩展它的邻居
↓
把符合条件的新节点加入 Container
↓
重复
```

它们最大的区别是：

> **下一次从 Container 里面取谁？**

| 算法   | Container / 选择规则     |
| ------ | ------------------------ |
| BFS    | Queue，先进先出          |
| DFS    | Stack，后进先出          |
| UCS    | 选择 g(n) 最小           |
| Greedy | 选择 h(n) 最小           |
| A*     | 选择 f(n)=g(n)+h(n) 最小 |

## C++ 文件

- `cpp/bfs.cpp`
- `cpp/dfs.cpp`
- `cpp/ucs.cpp`
- `cpp/greedy.cpp`
- `cpp/astar.cpp`



[www.redblobgames.com/pathfinding/a-star/introduction.html?utm_source=chatgpt.com](https://www.redblobgames.com/pathfinding/a-star/introduction.html?utm_source=chatgpt.com)
