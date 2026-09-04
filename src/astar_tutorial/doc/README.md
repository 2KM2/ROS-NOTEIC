# A* 与 Dijkstra 讲解

三篇，按顺序读。每个结论都能用这个包里的程序当场验证 —— 文档里所有数字都是真实跑出来的，
命令也一并给了，别信文档，自己跑。

| 篇 | 文件 | 讲什么 |
|---|---|---|
| 一 | [01-dijkstra.md](01-dijkstra.md) | 最短路问题到底在求什么、BFS 为什么不够、Dijkstra 的原理和**正确性证明** |
| 二 | [02-a-star.md](02-a-star.md) | h 是怎么加进去的、为什么排序用 `f=g+h`、可采纳性 / 一致性、A* 的**最优性证明** |
| 三 | [03-compare.md](03-compare.md) | 实测对比、扩展形状图、启发函数选错的反例、什么场合该用哪个 |

## 一句话总结

> **Dijkstra 是不知道终点在哪的 A*；A* 是知道该往哪走的 Dijkstra。**
> 两者唯一的结构差别是优先队列的排序键：Dijkstra 用 `g`，A* 用 `g + h`。

这个包里**没有单独的 Dijkstra 实现** —— 把启发函数设成 `h ≡ 0` 的 A* 就是 Dijkstra，
一行代码都不用改：

```bash
rosrun astar_tutorial astar_console_demo --map empty --size 21 11 --h zero      # Dijkstra
rosrun astar_tutorial astar_console_demo --map empty --size 21 11 --h diagonal  # A*
```

`astar_selfcheck` 正是靠这个等价关系工作的：拿 `h≡0` 的结果当"最优代价"真值，
去校验其他启发函数下的 A*。

## 和代码的对应

| 概念 | 在哪 |
|---|---|
| 12 行伪代码 | `include/astar_tutorial/astar.h` 开头 |
| 主循环 | `src/astar.cpp` 的 `AStar::search()`，每段标了 `[伪代码 N]` |
| 四种 h | `src/astar.cpp` 的 `rawHeuristic()` |
| 可采纳性检查 | `src/astar.cpp` 的 `isHeuristicAdmissible()` / `admissibilityWarning()` |
| 洪水填充（去掉 g/h/f 的骨架） | `src/grid_map_2d.cpp` 的 `GridMap2D::isConnected()` |
| 最优性验证 | `src/astar_selfcheck.cpp` |

上手运行、颜色图例、参数表在 [../README.md](../README.md)。
