# BFS / DFS / Dijkstra 讲解

四篇，按顺序读。每个结论都能用这个包里的程序当场验证 —— 文档里所有数字都是真实跑出来的，
命令也一并给了，别信文档，自己跑。

| 篇 | 文件 | 讲什么 |
|---|---|---|
| 一 | [01-bfs.md](01-bfs.md) | 栅格就是图、BFS 为什么能保证步数最少（**证明**）、以及 §1.4：4 邻域下"步数最少"就是"路径最短"，什么时候不是 |
| 二 | [02-dfs.md](02-dfs.md) | 换个容器就是 DFS、为什么它什么都不保证、栈里为什么会有重复条目、它到底有什么用 |
| 三 | [03-dijkstra.md](03-dijkstra.md) | 把 FIFO 换成小顶堆、出堆瞬间 g 已最优的证明、4 邻域下它为什么退化成 BFS（**证明**）、懒惰删除、"再加个 h 就是 A*" |
| 四 | [04-compare.md](04-compare.md) | 实测对比、六张扩展形状图、3000 实例统计、什么场合用哪个 |

## 前提：本包默认 4 邻域

**机器人只能上下左右走，不走对角线**（`SearchConfig::use_8_connected` 默认 `false`）。
这个前提比它看起来重要得多，因为它意味着**每条边的代价都是 1**，于是

```
g(n) == depth(n)     对每个节点都成立
```

「代价最小」和「步数最少」变成**同一个目标**。三条直接后果，贯穿这四篇：

| | |
| --- | --- |
| **BFS 是正确的最短路算法** | 而且比 Dijkstra 便宜（不用堆，`O(V+E)`）—— [01-bfs.md](01-bfs.md) §1.4 |
| **Dijkstra 退化成 BFS** | 单位边权下松弛永远不会成功，堆变成 FIFO 队列 —— [03-dijkstra.md](03-dijkstra.md) §3.4 |
| **两者的扩展形状是同一个菱形** | 曼哈顿距离的等距面 —— [04-compare.md](04-compare.md) §4.2 |

selfcheck 逐个实例断言了这些等式：**3000 个随机实例、308 万项检查、0 失败。**

那为什么还要学 Dijkstra？因为 `g == depth` 只在单位边权下成立。
所有需要"两个目标分开"的地方（8 邻域的 √2 斜边、地形代价、costmap 膨胀层），
文档都用 **`--conn 8` / rviz 里按 `v`** 显式打开 —— 那是**教学用的对照组**，不是默认。

## 一句话总结

> **四个算法是同一段代码。区别只有一处：每轮从 frontier 里取哪个条目出来。**
>
> | 取哪个 | 容器 | 算法 | 保证 |
> |---|---|---|---|
> | 最早进来的 | FIFO 队列 | **BFS** | 步数最少（4 邻域下 = 代价最小） |
> | 最晚进来的 | LIFO 栈 | **DFS** | 什么都不保证 |
> | `g` 最小的 | 小顶堆 | **Dijkstra** | 几何代价最小 |
> | `g + h` 最小的 | 小顶堆 | **A\*** | 几何代价最小（h 可采纳时）—— 在 [astar_tutorial](../../astar_tutorial/doc/README.md) |

这不是类比，是字面意思。本包的 `Frontier::pop()` 就是这么写的：

```cpp
FrontierItem pop() {
  if (algo_ == Algorithm::kDijkstra) { ... heap_.top();   heap_.pop();       ... }
  if (algo_ == Algorithm::kBfs)      { ... deque_.front(); deque_.pop_front(); ... }
  ...                                  // DFS: deque_.back(); deque_.pop_back();
}
```

换掉这一个函数，整段主循环一行都不用改 —— 这就是这个包存在的理由。

## 和 astar_tutorial 的分工

两个包是配套的，本包是**前置篇**：

```
BFS ──换成堆、按 g 排序──> Dijkstra ──排序键加个 h──> A*
DFS   （只换容器方向）
```

- 本包讲**取出规则**这件事本身，以及"步数 / 代价"两个目标函数的区别；
- `astar_tutorial` 讲**启发函数 h**：可采纳性、一致性、A* 最优性证明、Weighted A*。

有些内容故意**不在这里重复**，直接去那边看：

| 想看 | 去 |
|---|---|
| Dijkstra 正确性证明的完整版（逐步展开）、负权反例 | [astar_tutorial/doc/01-dijkstra.md](../../astar_tutorial/doc/01-dijkstra.md) §1.3 / §1.4 |
| decrease-key 的三种实现对照表（懒惰删除 / multimap / 斐波那契堆） | 同上 §1.5 |
| h 是什么、可采纳性 / 一致性、A* 最优性证明、Weighted A* | [astar_tutorial/doc/02-a-star.md](../../astar_tutorial/doc/02-a-star.md) |
| 启发函数实测表、选错 h 的反例（4 邻域为什么该用 manhattan） | [astar_tutorial/doc/03-compare.md](../../astar_tutorial/doc/03-compare.md) |

反过来，`astar_tutorial/doc/01-dijkstra.md` §1.2 只用两段话打发了"BFS 为什么不够"，
本包 [01-bfs.md](01-bfs.md) 把它展开成了一整篇：完整的正确性证明、
`Δ ≥ 3` 的量化推导、可复现的反例地图，以及 4 邻域下它**根本不成立**这件事。

## 和代码的对应

| 概念 | 在哪 |
|---|---|
| 13 行统一伪代码 + 三算法差异表 | `include/search_tutorial/graph_search.h` 开头 |
| ★ 唯一的算法差别 | `graph_search.h` 里 `Frontier::pop()`，标了 `[伪代码 3]` |
| 主循环 | `src/graph_search.cpp` 的 `GraphSearch::search()`，每段标了 `[伪代码 N]` |
| 第 12 行的三个分支 | `graph_search.cpp` 里 `switch (cfg_.algorithm)`，标了 `[伪代码 12]` |
| 三个算法各自保证什么 | `graph_search.cpp` 的 `guaranteesMinCost()` / `guaranteesMinSteps()` / `optimalityNote()` |
| `depth` 和 `g` 两个量 | `struct Node`，两个字段并排放着 |
| 4 邻域 = 单位边权这件事 | `graph_search.h` 开头的「本包默认 4 邻域」一节；方向表在 `graph_search.cpp` 的 `kDX/kDY`（**顺序不能改**） |
| 懒惰去重的开销 | `SearchStats::stale_pops`（丢弃逻辑在 `graph_search.cpp` 的 `[伪代码 4]`） |
| 正确性验证 | `src/search_selfcheck.cpp` —— 拿 3 份独立实现（参考 BFS、A\*(h≡0)、A\*(octile)）+ 回放器交叉验证 |

上手运行、rviz 单步调试、颜色图例、参数表在 [../README.md](../README.md)。
