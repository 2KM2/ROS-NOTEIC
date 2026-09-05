# 三、Dijkstra：把栈换成小顶堆

> 4 邻域（本包默认）下，Dijkstra 和 BFS 给出**完全相同**的最优值 —— 这是 [01-bfs.md](01-bfs.md) §1.4 证过的。
> 所以这一篇必须先回答一个问题：**既然如此，为什么还要学它？**

## 3.1 为什么还要学它

因为 BFS 的正确性建立在一个很脆的前提上：**所有边代价相等**。
Dijkstra 不需要这个前提，它对**任何非负边权**都给出最短路。

在 4 邻域栅格上，"边权不等"会从三个方向冒出来：

| 什么时候边权不等 | 例子 | BFS |
| --- | --- | --- |
| 允许对角线（8 邻域） | 直边 1，斜边 √2 | ❌ 实测 2.3% 的实例给出更长的路径 |
| **地形代价** | 公路 1、草地 2、泥地 5 | ❌ 彻底不对 |
| **代价地图 / 膨胀层** | 离障碍越近代价越高（真实机器人导航栈的标配） | ❌ 彻底不对 |

第二、三条是实际项目里最常见的。`move_base` 的 `global_planner` 之所以是
Dijkstra/A* 而不是 BFS，就是因为 `costmap_2d` 给每个格子附了一个代价。
**一旦你想让机器人"宁可绕远也别贴着墙走"，你就必须用 Dijkstra 或 A*。**

所以这一篇的正确读法是：

> **4 邻域 + 无地形代价是最简单的特例，此时 Dijkstra 退化成 BFS（§3.4 有证明）。
> 学 Dijkstra 是为了在这个特例之外还能用。**

## 3.2 唯一的改动（其实是两处）

| 伪代码行 | BFS | DFS | **Dijkstra** |
| --- | --- | --- | --- |
| 第 3 行 取哪个 | 最早入队（`deque.front()`） | 最晚入栈（`deque.back()`） | **`g` 最小（`heap.top()`）** |
| 第 12 行 已在 frontier 里的邻居 | 跳过 | 再压一次 | **更短就松弛并重入** |

代码：

```cpp
// graph_search.h, Frontier::pop()                          [伪代码 3]
if (algo_ == Algorithm::kDijkstra) {
  const FrontierItem it = heap_.top();  // g 最小的     -> Dijkstra
  heap_.pop();
  return it;
}
```

```cpp
// graph_search.cpp                                         [伪代码 12]
case Algorithm::kDijkstra:
  if (g_new < nb.g - kEps) {                        // 严格更短才动手
    nb.g = g_new;                                   // 松弛
    nb.depth = d_new;
    nb.parent = cur_id;                             // 换爹
    frontier.push({ev.id, cur_id, g_new, d_new});   // 重新入堆（旧条目留在堆里当垃圾）
    ++stats_.pushes;
    ev.action = NeighborAction::kUpdatedBetter;
  } else {
    ev.action = NeighborAction::kSkippedWorse;
  }
  break;
```

**松弛（relax）** 这个词的意思是：`g[nb]` 原本是个上界（"我知道有条路要花这么多"），
现在发现了更便宜的走法，就把这个上界**放松**下来。整个算法就是不停地松弛。

小顶堆的比较器也值得看一眼：

```cpp
struct ItemGreater {
  bool operator()(const FrontierItem& a, const FrontierItem& b) const {
    if (a.g != b.g) return a.g > b.g;
    return a.id > b.id;   // g 相同时按 id，保证同一输入每次跑出完全相同的过程
  }
};
```

第二行那个 `id` 比较**不影响正确性**，只是为了让教学演示可复现（否则 `std::priority_queue`
在并列时的取舍依赖堆内部布局，同一份代码换个编译器就可能走出不同的过程）。

## 3.3 正确性：出堆的那一刻 g 已经最优

**命题**：从堆中取出 `g` 最小的条目、对应节点 `cur` 尚未 CLOSED 时，`g[cur]` 已经是真实最短代价。

**证明**（反证，梗概）：假设还存在一条更短的路 `P: s → … → cur`。沿 `P` 走，`s` 已 CLOSED，
`cur` 未 CLOSED，所以 `P` 上存在**第一个未 CLOSED 的点** `z`。`z` 的前驱 `y` 已 CLOSED，
`y` 进 CLOSED 时松弛过 `z`，所以 `z` 已在堆里且 `g[z] ≤ (P 上 s→z 的代价)`。
又因为**边权非负**，`P` 上 `z→cur` 那段 ≥ 0，于是

```
g[z] ≤ (P 上 s→z 的代价) ≤ (整条 P 的代价) < g[cur]
```

`z` 和 `cur` 都在堆里而 `g[z] < g[cur]` —— 与"`cur` 是堆里 `g` 最小的"矛盾。∎

**完整版（每一步都展开写、外加负权让它崩掉的具体反例）在
[astar_tutorial/doc/01-dijkstra.md](../../astar_tutorial/doc/01-dijkstra.md) §1.3 / §1.4，
这里只留梗概。**

证明里**唯一**用到的额外假设是**边权非负**。栅格地图上代价是几何距离或地形代价，
天然非负，所以不用操心 —— 但如果哪天你把"下坡省电"建模成负代价，这个坑就在等着你。

### 推论 1：出堆的 g 序列单调不减

这是上面命题的直接后果，也是**调试 Dijkstra 最好用的一条不变量**。
真实数据（8 邻域 7×5 空地图，起点 `(0,0)`，前 22 次出堆）：

```
g:    0.000 1.000 1.000 1.414 2.000 2.000 2.414 2.414 2.828 3.000 3.000
      3.414 3.414 3.828 3.828 4.000 4.000 4.243 4.414 4.414 4.828 4.828
      ↑ 严格不减
步数:     0     1     1     1     2     2     2     2     2     3     3
          3     3     3     3     4     4     3     4     4     4     4
                                                      ↑ 4 之后又出现 3 —— 不单调
```

**`g` 单调不减，`步数` 不单调。** 后半句正是 Dijkstra 不保证步数最少的原因：
它宁可多走一格直线，也不肯多走一步 `√2` 的斜线。
（BFS 恰好反过来 —— `步数` 单调不减、`g` 不单调，见 [04-compare.md](04-compare.md) §4.2。）

如果你自己写的 Dijkstra 出现了"后出堆的 g 反而更小"，那必定有 bug。复现：

```bash
rosrun search_tutorial search_console_demo --algo dijkstra --conn 8 \
    --map empty --size 7 5 --no-skips
```

### 推论 2：CLOSED 里的节点可以放心跳过

伪代码第 9 行（`if state[nb] == CLOSED: 跳过`）的合法性就来自这里 ——
CLOSED 节点的 `g` 已经是真值，不可能被改小。

```cpp
// graph_search.cpp                                         [伪代码 9]
// 对 Dijkstra，这一步的合法性是有证明的（出队瞬间 g 已最优，见 doc/03-dijkstra.md）。
```

### 推论 3：终点必须"出堆"才能返回，不能"入堆"就返回

第一次把终点放进堆时，它的 `g` 只是**某一条路**的代价，不一定最优。
只有等它以最小 `g` 出堆，上面的命题才适用。

**提前返回是最常见的"路径偏长"bug**，而且肉眼几乎看不出来。
本包的第 6 行（终点判断）放在第 5 行（落定 CLOSED）之后，就是为了这个。

对比一下三个算法在这一点上的差异，很能说明"共用代码要按最严的算法来"：

| 算法 | 能不能在"入 frontier 时"就返回 | 为什么 |
| --- | --- | --- |
| BFS | **能**（但本包没这么写） | `depth` 第一次发现就是最终值 |
| DFS | 能（它反正什么都不保证） | — |
| **Dijkstra** | **不能** | `g` 还会被松弛改小 |

## 3.4 4 邻域下 Dijkstra 退化成 BFS

这是本包默认配置下最值得知道的一个结论，而且它有个漂亮的两行证明。

**命题**：所有边代价都是 1 时，Dijkstra 的**松弛永远不会成功** ——
既不会换父亲，也不会重复入堆。

**证明**：由推论 1，出堆的 `g` 单调不减。设当前弹出的节点 `cur` 有 `g[cur] = k`。
任何已在堆里（`state == OPEN`）的邻居 `v`，都是被某个**已经出堆**的节点发现的，
那个节点的 `g ≤ k`，所以

```
g[v] ≤ k + 1
```

而这一轮通过 `cur` 给出的新值是 `g_new = g[cur] + 1 = k + 1 ≥ g[v]`，**不严格更小**。
所以第 12 行走的是 `kSkippedWorse` 分支，什么都不做。∎

**推论：堆退化成了一个 FIFO 队列。** 每个格子恰好入堆一次，`stale_pops == 0`，
比 BFS 多付出的只有堆的那个 `log`。

selfcheck 逐个实例断言了这三条：

```cpp
check(updated_better == 0,           "4 邻域下 Dijkstra 一次成功松弛都没有（不会换爹）");
check(pushed_again == 0,             "4 邻域下 Dijkstra 不会重复入堆");
check(dij.stats().stale_pops == 0,   "4 邻域下 Dijkstra 的 stale_pops 也是 0");
```

实测（3000 个实例、308 万项检查、0 失败）在 `--compare` 表上也看得见 ——
`stale` 那一列 Dijkstra 是 0，`pushes` 和 BFS 一样：

```
同一张地图 (41x25, 4邻域), 起点(0,0) -> 终点(40,24)
algo        expanded   pushes    stale      peak  steps       cost       ms
BFS              820      820        0        27     64    64.0000    0.073
Dijkstra         820      820        0        26     64    64.0000    0.075
```

一模一样。切到 8 邻域，`stale` 立刻就不是 0 了：

```
同一张地图 (41x25, 8邻域), 起点(0,0) -> 终点(40,24)
algo        expanded   pushes    stale      peak  steps       cost       ms
BFS              815      820        0        38     43    51.6985    0.130
Dijkstra         820      826        6        43     43    51.6985    0.317
```

`pushes 826 > expanded 820`，`stale 6` —— 松弛开始成功了，这才是 Dijkstra 真正在工作的样子。

复现：

```bash
rosrun search_tutorial search_console_demo --compare --map random --size 41 25 --seed 11
rosrun search_tutorial search_console_demo --compare --map random --size 41 25 --seed 11 --conn 8
```

### 所以：4 邻域该用哪个？

**用 BFS。** 同样的答案，少一个 `log`，少一个堆，`stale_pops` 恒为 0。
`optimalityNote()` 就是这么说的：

```
Dijkstra + 4 邻域（本包默认）：边代价全是 1，代价最小 == 步数最少，答案和 BFS 一模一样
—— 这种场合用 BFS 更划算（省掉堆的 log 和那些重复条目）。
Dijkstra 的价值在边代价**不相等**的时候：打开 8 邻域，或者给格子加上地形代价。
```

## 3.5 堆里的垃圾：懒惰删除

标准 Dijkstra 需要 **decrease-key**（`g` 变小时把节点在堆里往上调），
而 `std::priority_queue` 不支持。本包用的是**懒惰删除**：不改旧条目，
直接再 push 一条新的，出堆时发现节点已 CLOSED 就当垃圾丢掉。

（三种替代方案 —— 懒惰删除 / `multimap` + 存迭代器 / 斐波那契堆 —— 的复杂度对照表在
[astar_tutorial/doc/01-dijkstra.md](../../astar_tutorial/doc/01-dijkstra.md) §1.5，
连"作业里那份用的是哪种"都标了，这里不重复。三者结果**完全一致**，只差常数和内存。）

这一节要补的是隔壁那张表**没讲**的东西：**懒惰删除的开销可以量化，而且和 DFS 共用同一个字段。**
`SearchStats::stale_pops` 就是被丢掉的过期条目数，本包因此把两个数分开报：

```
入队次数   : 826   其中弹出后发现是重复条目 6 次
峰值       : frontier 条目 43   真实 OPEN 节点 42
```

`frontier 条目` 峰值是**真实内存开销**，`OPEN 节点` 峰值才是**信息量**。
Dijkstra 的这两个数通常很接近（垃圾少）；DFS 的能差好几倍
（见 [02-dfs.md](02-dfs.md) §2.4）。

### 一个必须写对的细节

出堆时的甄别只需要检查 `state == CLOSED`，**不需要**额外比较"这个条目的 g 是不是过期了"：

```
4      if state[item.id] == CLOSED:  丢掉（重复条目），continue
```

*为什么够*：同一个节点在堆里的多份条目里，`g` 最小的那份必然**最先**出堆
（这就是小顶堆的定义）。它出堆时节点被 CLOSED，剩下那些 `g` 更大的条目后续出堆时
必然撞上 `state == CLOSED` 而被丢掉。所以"CLOSED 检查"已经覆盖了"g 过期检查"。

多写一个 `if (item.g > nodes_[id].g) continue;` 不算错，但是多余的。

### `<` 还是 `<=`

```cpp
if (g_new < nb.g - kEps) { ... }     // 严格小于
```

用严格小于。相等时换父亲不改变代价，只是白白多一次 push，
还让搜索过程随实现细节抖动。改成 `<=` 再跑 selfcheck ——
**代价一个字都不会变，`pushes` 和 `stale_pops` 会变大**。这是个很好的练习。

## 3.6 复杂度

用二叉堆：**`O((V + E) log V)`**。4 邻域下 `E ≤ 4V`，所以就是 `O(V log V)`。

但和 BFS 一样，真正的问题不是那个 `log`，而是 `V` 是**格子数**，随分辨率的**平方**增长。
Dijkstra 和 BFS 都**不知道终点在哪**，所以往所有方向平摊。41×25 空地图：

```
algo        expanded   pushes    stale      peak  steps       cost       ms
BFS             1025     1025        0        26     64    64.0000    0.038
Dijkstra        1025     1025        0        26     64    64.0000    0.083
A*(manhat)        65      127        0        63     64    64.0000    0.025
```

**1025 → 65。** 那一行就是下一步要做的事。

## 3.7 再加一个 h，就是 A*

Dijkstra 按 `g` 排序。如果排序键换成 `f = g + h`，其中 `h(n)` 是"从 `n` 到终点的估计代价"，
得到的就是 **A\***。**第 3 行的比较器换一个字段，别的什么都不改。**

```
BFS       第 3 行取: 最早入队的
DFS       第 3 行取: 最晚入栈的
Dijkstra  第 3 行取: g     最小的
A*        第 3 行取: g + h 最小的      <- 就这一处
```

反过来说：**`h ≡ 0` 的 A* 就是 Dijkstra**。这不是类比，`astar_tutorial` 里没有单独的
Dijkstra 实现，`--h zero` 就是它。本包的 `search_selfcheck` 正是利用这个等价关系工作的 ——
它拿隔壁包的 A* 当独立的第二、第三份实现来交叉验证：

```cpp
// search_selfcheck.cpp  ---- 检查 4：Dijkstra 的代价 == A*(h≡0) == A*(octile) ----
for (const astar_tutorial::Heuristic hh :
     {astar_tutorial::Heuristic::kZero, astar_tutorial::Heuristic::kDiagonal}) {
  astar_tutorial::AStar a;
  acfg.heuristic = hh;
  a.setMap(&map);
  a.setConfig(acfg);
  a.search(sx, sy, gx, gy);
  check(std::fabs(a.stats().path_cost - dij.stats().path_cost) < 1e-6,
        std::string("Dijkstra 的代价 == A*(") + astar_tutorial::toString(hh) + ") 的代价");
}
```

**三份独立实现给出同一个数字，才敢说它是最优解。**

4 邻域下该配哪个 `h`？**manhattan**（`|dx| + |dy|`，正好是真实最少代价）。
配 octile 也可采纳、结果也对，但太保守、白扩展一堆节点：

| 4 邻域 41×25 空地图 | expanded |
| --- | --- |
| Dijkstra（`h ≡ 0`） | 1025 |
| A\*(octile) | 962 |
| **A\*(manhattan)** | **65** |

`h` 怎么选、什么是可采纳性 / 一致性、A* 的最优性证明、Weighted A* 拿什么换什么 —— 全在
[astar_tutorial/doc/02-a-star.md](../../astar_tutorial/doc/02-a-star.md)
（四种 `h` 各自的可采纳性在 §2.4，包括 8 邻域下 manhattan 为什么**不可采纳**；
真的走错的那个反例地图在 [03-compare.md](../../astar_tutorial/doc/03-compare.md) §3.3）。

## 3.8 亲手跑一遍

4 邻域下 Dijkstra 和 BFS 走出同一个菱形，这件事最好自己看一遍：

```bash
# 两条命令的 CLOSED 区域是同一个菱形
rosrun search_tutorial search_console_demo --algo bfs      --map empty --size 21 13 \
    --start 10 6 --goal 20 12 --range 40 40 --no-skips
rosrun search_tutorial search_console_demo --algo dijkstra --map empty --size 21 13 \
    --start 10 6 --goal 20 12 --range 40 40 --no-skips
```

```
BFS  第 40 步                                Dijkstra 第 40 步
. . . . . . . . . . . . . . . . . . . . G    . . . . . . . . . . . . . . . . . . . . G
. . . . . . . . . . o . . . . . . . . . .    . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . o @ o . . . . . . . . .    . . . . . . . . . o o o . . . . . . . . .
. . . . . . . . o x x x o . . . . . . . .    . . . . . . . . o x x @ o . . . . . . . .
. . . . . . . o x x x x x o . . . . . . .    . . . . . . . o x x x x x o . . . . . . .
. . . . . . o x x x x x x x o . . . . . .    . . . . . . o x x x x x x x o . . . . . .
. . . . . o x x x x S x x x x o . . . . .    . . . . . o x x x x S x x x x o . . . . .
. . . . . . o x x x x x x x o . . . . . .    . . . . . . o x x x x x x x o . . . . . .
. . . . . . . o x x x x x o . . . . . . .    . . . . . . . o x x x x x o . . . . . . .
. . . . . . . . o x x x o . . . . . . . .    . . . . . . . . o x x x o . . . . . . . .
. . . . . . . . . o o o . . . . . . . . .    . . . . . . . . . o x o . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .    . . . . . . . . . . o . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .    . . . . . . . . . . . . . . . . . . . . .
```

**同一个菱形。** 差别只在最外圈的并列格子上谁先出队 —— 那是实现细节（FIFO 顺序 vs 堆里的
`id` 比较），不是算法差别。这正是 §3.4 那个证明的图像版：4 邻域下堆退化成了队列。

再切到 8 邻域，形状立刻分开：

```bash
rosrun search_tutorial search_console_demo --algo bfs      --conn 8 --map empty --size 21 13 \
    --start 10 6 --goal 20 12 --range 40 40 --no-skips
rosrun search_tutorial search_console_demo --algo dijkstra --conn 8 --map empty --size 21 13 \
    --start 10 6 --goal 20 12 --range 40 40 --no-skips
```

BFS 变成**正方形**（切比雪夫等距面 `max(|dx|,|dy|) = 常数`，因为斜走一步也只算 1 步），
Dijkstra 变成**八角形**（欧氏距离的近似等距面，因为斜走要付 √2）。
**两个不同的形状 = 两个不同的目标函数**，这就是 §3.1 那张表的几何版本。

想在 rviz 里对着看，最省事的办法是按 `3` / `1` 来回切，再按 `v` 切邻域 ——
见 [../README.md](../README.md) 第 1 节。

---

下一篇：把三个算法（外加 A\*）放在一起，用实测数字回答"什么时候用哪个"。
→ [04-compare.md](04-compare.md)
