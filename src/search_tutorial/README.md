# search_tutorial —— BFS / DFS / Dijkstra：三个算法，一个循环

这个包想说明的**唯一**一件事：

> **BFS、DFS、Dijkstra（以及隔壁包的 A\*）是同一段代码。
> 区别只有一处 —— 每一轮从"待处理集合"里取哪一个出来。**

不是类比，是字面意思。整个包的核心就是 `graph_search.h` 里的 `Frontier::pop()`：

```cpp
// ★★★ 整个包的核心就是这个函数 ★★★                              [伪代码 3]
// 三行代码，三个算法。别的地方一个字都不用改。
FrontierItem pop() {
  if (algo_ == Algorithm::kDijkstra) {
    const FrontierItem it = heap_.top();  // g 最小的     -> Dijkstra
    heap_.pop();
    return it;
  }
  if (algo_ == Algorithm::kBfs) {
    const FrontierItem it = deque_.front();  // 最早进来的 -> BFS（FIFO 队列）
    deque_.pop_front();
    return it;
  }
  const FrontierItem it = deque_.back();  // 最晚进来的   -> DFS（LIFO 栈）
  deque_.pop_back();
  return it;
}
```

换掉这一个函数，**整段主循环、邻居检查、路径回溯，一行都不用改**。

## 前提：默认 4 邻域

**机器人只能上下左右走，不走对角线。** 于是每条边的代价都是 1，`g(n) ≡ depth(n)`，
「代价最小」和「步数最少」是**同一个目标**。所以在默认配置下：

| 结论 | 出处 |
| --- | --- |
| **BFS 就是正确的最短路算法**，而且不用堆（`O(V+E)`） | [doc/01-bfs.md](doc/01-bfs.md) §1.4 |
| **Dijkstra 退化成 BFS** —— 松弛永远不会成功，`stale_pops` 恒为 0 | [doc/03-dijkstra.md](doc/03-dijkstra.md) §3.4 |
| **两者扩展出同一个菱形** | [doc/04-compare.md](doc/04-compare.md) §4.2 |

selfcheck 逐个实例断言了这些等式：**3000 个随机实例、308 万项检查、0 失败。**

需要看"两个目标分开"的场合（8 邻域的 √2 斜边、地形代价），
文档都用 `--conn 8` / rviz 里按 `v` **显式打开** —— 那是教学用的对照组，不是默认。

---

## 1. 一步一步在 rviz 里调试路径

```bash
cd docker && ./build_ws.sh          # 编译

# 终端 1：可视化 + rviz。auto_play:=false 让它停在第 0 步等你
./run.sh
roslaunch search_tutorial search_demo.launch auto_play:=false \
    map_type:=empty map_width:=20 map_height:=14 map_resolution:=1.0 show_cost_text:=true

# 终端 2：键盘（要独立的 stdin，所以必须单独开一个终端）
cd docker && ./run.sh
rosrun search_tutorial search_keyboard
```

然后按 `n` 一步一步走。**终端 1 会同步打印这一步的全部细节**
（取出了谁、每个邻居怎么判、frontier 里还剩多少），rviz 里同步高亮 —— 两边对着看。

### 推荐的第一次玩法

| 按 | 看什么 |
| --- | --- |
| `1` 然后一直按 `n` | **BFS** 一圈圈往外推，长成一个菱形（曼哈顿距离的等距面） |
| `3` 然后 `n` | **Dijkstra** —— **同一个菱形**。4 邻域下它们是同一个算法 |
| `2` 然后 `n` | **DFS** 一头扎到底。注意终端里刷过去的"已在栈里,DFS再压一次"和紫色的"重复条目" |
| `c` | 三算法对比表，确认 BFS / Dijkstra 的 `cost` 逐位相同、DFS 的路径长几倍 |
| `v` 再按 `c` | 切到 **8 邻域** —— BFS 那行的 `cost` 开始比 Dijkstra 大 |
| `v` 后再 `1` / `3` | **正方形 vs 八角形** —— 两个不同的目标函数，肉眼可见 |
| `f` | 直接跳到结果，看最终路径 |
| `b` | 上一步（往回走是重放到那一步，不是"撤销"） |
| `t` | 每格显示 `g` / `步数` 两行数字（地图小的时候再开） |

rviz 里用工具栏的 **2D Pose Estimate** 设起点、**2D Nav Goal** 设终点，点完立刻重搜。

### 全部按键

```
空格 / n / →   下一步        b / ←   上一步      p  播放/暂停
r  回到第 0 步                f  跳到最后一步     ] / [  加速 / 减速
1 / 2 / 3  BFS / DFS / Dijkstra          a  循环切换算法
c  三算法对比表               v  切 4 / 8 邻域    t  切 g/步数 文字
m  换一张新地图(换 seed)      g  重新规划         h  帮助      q  退出
```

不想用键盘也行，每个键都有对应的服务：

```bash
rosservice call /search/step          # 下一步
rosservice call /search/back          # 上一步
rosservice call /search/play_pause
rosservice call /search/reset
rosservice call /search/finish
rosservice call /search/replan
rosservice call /search/new_map
rosservice call /search/use_bfs       # 换算法（地图和起终点不变）
rosservice call /search/use_dfs
rosservice call /search/use_dijkstra
rosservice call /search/compare       # 打一遍对比表
```

（`v` 切邻域只有键盘/话题版：`rostopic pub -1 /search/cmd std_msgs/String "data: conn"`。）

### 颜色对照

| 颜色 | 含义 |
| --- | --- |
| 灰色格子 | 障碍（`nav_msgs/OccupancyGrid`） |
| **蓝色** | CLOSED —— 已经出队处理过，`g` / 步数 / 父亲定死了 |
| **绿色** | 在 frontier 里排队，等着被取出来 |
| **红色柱子** | 本步取出的节点 |
| **紫色柱子** | 本步取出的是**重复条目** —— 懒惰去重丢掉它，这一步什么都不做（DFS 下很多） |
| **黄色** | 本步检查过的邻居 |
| **青色细线** | 起点 → 当前节点的**临时**父链（Dijkstra 松弛 / DFS 重压时会抖动） |
| **橙色粗线** | 最终路径 |
| 左上白字 | 算法 / 邻域 / 保证什么 / 第几步 / 当前 `g` 和步数 / `frontier条目` vs `OPEN节点` / 结果 |

左上那一行 **`frontier条目=… OPEN节点=…`** 值得单独盯：两个数**差多少就是垃圾条目有多少**。
BFS 下它们永远相等，DFS 下能差三四倍。

### 发布的话题

| 话题 | 类型 |
| --- | --- |
| `/search/map` | `nav_msgs/OccupancyGrid`（latched） |
| `/search/markers` | `visualization_msgs/MarkerArray` |
| `/search/path` | `nav_msgs/Path`（latched） |
| `/search/cmd` ← | `std_msgs/String`，键盘节点发的就是这个 |

---

## 2. 强烈建议：先在终端里啃一遍

rviz 好看，但抠细节还是控制台舒服 —— 它把每一次出队、每个邻居的判决全打出来。

```bash
# 8x6 手算规模的小地图，一步一步按回车
rosrun search_tutorial search_console_demo --algo bfs --map tiny --step --no-skips
```

```
=== 第 1/39 步 ===
① 取出**最早入队**的条目 (FIFO 队列头): (0,0)  g=0.000  步数=0
② 标记 CLOSED（g / 步数 / 父亲就此定死，以后不再改）
③ 逐个检查 4 个邻居（其中 2 个被跳过，--no-skips 已隐藏）:
     ( 1, 0) 边代价1.000  新发现,入frontier       g: inf -> 1.000   步数=1
     ( 0, 1) 边代价1.000  新发现,入frontier       g: inf -> 1.000   步数=1
④ 本步结束: frontier条目=2  OPEN节点=2  CLOSED=1
```

同一张图换成 DFS，看它有多离谱：

```bash
rosrun search_tutorial search_console_demo --algo dfs --map tiny --step --no-skips
```

其它常用玩法：

```bash
# 三算法 + A* 横向对比表（默认 4 邻域）
rosrun search_tutorial search_console_demo --compare --map empty --size 41 25

# 8 邻域：BFS 那行的 cost 开始变大
rosrun search_tutorial search_console_demo --compare --map empty --size 41 25 --conn 8

# 自动搜一个"BFS 步数最少但路径更长"的地图并并排打印（强制 8 邻域）
rosrun search_tutorial search_console_demo --counterexample

# 只详细看第 40~45 步，前面快速跳过
rosrun search_tutorial search_console_demo --algo dfs --map walls --range 40 45

# DFS 的"弹出重复条目"空步太多，想看干净点
rosrun search_tutorial search_console_demo --algo dfs --map tiny --no-stale
```

全部选项：

```
--algo bfs|dfs|dijkstra   选算法 (默认 dijkstra)
--map tiny|empty|walls|random|maze|corner   地图类型 (默认 tiny)
--size W H            地图尺寸 (默认 40 20，tiny/corner 忽略)
--seed N              随机种子 (默认 7)
--ratio R             random 地图的障碍占据率 (默认 0.2)
--conn 4|8            邻域 (默认 4，即只上下左右)
--corner-cut          允许斜穿墙角（只在 8 邻域下有意义）
--start X Y / --goal X Y   起终点栅格坐标
--step                每步等回车（单步调试模式）
--range A B           只详细打印第 A..B 步
--no-skips            不打印"跳过"类事件
--no-stale            不打印"弹出重复条目"的空步(DFS 下很多)
--quiet               只打印最终结果
--compare             BFS/DFS/Dijkstra/A* 横向对比表
--counterexample      自动搜一个"BFS 步数最少但路径更长"的地图
```

---

## 3. 代码结构：从哪儿开始读

**按这个顺序读，别从 `search()` 开始。**

| 读第几个 | 文件 / 位置 | 看什么 |
| --- | --- | --- |
| **1** | `include/search_tutorial/graph_search.h` **开头 60 行** | 13 行统一伪代码 + 三算法差异表 + "4 邻域 = 单位边权"那一节。**先把这张表看懂。** |
| **2** | 同文件里的 `Frontier::pop()`，标了 `[伪代码 3]` | ★ 唯一的算法差别。三个 `return`，三个算法。 |
| **3** | `src/graph_search.cpp` 的 `switch (cfg_.algorithm)`，标了 `[伪代码 12]` | 第二处差别：已在 frontier 里的邻居怎么办。三个分支并排放着。 |
| **4** | `src/graph_search.cpp` 的 `GraphSearch::search()` | 主循环。每一段都标了 `[伪代码 N]`，对着第 1 步那张伪代码读。 |
| **5** | `guaranteesMinCost()` / `guaranteesMinSteps()` / `optimalityNote()` | 每个算法保证什么、为什么。运行时会打在结果里。 |
| **6** | `src/trace_replayer.cpp` | 怎么把"事件序列"重放成"第 k 步的完整状态"（`b` 键往回走靠的就是它）。 |
| 之后 | `src/search_rviz_node.cpp` / `search_console_demo.cpp` | 纯展示层，不含算法。 |

三个关键量在 `struct Node` 里并排放着，别搞混：

```
depth(n)  从起点到 n 的**步数**（走了几格）。每条边算 1 步。   BFS 优化这个
g(n)      从起点到 n 的**几何代价**。4 邻域全是 1，8 邻域斜边 √2。 Dijkstra 优化这个
h(n)      到终点的估计代价。本包**没有** —— 加上它就是 A*，在隔壁包
```

原理讲解（四篇，带证明和实测数据）在 **[doc/README.md](doc/README.md)**。

---

## 4. 六个最值得琢磨的点

### 4.1 4 邻域下 BFS 和 Dijkstra 为什么"必然"相同

每条边代价都是 1 ⇒ `g(v) = 1 × depth(v) = depth(v)` ⇒ 两个目标函数逐点相等。
更强的结论：**单位边权下 Dijkstra 的松弛永远不会成功**。

出堆的 `g` 单调不减，弹出 `g = k` 的节点时，任何已在堆里的邻居 `v` 都是从某个 `g ≤ k`
的节点发现的，所以 `g[v] ≤ k+1`；而新路径给出 `g_new = k+1 ≥ g[v]`，不严格更小。
于是既不换父亲也不重复入堆 —— **堆退化成一个 FIFO 队列**。

selfcheck 把它变成了三条断言（`updated_better == 0`、`pushed_again == 0`、`stale_pops == 0`），
3000 个实例全过。证明和数据：[doc/03-dijkstra.md](doc/03-dijkstra.md) §3.4。

**所以 4 邻域下就用 BFS** —— 同样的答案，省掉堆的一个 `log`。

### 4.2 那为什么还要学 Dijkstra

因为 `g ≡ depth` **只在单位边权下成立**。两种崩法：

- **8 邻域**：斜边 `√2`。3 步斜线（`3√2 ≈ 4.243`）比 4 步直线（`4.000`）**步数更少却更长**，
  BFS 立刻开始给出次优路径 —— 而路径看起来完全正常。实测 **2.3%** 的实例会中，最差长 **7.93%**。
- **地形代价 / costmap**：草地 2、公路 1、离墙越近越贵。BFS 彻底没救。
  `move_base` 的 global planner 之所以是 Dijkstra/A* 而不是 BFS，就是因为这个。

量化推导（`Δ ≥ 3`）和可复现的反例地图：[doc/01-bfs.md](doc/01-bfs.md) §1.4。

### 4.3 终点必须"出队"才能返回

BFS 的 `depth` 第一次发现就是最终值，所以对 BFS 来说"入队时判断终点并返回"是**正确**的。
但 **Dijkstra 绝对不行** —— 那时终点的 `g` 只是某一条路的代价。

本包的伪代码第 6 行（终点判断）放在第 5 行（落定 CLOSED）**之后**，
就是因为这段循环要同时给三个算法用。**共用代码时，正确性要按最严的那个算法取。**

提前返回是最常见的"路径偏长"bug，而且肉眼几乎发现不了。

### 4.4 栈/堆里为什么会有重复条目

DFS 发现邻居已经在栈里时**照压一份新的**（这是标准 DFS，不是 bug）：
新的那份在栈顶附近、马上会被处理，这才是"优先走最新发现的分支"。
Dijkstra 松弛成功时也会重新入堆，旧条目留在堆里当垃圾。

旧条目不删（在容器中间删太贵），等它出来时靠伪代码第 4 行的 `state == CLOSED` 丢掉 ——
这叫**懒惰去重**，`SearchStats::stale_pops` 就是被丢掉的数量。

**这也是本包把 `frontier条目` 和 `OPEN节点` 分开显示的原因**：
前者是真实内存开销，后者才是信息量。DFS 的这两个数能差 3~4 倍。
细节：[doc/02-dfs.md](doc/02-dfs.md) §2.4。

顺带一个反直觉的观察：**同样是 4 邻域单位边权，`A*` 的 `stale` 却不是 0**
（`walls` 图上 258 次）—— 因为 4.1 那个证明用的是"出堆 `g` 单调不减"，
而 A\* 出堆的是 `f = g + h` 单调不减，`g` 不再单调，证明失效。

### 4.5 方向表的顺序是有意义的

```cpp
// 前 4 个是直走(右/左/上/下，代价 1)，后 4 个是斜走(代价 √2)。
// 默认 use_8_connected=false，只取前 4 个 —— **所以这个顺序不能改**，
// 一旦把某个斜方向挪到前 4 个里，4 邻域就悄悄变成了"允许对角线"。
static const int kDX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
static const int kDY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
const int num_dirs = cfg_.use_8_connected ? 8 : 4;
```

另一件事：**这个顺序对 BFS / Dijkstra 的代价毫无影响**（只影响并列时谁先出队），
**但对 DFS 影响巨大** —— DFS 走的是"最后压进去的那个方向"。
一个算法的答案依赖于你循环邻居的顺序，这本身就说明它没在优化任何东西。

想试：把前四项换个顺序重新编译，`--algo dfs` 的路径会完全变样，
而 `--algo bfs` / `--algo dijkstra` 的代价一个字都不会变（selfcheck 也照样全过）。

### 4.6 DFS 到底有什么用

上面全是缺点，但 DFS 在真实工程里到处都是 —— 因为它的目标压根不是最短路：

- **可达性 / 连通性**：只想知道"通不通"、"有几个连通块"。任何遍历都对，选最便宜的。
- **分支因子大、深度浅的搜索树**（博弈树、SAT、回溯法）：BFS 的队列宽 `b^d`（指数），
  DFS 的栈深只有 `O(d)`（线性）。**栅格地图恰好相反，所以别把这里的结论往那边推。**
- **迷宫这类"路本来就唯一"的图**：完美迷宫是一棵树，最短路 = 随便一条路，
  实测 DFS 的路径和 BFS **一样长**（都是 140 步）。
- **Tarjan 那一套**（拓扑排序、强连通分量、桥和割点）：建立在 DFS 的递归结构上，BFS 做不了。

详见 [doc/02-dfs.md](doc/02-dfs.md) §2.5。

---

## 5. 实测对比

```bash
rosrun search_tutorial search_console_demo --compare --map empty --size 41 25
```

```
同一张地图 (41x25, 4邻域), 起点(0,0) -> 终点(40,24)
algo        expanded   pushes    stale      peak  steps       cost       ms
------------------------------------------------------------------------------
BFS             1025     1025        0        26     64    64.0000    0.061
DFS             1025     1985        0       961   1024  1024.0000    0.080   <- 路径比最短路长 1500.0%
Dijkstra        1025     1025        0        26     64    64.0000    0.088
A*(manhat)        65      127        0        63     64    64.0000    0.027
```

三件事：

1. **BFS 和 Dijkstra 每一列都相同**（连 `peak` 都是 26）—— 4 邻域下它们是同一个算法。
2. **DFS 走了 1024 步走完 64 步就能到的路**，`peak` 是 BFS 的 37 倍。
   1025 个格子它经过了 1024 个 —— 把空地图当成一条蛇形走廊走了一遍。
3. **`A*(manhat)` 一行：1025 → 65。**

第 3 点是这个包最该带走的一句话：

> **BFS / DFS / Dijkstra 是理解那个统一循环的最短路径，而实际要用的是 A\*。**
> 它只是在第 3 行的排序键上加了一个 `h`，换来 **15.8 倍**。

DFS 在四种地图上有多差（4 邻域，41×25）：

| 地图 | BFS / Dijkstra 步数 | DFS 步数 | DFS 长了 |
| --- | --- | --- | --- |
| `empty` | 64 | **1024** | **+1500.0%** |
| `walls` (seed 7) | 80 | 448 | +460.0% |
| `random` (seed 11) | 64 | 510 | +696.9% |
| `maze` (seed 3) | 140 | 140 | **+0%** ← 树上只有一条路 |

完整的六张扩展形状图、3000 实例统计、一张"什么时候用哪个"的选择表：
**[doc/04-compare.md](doc/04-compare.md)**。

---

## 6. 自检：怎么知道这三份实现是对的

```bash
rosrun search_tutorial search_selfcheck        # 默认 300 个实例
rosrun search_tutorial search_selfcheck 3000   # 慢一点，覆盖更全
```

它不是"跑一遍看不崩"，而是拿**三份独立实现**对答案：

| 交叉验证对象 | 查什么 |
| --- | --- |
| 一份独立写的参考 BFS | `BFS 的 depth[] 逐格等于最少步数` |
| `astar_tutorial` 的 **A\*(h≡0)** | `Dijkstra 的代价 == A*(zero) 的代价` |
| `astar_tutorial` 的 **A\*(octile)** | `Dijkstra 的代价 == A*(diagonal) 的代价` |
| `TraceReplayer` 回放到底 | 回放出的 `state/g/depth/parent` 和 `nodes()` **逐位相等**，且 seek 幂等 |

外加各算法自己的不变量：`BFS 的 stale_pops 恒为 0`、`DFS 的代价 >= Dijkstra 的代价`、
`三个算法对可达性的判断一致`、以及 §4.1 那三条 4 邻域断言。

真实输出：

```
================ 统计（8 邻域，2771 个可解实例）================
BFS 的路径严格比最短路更长 : 65 次 (2.3%)，最差比最优解长 7.93%
Dijkstra 的步数严格比 BFS 多 : 20 次 (0.7%)
DFS 的路径严格比最短路更长   : 2171 次，最差长 1698.7%

================ 统计（4 邻域 = 本包默认，2737 个可解实例）================
BFS 的路径严格比最短路更长 : 0 次（必须是 0）
Dijkstra 的步数严格比 BFS 多 : 0 次（必须是 0）

================ 结果 ================
检查 3084303 项，失败 0 项
全部通过 ✓
```

**三份独立实现给出同一个数字，才敢在文档里写"最优"。**

---

## 7. launch 参数

```bash
roslaunch search_tutorial search_demo.launch [参数:=值 ...]
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `algorithm` | `dijkstra` | `bfs` / `dfs` / `dijkstra`。跑起来后也能按 `1`/`2`/`3` 在线换 |
| **`use_8_connected`** | **`false`** | **默认 4 邻域**（只上下左右）。`true` = 8 邻域，斜边 `√2`，BFS 开始次优 |
| `allow_corner_cutting` | `false` | 允许贴着墙角斜穿。**4 邻域下不起作用**（没有斜线） |
| `map_type` | `walls` | `empty` / `random` / `walls` / `maze`。**`empty` 最能看清扩展形状** |
| `map_width` / `map_height` | `40` / `28` | 栅格数 |
| `map_resolution` | `0.5` | 米/格 |
| `obstacle_ratio` | `0.18` | `random` 的障碍占据率 |
| `map_seed` | `2026` | 随机种子。按 `m` 换一张 |
| `auto_play` | `true` | **想单步调试就设 `false`**，它会停在第 0 步等你按 `n` |
| `replay_rate` | `40.0` | 播放速度（步/秒），`]` / `[` 调 |
| `show_cost_text` | `false` | 每格显示 `g` / `步数`。**地图小的时候再开**（超过 500 格自动不画） |
| `record_stale_pops` | `true` | 把"弹出重复条目"也算成一步录进 trace。**DFS 下正是看懂懒惰去重的地方**；想让回放干净就设 `false` |
| `compare_on_replan` | `true` | 每次重规划都打一遍三算法对比表 |
| `rviz` | `true` | 是否起 rviz |
| `keyboard` | `false` | 试着用 xterm 弹窗起键盘节点（镜像里默认没装 xterm，所以还是手动开第二个终端） |

起终点：`planning/start_ix` 等四个参数，`-1` = 自动（左下角 → 右上角）。或者直接在 rviz 里点。

### 想看清每格的数字，就用小地图

```bash
roslaunch search_tutorial search_demo.launch auto_play:=false show_cost_text:=true \
    map_type:=empty map_width:=20 map_height:=14 map_resolution:=1.0
```

4 邻域下每格会显示两行完全相同的数字（`g` 和 `步数`）—— **那就是 `g ≡ depth` 的样子。**
按 `v` 切到 8 邻域，它们立刻开始不一样。

### 一个环境坑：中文别走 ROS_INFO

`log4cxx` 会把每个非 ASCII 字节换成 `?`，所以 `ROS_INFO("菱形")` 会打成 `??????`。
本包所有中文输出都走自己的 `say()` / `sayLine()` 直接 `fwrite` 到 stdout。
**往这个包里加中文提示时，别用 `ROS_INFO`。**

---

## 8. 不用 ROS 也能跑

算法核心（`graph_search.cpp` + `trace_replayer.cpp`）不含一行 ROS 代码，
只依赖 STL 和 `astar_tutorial` 的 `GridMap2D`：

```bash
cd src/search_tutorial

g++ -std=c++17 -O2 -I include -I ../astar_tutorial/include \
    ../astar_tutorial/src/grid_map_2d.cpp ../astar_tutorial/src/astar.cpp \
    src/graph_search.cpp src/trace_replayer.cpp src/search_console_demo.cpp \
    -o /tmp/search_console
/tmp/search_console --map tiny --step

g++ -std=c++17 -O2 -I include -I ../astar_tutorial/include \
    ../astar_tutorial/src/grid_map_2d.cpp ../astar_tutorial/src/astar.cpp \
    src/graph_search.cpp src/trace_replayer.cpp src/search_selfcheck.cpp \
    -o /tmp/search_selfcheck
/tmp/search_selfcheck
```

`astar.cpp` 只有 selfcheck 的交叉验证要用，但两个目标都链上更省事。
（用 `-std=c++17`，别用 `c++14` —— `GridMap2D` 的 `static constexpr` 成员在 C++14 下要额外的
out-of-line 定义，会链接失败。）

想把它搬进别的项目：拷 `include/search_tutorial/` + `src/{graph_search,trace_replayer}.cpp`，
外加 `astar_tutorial` 的 `grid_map_2d.{h,cpp}`。

---

## 9. 练习：改代码，用自检验证

每一条都能用 `search_selfcheck` 判对错。

1. **把 `<` 改成 `<=`**（`graph_search.cpp` 的 Dijkstra 松弛）。
   预测：`cost` 一个字都不变，`pushes` 和 `stale_pops` 变大。跑 selfcheck 确认。
2. **把方向表 `kDX/kDY` 的前四项换个顺序**，重新编译。
   预测：`--algo dfs` 的路径完全变样，`bfs` / `dijkstra` 的代价不变，selfcheck 全过。
3. **给 BFS 加上"入队时就判终点并返回"**，看能不能通过 selfcheck。
   然后**对 Dijkstra 做同样的事**，看它在哪个实例上开始给出更长的路径（§4.3）。
4. **删掉伪代码第 4 行的 `state == CLOSED` 检查**，跑 `--algo dfs`。
   预测：重复扩展、`parent` 被反复改写、回溯成环 → 死循环或路径错乱。
5. **给 `GridMap2D` 加一个"地形代价"**（比如某些格子代价 2），让 `cost(cur,nb)` 读它。
   然后看 BFS 怎么崩、Dijkstra 怎么照样对。**这是本包最能说明"为什么要学 Dijkstra"的练习。**
6. **给第 3 行的排序键加一个 `h`**，把它变成 A*。
   4 邻域用 manhattan，8 邻域用 octile —— 然后和 `astar_tutorial` 的实现对答案。
7. **给 DFS 加上深度上限**（超过 `d_max` 就不入栈），反复加大上限重跑。
   这就是 **IDDFS（迭代加深）**，看它怎么用 DFS 的内存换回 BFS 的最优性。

---

## 10. 文件清单

```
search_tutorial/
├── include/search_tutorial/
│   ├── graph_search.h          ★ 13 行统一伪代码 + Frontier::pop()  —— 从这里开始读
│   └── trace_replayer.h        逐步回放的接口
├── src/
│   ├── graph_search.cpp        ★ 主循环，每段标了 [伪代码 N]；第 12 行的三个分支
│   ├── trace_replayer.cpp      事件序列 -> 第 k 步的完整状态（`b` 键靠它）
│   ├── search_rviz_node.cpp    rviz 可视化 + 在线换算法（不含算法逻辑）
│   ├── search_keyboard.cpp     键盘遥控（cbreak 模式，退出会还原终端）
│   ├── search_console_demo.cpp 控制台逐步演示 / --compare / --counterexample
│   └── search_selfcheck.cpp    对 3 份独立实现 + 回放器交叉验证
├── launch/
│   ├── search_demo.launch      参数表见第 7 节
│   └── search_demo.rviz        配好的 rviz 布局
└── doc/
    ├── README.md               ★ 四篇讲解的索引 + 一句话总结
    ├── 01-bfs.md               BFS 正确性证明；§1.4 4 邻域为什么让它变成最短路算法
    ├── 02-dfs.md               换个容器就是 DFS；重复条目；它到底有什么用
    ├── 03-dijkstra.md          出堆瞬间 g 已最优；§3.4 4 邻域下它退化成 BFS 的证明
    └── 04-compare.md           六张扩展形状图、实测表、3000 实例统计、选择表
```

---

## 续集

```
BFS ──换成堆、按 g 排序──> Dijkstra ──排序键加个 h──> A*
DFS   （只换容器方向）
```

本包讲**取出规则**这件事本身，以及"步数 / 代价"两个目标函数的区别。
`h` 怎么选、什么是可采纳性和一致性、A\* 的最优性证明、Weighted A\* 拿什么换什么 ——
在 **[astar_tutorial](../astar_tutorial/README.md)**。
