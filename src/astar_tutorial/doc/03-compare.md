# 三、实测对比：把结论跑出来

前两篇的所有结论，这一篇全部用这个包的程序验证。**每段输出都是真实跑出来的，命令都给了。**

---

## 3.1 同一张图，同一条路，两种搜索形状

21×11 空地图，`(1,5) → (19,5)`，8 邻域。两条命令只差 `--h`：

```bash
rosrun astar_tutorial astar_console_demo --map empty --size 21 11 --start 1 5 --goal 19 5 --h zero
rosrun astar_tutorial astar_console_demo --map empty --size 21 11 --start 1 5 --goal 19 5 --h diagonal
```

`x` = CLOSED，`o` = OPEN，`*` = 最终路径，`.` = 从没碰过。

**Dijkstra（`--h zero`）：**

```
x x x x x x x x x x x x x x x x x o o . .
x x x x x x x x x x x x x x x x x x o . .
x x x x x x x x x x x x x x x x x x o o .
x x x x x x x x x x x x x x x x x x x o .
x x x x x x x x x x x x x x x x x x x o .
x S * * * * * * * * * * * * * * * * * G .
x x x x x x x x x x x x x x x x x x x o .
x x x x x x x x x x x x x x x x x x x o .
x x x x x x x x x x x x x x x x x x o o .
x x x x x x x x x x x x x x x x x x o . .
x x x x x x x x x x x x x x x x x o o . .
```

**A\*（`--h diagonal`）：**

```
. . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .
o o o o o o o o o o o o o o o o o o o o .
o S * * * * * * * * * * * * * * * * * G .
o o o o o o o o o o o o o o o o o o o o .
. . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . .
```

| | 扩展节点 | 入堆 | OPEN 峰值 | 代价 | 耗时 |
|---|---|---|---|---|---|
| `zero`（Dijkstra） | **202** / 231 格 | 216 | 25 | 18.0000 | 0.094 ms |
| `diagonal`（A\*） | **19** / 231 格 | 60 | 42 | 18.0000 | 0.037 ms |

**代价一模一样，扩展数 202 → 19（少了 90%）。**

这两张图值得盯久一点。Dijkstra 把**整张地图**都染成了 `x`，包括左边、上边、下边 ——
那些方向和终点毫无关系。A* 只扩展了那条 19 格的走廊本身（正好是路径的长度，
一格没多走），周围一圈 `o` 是被发现但还没轮到、也永远不会轮到的候选。

A* 只碰了 3 行，甚至连第 1~4 行和第 8~11 行都没被"发现"过。
这就是 `h` 唯一在做的事：**告诉搜索"别往那边看"**。

有一个反直觉的数字：A\* 的 OPEN 峰值（42）比 Dijkstra（25）**还大**。
不矛盾 —— Dijkstra 的 OPEN 是一圈不断往外推的"波前"，走过的地方立刻变 CLOSED，
所以波前一直保持差不多的长度；A\* 沿走廊推进时，两侧那两长排 `o` 全程留在 OPEN 里
永不出堆，越走越多。**A\* 省的是扩展次数和时间，不一定省内存**。

---

## 3.2 有障碍时会怎样：`--map tiny`（8×6，48 格）

```bash
rosrun astar_tutorial astar_console_demo --map tiny --h zero        # / manhattan / diagonal
```

**Dijkstra**（39 扩展）：

```
x x x x x x x G
x x # # # x * x
x x # * * * x x
x x # * # # # x
x x x * # x x x
S * * x x x x x
```

**A\* octile**（16 扩展）：

```
. . . . . o o G
o o # # # o * o
o x # * * * o o
o x # * # # # .
x * * * # . . .
S x x x o . . .
```

**A\* 曼哈顿**（10 扩展，但**不可采纳**，程序会打警告）：

```
. . . . . o o G
. . # # # o * o
. . # * * * o o
o o # * # # # .
o * * * # . . .
S o o o . . . .
```

| h | 扩展 | 入堆 | 过期出堆 | 代价 | 可采纳 |
|---|---|---|---|---|---|
| `zero` | 39 / 48 | 39 | 0 | 10.2426 | ✅ |
| `manhattan` | **10** | 22 | 0 | 10.2426 | ❌ |
| `diagonal` | 16 | 30 | 1 | 10.2426 | ✅ |

三个都给出了 10.2426 —— **别急着下结论**。这张图小、障碍摆得也巧，
曼哈顿这次侥幸没亏。它下一次会亏，见 3.3。

顺便注意 `diagonal` 那行的"过期出堆 1 次"：这就是懒惰删除的全部代价 ——
30 次入堆里有 1 条是垃圾。换用 `multimap` 能省掉这 1 次，代价是多一倍代码。

---

## 3.3 反例：8 邻域下曼哈顿真的会走错

上一节没抓到，换张有障碍的随机图就抓到了。**可复现，种子写死：**

```bash
rosrun astar_tutorial astar_console_demo --map random --size 16 10 --seed 2 --ratio 0.18 --h zero
rosrun astar_tutorial astar_console_demo --map random --size 16 10 --seed 2 --ratio 0.18 --h manhattan
rosrun astar_tutorial astar_console_demo --map random --size 16 10 --seed 2 --ratio 0.18 --h diagonal
```

**Dijkstra —— 真值，代价 20.4853，19 个格子：**

```
# # x x x x x x # # x x x x x G
x x x x x x x x x x x x x x * x
x x x # # x x x x x x x x * x x
x x x # # x x x x x x x * x x x
x x x x # # # # x x x * x x x x
x x x x # # # # x x * x x x x x
x x x x x x # # * * x x x x x x
x x x x x # # # * # # x x x x x
x x x x x # # # * # # x x x x x
S * * * * * * * * x x x x x x x
```

**A\* octile —— 代价 20.4853，19 个格子，和真值逐位相同：**

```
# # . . . . . . # # . . . o o G
. . . . . . . . . . . . o o * o
. . . # # . . . . . . o o * o o
. o o # # . . . . o o o * o o .
o o x x # # # # o o * * o o . .
o x x x # # # # o * x o o . . .
x x x x x x # # * x o o . . . .
x x x x x # # # * # # . . . . .
x x x x x # # # * # # . . . . .
S * * * * * * * * x o . . . . .
```

**A\* 曼哈顿 —— 代价 22.2426，22 个格子。走错了。**

```
# # o x x x x x # # o * * * * G
. o * * * * * * * * * o o o o o
. o * # # o o o o o o o . . . .
. o * # # . . . . . . . . . . .
. o * x # # # # . . . . . . . .
. o * x # # # # . . . . . . . .
. o * x x x # # . . . . . . . .
o o * x x # # # . # # . . . . .
o * o o o # # # . # # . . . . .
S o o . . . . . . . . . . . . .
```

| h | 扩展 | 代价 | 格子数 | 相对最优 |
|---|---|---|---|---|
| `zero`（真值） | 132 / 160 | **20.4853** | 19 | 1.000 |
| `diagonal` | 43 | **20.4853** | 19 | **1.000** ✅ |
| `manhattan` | 34 | **22.2426** | 22 | **1.086** ❌ 长了 8.6% |

看路径形状：真值和 octile 都是"先沿底边往右、再从中间的缺口斜着爬上去"。
曼哈顿版走了**完全不同的一条路** —— 贴着左边墙一路爬到顶，再沿着顶边横穿过去。

为什么？曼哈顿把每个斜走高估成 2，于是"斜着切过去"在它眼里比实际贵 41%，
它就宁愿走直角。可真实代价是按 `√2` 结算的，账单最后由路径长度来付。

**注意它扩展得更少（34 < 43）。** 这是最危险的地方：不可采纳的 h 通常**看起来更快**。
如果你只盯着 "expanded" 这个指标调参，会一路调到一个又快又错的配置上，
而且路径肉眼看着完全正常 —— 它是通的，只是不是最短的。

这就是 `astar_selfcheck` 必须存在的理由。它在 400 张随机图上跑：

```
不可采纳的配置  : 跑了 1599 次, 其中 183 次真的给出了比最优解更长的路径 (最差 1.2857 倍)
```

约 11% 的搜索受害，最差一次路径长了 **28.6%**。

---

## 3.4 障碍越多，h 越不管用

同一张 60×40 地图，空的和带墙的（`--compare` 一次跑完所有配置）：

```bash
rosrun astar_tutorial astar_console_demo --map empty --size 60 40 --compare
rosrun astar_tutorial astar_console_demo --map walls --size 60 40 --compare
```

**空地图：**

```
heuristic                 expanded    pushes         cost       ms  admis.
zero (=Dijkstra)              2400      2400      75.1543    0.237     yes
euclidean                      983      1907      75.1543    0.151     yes
diagonal(octile)                68       268      75.1543    0.021     yes
```

**带墙：**

```
heuristic                 expanded    pushes         cost       ms  admis.
zero (=Dijkstra)              2069      2083     105.1543    0.285     yes
euclidean                     1682      2592     105.1543    0.307     yes
diagonal(octile)              1542      2563     105.1543    0.310     yes
```

| 地图 | Dijkstra | A\* octile | 提速比 |
|---|---|---|---|
| 空 | 2400 | **68** | **35×** |
| 带墙 | 2069 | **1542** | **1.34×** |

**从 35 倍掉到 1.34 倍。** 这是 A* 最需要认清的一件事。

原因：`h` 只知道"终点在哪个方向"，它对障碍**一无所知**。
遇到一个朝向终点的死胡同，`h` 会热情地把 A* 往里推，
A* 只能老老实实把整个死胡同填满，才能算清"这条不通"。

障碍越复杂，`h` 和 `h*` 的差距越大，A* 就越退化回 Dijkstra。极端情况（螺旋迷宫）
它可以和 Dijkstra 一样慢。想验证：`--map maze`。

**这一条决定了后续所有算法的走向**：既然瓶颈是"h 不懂障碍"，出路就是

- 让 h 懂障碍（预计算的启发式、分层规划的粗层代价）；
- 不逐格扩展（**JPS** 跳点搜索，在开阔区一跳跳到障碍边缘）；
- 干脆放弃栅格（可视图、RRT / RRT* 采样法）。

注意 `euclidean` 在带墙图上耗时（0.307 ms）居然比 `octile`（0.310 ms）几乎一样、
比 Dijkstra 还慢一点点 —— 扩展数少了，但每次要算平方根，单次更贵。
**扩展数不等于耗时**，优化时两个都得看。

---

## 3.5 手算规模：5×3，4 邻域

想彻底看清"排序键从 `g` 换成 `g+h`"这一处改动的后果，用最小的图：

```bash
rosrun astar_tutorial astar_console_demo --map empty --size 5 3 --conn 4 --start 0 1 --goal 4 1 --h zero      --step --no-skips
rosrun astar_tutorial astar_console_demo --map empty --size 5 3 --conn 4 --start 0 1 --goal 4 1 --h manhattan --step --no-skips
```

| | 步数 | 扩展 | 每步出堆的键 |
|---|---|---|---|
| `zero`（Dijkstra） | 12 | 12 / 15 格 | `g = 0,1,1,1,2,2,2,3,3,3,4,4` —— **单调不减** |
| `manhattan`（h 精确） | **5** | 5 / 15 格 | `f = 4.000, 4.000, 4.000, 4.000, 4.000` —— **恒定** |

两行右边那列各自印证了一个定理：

- Dijkstra 出堆的 `g` 单调不减 → [01-dijkstra.md §1.3](01-dijkstra.md) 的证明推论；
- `h` 精确时 `f` 沿最优路是常数 → [02-a-star.md §2.7](02-a-star.md)。

4 邻域下曼哈顿距离**就是** `h*`，所以 A* 一格也没走错：5 步走完 5 格的路。
这是 A* 的理论下限 —— 不可能比这更好了。

**建议真的按回车一步一步看完这 5 步**，每一步都能拿纸笔核对。看懂这 5 步，A* 就懂了。

---

## 3.6 什么场合用哪个

| 场合 | 用 | 为什么 |
|---|---|---|
| 单起点 → 单终点，有几何信息 | **A\*** | h 免费，白拿几倍到几十倍提速 |
| 需要**距离场**（每格到某点的距离） | **Dijkstra** | 反正要算全图，h 没用；ESDF、势场法、Voronoi 都是这类 |
| 一个起点 → **多个**候选终点 | **Dijkstra** | 一次搜索全拿到。想用 A* 就得 h 取所有终点里的最小值，会很弱 |
| 代价来自查表/学习，没有几何意义 | **Dijkstra** | 构造不出可采纳的 h |
| 状态空间抽象（拼图、任务规划） | **A\*** | 关键就在设计 h，那才是主要工作量 |
| 要求实时、可接受次优 | **Weighted A\* / ARA\*** | 路径长度有 `≤ w·C*` 的明码上界 |
| 大片开阔栅格 | **JPS** | 免掉开阔区里所有等价的对称扩展 |
| 环境会变（地图更新、动态障碍） | **D\* Lite** | 增量修复上次的搜索结果，不用从头重来 |
| 高维（机械臂、带朝向和速度的车） | **RRT / RRT\* / kinodynamic** | 栅格数随维度指数爆炸，采样法绕开这个问题 |

一条实用的经验：

> **先用 Dijkstra（`--h zero`）跑一遍拿到真值，再换 A* 比对。**
> 代价对不上，就是 h 的问题，不是 A* 的问题。

这正是 `astar_selfcheck` 的全部工作原理。你自己写规划器时也该这么干 ——
它抓得住"路径还是通的、但不再最短"这类几乎不可能靠肉眼发现的退化。

---

## 3.7 往下走

理解顺序建议：

1. **JPS**（Jump Point Search）—— 直接对上作业里的 `grid_path_searcher`。
   核心思路：开阔栅格上大量路径是对称等价的，只在"跳点"处扩展就够了。
2. **Weighted A\* → ARA\***（Anytime Repairing A\*）—— 先粗后精，随时可中断取当前最好解。
3. **D\* Lite** —— 地图局部变化时增量重规划，机器人边走边更新。
4. **Hybrid A\*** —— 把车辆运动学（转弯半径）塞进邻居扩展里，输出可执行的路。
5. **采样法**（RRT / RRT* / Informed RRT*）—— 高维空间的另一条路线。

这五个全都以本篇的 A* 为基础。回头看 [02-a-star.md](02-a-star.md) 的
可采纳性 / 一致性 / 有界次优三个概念，它们在上面每一个算法里都会再出现一次。
