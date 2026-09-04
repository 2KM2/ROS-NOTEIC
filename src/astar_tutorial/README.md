# astar_tutorial —— 从零手写 A*，每一步都能看见

一个独立的 catkin 包，目标不是"跑得快"，而是**看得懂**：

- 算法核心 `src/astar.cpp` 只用 STL，逐行注释对应标准伪代码，可脱离 ROS 单独编译；
- 搜索过程被完整录成 **trace**，rviz 里可以**一步一步前进 / 后退**，看 OPEN / CLOSED / g / h / f 怎么变；
- 一个 ASCII 控制台版，边跑边打地图和数值，最适合抠细节；
- 一个自检程序，用 Dijkstra 交叉验证 A* 的最优性 —— 保证你学的是**对的** A*。

和作业里的 `grid_path_searcher` 无关，可以对照着看（那份是 3D + JPS 的参考实现，注释很少）。

> **这份 README 讲"怎么用"。想搞懂"为什么"** —— 最短路问题、Dijkstra 的正确性证明、
> A* 的最优性证明、可采纳性 / 一致性、实测对比和反例 —— **看 [doc/](doc/README.md)。**

---

## 1. 三十秒跑起来

```bash
cd docker && ./build_ws.sh          # 编译（已验证零 error）

# 终端 1：可视化 + rviz
./run.sh
roslaunch astar_tutorial astar_demo.launch

# 终端 2：键盘单步控制（要独立的 stdin，所以单独开）
cd docker && ./run.sh
rosrun astar_tutorial astar_keyboard
```

键盘：`空格`下一步　`b`上一步　`p`播放/暂停　`r`回到开头　`f`跳到结尾
`[`/`]`调速　`t`切换 g/h/f 文字　`m`换地图　`c`重新规划　`q`退出

rviz 里用工具栏的 **2D Pose Estimate** 设起点、**2D Nav Goal** 设终点，点完立刻重新搜索。

不想用键盘也行，服务同样能控制：

```bash
rosservice call /astar/step        # 下一步
rosservice call /astar/back        # 上一步
rosservice call /astar/play_pause
rosservice call /astar/reset
rosservice call /astar/finish
rosservice call /astar/new_map
```

### 颜色对照

| 颜色               | 含义                                                      |
| ------------------ | --------------------------------------------------------- |
| 灰色格子           | 障碍（`nav_msgs/OccupancyGrid`）                        |
| **蓝色**     | CLOSED —— g 已经确定是最优，永远不会再改                |
| **绿色**     | OPEN —— 已发现，在堆里排队等着出堆                      |
| **红色柱子** | 本步正在扩展的节点（OPEN 里 f 最小的那个）                |
| **黄色**     | 本步检查过的 8 个邻居                                     |
| **青色细线** | 起点 → 当前节点的**临时**父链，会随 g 被改小而抖动 |
| **橙色粗线** | 最终路径                                                  |
| 紫球 / 青球        | 起点 / 终点                                               |
| 左上白字           | 步数、当前节点的 g/h/f、OPEN 和 CLOSED 大小、统计         |

---

## 2. 强烈建议：先在终端里啃一遍

rviz 好看，但数值看不清。真正理解每一步，用这个：

```bash
./run.sh
# 8x6 小地图，一步一步按回车，每步打印"取出谁、每个邻居怎么判、地图长啥样"
rosrun astar_tutorial astar_console_demo --map tiny --step --no-skips
```

输出长这样（真实输出，未删改）：

```
=== 第 2/16 步 ===
① 从 OPEN 里取出 f 最小的节点: (1,1)  g=1.414  h=7.657  f=9.071
② 把它标记为 CLOSED（g 已确定，以后不再改）
③ 逐个检查 8 个邻居（其中 4 个被跳过，--no-skips 已隐藏）:
     ( 2, 1) 边代价1.000  新发现,入OPEN       g: inf -> 2.414   h=6.657   f=9.071
     ( 1, 2) 边代价1.000  新发现,入OPEN       g: inf -> 2.414   h=7.243   f=9.657
     ( 2, 0) 边代价1.414  新发现,入OPEN       g: inf -> 2.828   h=7.071   f=9.899
     ( 0, 2) 边代价1.414  新发现,入OPEN       g: inf -> 2.828   h=8.243   f=11.071
④ 本步结束: OPEN=6  CLOSED=2
. . . . . . . G
. . # # # . . .
. . # . . . . .
o o # . # # # .
o @ o . # . . .
S o o . . . . .
```

`--map tiny` 只有 48 格，**拿纸笔跟着算一遍**，A* 就再也不会忘了。

其他玩法：

```bash
# 各启发函数的横向对比表（下面第 5 节详解）
rosrun astar_tutorial astar_console_demo --map empty --size 60 40 --compare

# 只详细看第 40~45 步，前面的快速跳过
rosrun astar_tutorial astar_console_demo --map walls --size 40 20 --range 40 45

# 4 邻域 + 曼哈顿（这是 4 邻域下的正确搭配）
rosrun astar_tutorial astar_console_demo --map random --conn 4 --h manhattan

--help 看全部参数
```

---

## 3. 代码结构：从哪儿开始读

按这个顺序读，一遍就够：

| 顺序 | 文件                                     | 读什么                                                        |
| ---- | ---------------------------------------- | ------------------------------------------------------------- |
| 1    | `include/astar_tutorial/astar.h`       | 开头的**伪代码 12 行** + g/h/f 三个概念 + 可采纳性那段  |
| 2    | `src/astar.cpp` 的 `search()`        | 主体。每个环节都标了`[伪代码 N]`，和第 1 步的伪代码一一对应 |
| 3    | `src/astar.cpp` 的 `rawHeuristic()`  | 四种启发函数，为什么 8 邻域要用 octile                        |
| 4    | `src/astar.cpp` 的 `pathIds()`       | 沿 parent 回溯                                                |
| 5    | `include/astar_tutorial/grid_map_2d.h` | 三套坐标（栅格索引 / 一维下标 / 世界坐标）的换算              |
| 6    | `src/trace_replayer.cpp`               | 怎么把过程"重放"出来（和算法无关，只是可视化的支撑）          |
| 7    | `src/astar_rviz_node.cpp`              | ROS 胶水层：造地图、接点击、画 Marker。**不含任何算法** |

`search()` 里对应伪代码的锚点（改代码时搜 `[伪代码` 就能定位）：

```
[伪代码 1]     起点入 OPEN，g=0，f=h
[伪代码 2]     while (!open.empty())
[伪代码 3]     open.top() —— 取 f 最小
[伪代码 4]     标记 CLOSED（附带"为什么 CLOSED 的节点可以放心跳过"的证明）
[伪代码 5]     终点判断（附带"为什么必须等它出堆才能返回"）
[伪代码 6]     遍历邻居
[伪代码 7]     越界 / 障碍 / 斜穿墙角 / 已 CLOSED -> 跳过
[伪代码 8]     tentative_g = g[cur] + edge_cost
[伪代码 9-11]  更短就更新 g、f、parent，重新入堆
[伪代码 12]    OPEN 抽干 -> 无解
```

---

## 4. 七个最容易写错 / 最值得琢磨的点

代码里都有对应注释，这里汇总一下。

### 4.1 终点必须"出堆"才算到达，不能"入堆"就返回

第一次把终点放进 OPEN 时，它的 g 只是**某一条路**的代价，不一定最优。
只有等它以最小 f 出堆，才能确定没有更短的路。提前返回是最常见的
"A* 路径偏长" bug，而且肉眼几乎看不出来。→ `astar.cpp` 的 `[伪代码 5]`

### 4.2 为什么 CLOSED 里的节点可以直接跳过

因为**只要 h 可采纳，节点出堆的那一刻 g 就已经最优了**。
反证：若还存在一条更短的路通向 `cur`，那条路上必有节点还在 OPEN 里，
而它的 `f = g + h ≤ 真实总代价 ≤ f(cur)`，于是它会先于 `cur` 出堆 —— 矛盾。
→ `astar.cpp` 的 `[伪代码 4]` 上方注释

### 4.3 8 邻域千万别用曼哈顿距离

斜走一步真实代价 √2≈1.41，曼哈顿算成 2 —— **高估了**，于是不可采纳，A* 不再保证最短路。
这不是理论洁癖，自检程序实测：1599 次不可采纳的搜索里，**183 次真的给出了更长的路径，最差 1.29 倍**。

```bash
# 亲眼看一次
rosrun astar_tutorial astar_console_demo --map walls --size 60 40 --compare
roslaunch astar_tutorial astar_demo.launch heuristic:=manhattan   # 节点会打 warn
```

代码里 `isHeuristicAdmissible()` / `admissibilityWarning()` 会主动警告你。

### 4.4 OPEN 用什么数据结构：懒惰删除 vs multimap

标准 A* 需要 decrease-key（g 变小时把节点在堆里往上调），`std::priority_queue` 不支持。两种做法：

| 做法                 | 怎么实现                                                                             | 用在哪                                          |
| -------------------- | ------------------------------------------------------------------------------------ | ----------------------------------------------- |
| **懒惰删除**   | 直接再 push 一份新的`(f, id)`；出堆时若节点已 CLOSED 或条目 f 已过期，就当垃圾丢掉 | 本包（`stats.stale_pops` 就是这份开销）       |
| `multimap<f, ptr>` | 存迭代器，更新时 erase + insert                                                      | 作业的`grid_path_searcher/graph_searcher.cpp` |

两者结果**完全一致**，只是常数和内存不同。想对照就打开那个文件比一比。

### 4.5 `<` 还是 `<=`

`tentative_g < nb.g` 用严格小于。相等时换父亲不改变代价，只是白白多一次 push，
还让搜索过程随实现细节抖动。作业参考实现写的是 `<=`，想看差别就改 `astar.cpp` 里那一行
（注释已标出位置），再跑 `astar_selfcheck` —— 代价不会变，扩展数会变。

### 4.6 斜穿墙角（corner cutting）

```
. #        从 c 斜着走到 X，路径正好从两个障碍的夹缝里穿过。
c X        真机器人有体积，穿不过去 -> 默认禁止。
```

默认要求两个正交邻居都空闲才允许斜走。想看区别：

```bash
rosrun astar_tutorial astar_console_demo --map corner --corner-cut
roslaunch astar_tutorial astar_demo.launch allow_corner_cutting:=true
```

### 4.7 "地图连通"要和 A* 的规则对齐

`GridMap2D::isConnected()` 是个纯洪水填充，节点用它在造完随机地图后确认起终点确实通
（不通就自动换 `map/seed` 重试，并在终端里告诉你换到了哪个种子）。

这里踩过一个坑，值得记住：**连通性判断必须和 A* 用同一套规则**。
最早那版洪水填充允许斜穿墙角，A* 不允许，于是出现了"检查说连通、A* 说无解"的自相矛盾 ——
表现就是 demo 一启动就显示"OPEN 抽干，无路径"。

推论：`allow_corner_cutting` 改变的不只是路径代价，它改变的是**哪些格子彼此可达**。
只能靠斜挤过两个障碍夹缝的走廊，在禁止斜穿时是一堵墙。

顺带一提，`isConnected()` 就是把 A* 的 g/h/f 和优先队列全部拿掉之后剩下的骨架，
和 `search()` 并排读，能很清楚地看出"启发式搜索"到底在朴素搜索之上加了什么。

---

## 5. 启发函数怎么选：一张实测表

同一张 60×40 空地图，起点 (0,0) → 终点 (59,39)，8 邻域：

```
heuristic                 expanded    pushes         cost       ms  admis.
--------------------------------------------------------------------------------
zero (=Dijkstra)              2400      2400      75.1543    0.237     yes
euclidean                      983      1907      75.1543    0.151     yes
manhattan                       60       234      75.1543    0.018      NO
diagonal(octile)                68       268      75.1543    0.021     yes
diagonal + tie 1.0001           60       234      75.1543    0.016      NO
diagonal, weight 1.5            60       234      75.1543    0.016      NO
diagonal, weight 3.0            60       234      75.1543    0.016      NO
```

空地图上没有绕路的余地，所以连不可采纳的几行也碰巧拿到了最优代价 —— **别被它骗了**，
换成有墙的地图立刻露馅（见下表）。

**2400 → 68**，扩展数少了 35 倍，路径代价一模一样。这就是 A* 相对 Dijkstra 的全部意义。

同一张图加上墙（`--map walls`）：

```
heuristic                 expanded    pushes         cost       ms  admis.
--------------------------------------------------------------------------------
zero (=Dijkstra)              2069      2083     105.1543    0.285     yes
euclidean                     1682      2592     105.1543    0.307     yes
manhattan                     1521      2624     108.0833    0.324      NO   <- 路径比最优解长!
diagonal(octile)              1542      2563     105.1543    0.310     yes
diagonal + tie 1.0001         1542      2547     105.1543    0.252      NO
diagonal, weight 1.5          1473      2538     107.1543    0.273      NO   <- 路径比最优解长!
diagonal, weight 3.0           718      1439     110.5685    0.154      NO   <- 路径比最优解长!
```

两个结论：

1. **障碍越多，启发函数越不管用**。h 只会"往终点方向指"，遇到墙照样得把死胡同全填一遍。
   这正是 JPS / 可视图 / RRT 等方法要解决的问题。
2. **weight=3 扩展数少了三分之二，路径长了 5.2%**。Weighted A* 的路径长度上界就是 w 倍最优解，这是明码标价的交易。

选择建议：

- 4 邻域 → **manhattan**（就是真实值）
- 8 邻域 → **diagonal / octile**（就是真实值）
- 想要最优就别动 `weight` 和 `tie_breaker`；想快就加 `weight`，并且清楚自己牺牲了什么
- `h ≡ 0` → 就是 Dijkstra，永远拿它当"最优代价"的基准

`tie_breaker=1.0001` 那行被标成"不可采纳"、代价却照样最优 —— 这不矛盾：
可采纳性是**保证**最优的充分条件，不是必要条件。放弃它就等于放弃保证，
具体这一次亏没亏要看运气。要不要接受这种"通常没事"，是工程判断。

---

## 6. 自检：怎么知道这份 A* 是对的

```bash
rosrun astar_tutorial astar_selfcheck 400
```

它在 400 张随机地图上跑 4/8 邻域 × 5 种启发配置，断言：

1. `h≡0` 的 A* 就是 Dijkstra，其代价视为**真值**；
2. **所有可采纳的启发函数必须给出和真值完全相同的代价**（这是 A* 最优性的核心）；
3. 不可采纳的组合代价 ≥ 真值（比真值还小就说明基准算错了）；
4. 路径本身合法：首尾正确、相邻格真相邻、不穿障碍、不违反斜穿规则、代价累加对得上 g[goal]；
5. 有解 / 无解的判断和 Dijkstra 一致。

实测输出：

```
=== A* 自检：400 张随机地图，每张 4/8 邻域 × 5 种启发配置 ===

统计:
  有解的搜索      : 598 次
  无解的搜索      : 53 次 (两种算法都判无解)
  不可采纳的配置  : 跑了 1599 次, 其中 183 次真的给出了比最优解更长的路径 (最差 1.2857 倍)
     ^ 这一行不是 bug: 它正是"h 高估 => 丢掉最优性"的实验证据。

全部通过：可采纳的启发函数下，手写 A* 的代价和 Dijkstra 逐位相同。
```

**改完代码就跑一遍这个。** 它能抓住所有"路径还是通的、但不再最短"的退化。

---

## 7. launch 参数

```bash
roslaunch astar_tutorial astar_demo.launch \
    map_type:=maze map_width:=40 map_height:=30 map_resolution:=1.0 \
    heuristic:=diagonal use_8_connected:=true weight:=1.0 \
    replay_rate:=10 show_cost_text:=true
```

| 参数                         | 默认         | 说明                                                    |
| ---------------------------- | ------------ | ------------------------------------------------------- |
| `map_type`                 | `walls`    | `empty` / `random` / `walls` / `maze`           |
| `map_width` `map_height` | 60 / 40      | 栅格数                                                  |
| `map_resolution`           | 0.5          | 米/格                                                   |
| `obstacle_ratio`           | 0.18         | `random` 的障碍占据率                                 |
| `map_seed`                 | 2026         | 随机种子（同种子结果完全一致，便于对比）                |
| `heuristic`                | `diagonal` | `zero` / `manhattan` / `euclidean` / `diagonal` |
| `use_8_connected`          | true         | false = 4 邻域                                          |
| `allow_corner_cutting`     | false        | 允许斜穿墙角                                            |
| `tie_breaker`              | 1.0          | 试 1.0001                                               |
| `weight`                   | 1.0          | Weighted A*                                             |
| `replay_rate`              | 60.0         | 回放速度（步/秒），运行中用`[` `]` 调               |
| `auto_play`                | true         | false = 启动就停在最后一帧                              |
| `show_cost_text`           | false        | 每格显示 g/h/f 三行数字                                 |
| `rviz`                     | true         | 是否自动起 rviz                                         |

### 想看清 g/h/f 数字就用小地图

文字标签在几千个格子上会拖死 rviz，所以有 `max_text_labels`(500) 上限。推荐：

```bash
roslaunch astar_tutorial astar_demo.launch \
    map_width:=20 map_height:=14 map_resolution:=1.0 \
    show_cost_text:=true replay_rate:=2
```

### 一个环境坑：中文别走 ROS_INFO

这个镜像的 locale 下，rosconsole 底层的 log4cxx 会把每一个非 ASCII **字节**替换成 `?`，
中文提示全变成 `?????`（`rosrun` 和 `roslaunch` 都一样，跟终端没关系）。
所以 `astar_rviz_node.cpp` 里的教学输出走的是自己的 `say()` / `sayLine()`，直接写 stdout；
`ROS_WARN`/`ROS_ERROR` 只留给纯 ASCII 的消息。

后果：这些中文提示**不会**进 `/rosout` 和 `~/.ros/log/`，只在你启动节点的那个终端里可见。
自己往这个包里加中文日志时记得用 `say()`，别用 `ROS_INFO`。

---

## 8. 不用 ROS 也能跑

算法核心（`grid_map_2d.cpp` + `astar.cpp` + `trace_replayer.cpp`）不含一行 ROS 代码：

```bash
cd src/astar_tutorial
g++ -std=c++17 -O2 -I include src/grid_map_2d.cpp src/astar.cpp src/trace_replayer.cpp \
    src/astar_console_demo.cpp -o /tmp/astar_console
/tmp/astar_console --map tiny --step

g++ -std=c++17 -O2 -I include src/grid_map_2d.cpp src/astar.cpp \
    src/astar_selfcheck.cpp -o /tmp/astar_selfcheck
/tmp/astar_selfcheck
```

想把它搬进别的项目，拷 `include/astar_tutorial/` + `src/{grid_map_2d,astar}.cpp` 就够了。

---

## 9. 练习：改代码，用自检验证

按难度排：

1. **`<=` 改 `<`**：`astar.cpp` 里 `tentative_g < nb.g - kEps` 改成 `<=`，跑 selfcheck。代价变了吗？扩展数变了吗？
2. **提前返回**：把终点判断从"出堆后"挪到"邻居入 OPEN 时"，跑 selfcheck。它应该开始报 FAIL —— 亲眼看看 4.1 说的 bug。
3. **斜线代价写成 1**：把 `ev.edge_cost = diagonal ? kSqrt2 : 1.0` 改成恒为 1.0，看 selfcheck 怎么骂你。
4. **换 OPEN 的实现**：把 `priority_queue` + 懒惰删除换成 `std::multimap` + 迭代器（照 `grid_path_searcher` 那份写），对比 `stale_pops` 和耗时。
5. **加新启发函数**：比如 `h = 0.5 × octile`（故意保守），观察扩展数变化 —— 它仍然可采纳，所以代价必须不变。
6. **扩到 3D**：`GridMap2D` → `GridMap3D`，邻居从 8 个变 26 个，octile 距离推广成三维版
   `√3·dmin + √2·(dmid−dmin) + (dmax−dmid)`。改完就能直接对上作业的 `grid_path_searcher`。
7. **接真实地图**：让节点订阅外部的 `nav_msgs/OccupancyGrid` 替代自己造图。

---

## 10. 文件清单

```
src/astar_tutorial/
├── include/astar_tutorial/
│   ├── astar.h              ★ 伪代码 + 概念 + 数据结构，从这里开始读
│   ├── grid_map_2d.h          栅格地图，三套坐标的换算
│   └── trace_replayer.h       过程重放（可视化支撑）
├── src/
│   ├── astar.cpp            ★ 核心实现，每段标了 [伪代码 N]
│   ├── grid_map_2d.cpp        地图 + 随机地图/迷宫生成 + ASCII 渲染
│   ├── trace_replayer.cpp     按事件重放出第 k 步的状态
│   ├── astar_rviz_node.cpp    ROS 胶水层（不含算法）
│   ├── astar_keyboard.cpp     终端键盘遥控
│   ├── astar_console_demo.cpp 纯终端逐步演示 + 启发函数对比表
│   └── astar_selfcheck.cpp    用 Dijkstra 交叉验证最优性
├── launch/
│   ├── astar_demo.launch
│   └── astar_demo.rviz
├── doc/                      ★ 原理讲解（证明 + 实测对比），和本文件互补
│   ├── README.md               索引 + "Dijkstra 就是 h≡0 的 A*" + 概念到代码的对应表
│   ├── 01-dijkstra.md          最短路问题、BFS 为什么不够、Dijkstra 正确性证明
│   ├── 02-a-star.md            h/f 的引入、可采纳性与一致性、A* 最优性证明、Weighted A*
│   └── 03-compare.md           实测数据、扩展形状图、曼哈顿走错的可复现反例、选型表
├── CMakeLists.txt
├── package.xml
└── README.md
```
