# D* Lite：地图变化后只修补受影响的最短路

## 1. 它解决什么问题

Dijkstra/A* 默认地图不变。障碍一变，从机器人当前位置重新搜索当然正确，但会丢掉上一轮
已经算出的全部信息。D* Lite 保存整张图的 `g` 与 `rhs`，只传播地图变化造成的局部不一致，
适合机器人边走边发现障碍的场景。

本包实现的是 **D* Lite**，不是最初版 D*。两者目标相同，但 D* Lite 的推导更短，工程实现
也更清楚。

## 2. 从终点反向维护

终点通常固定，机器人起点却会移动，因此把终点设为根：

```text
rhs(goal) = 0
rhs(u) = min(cost(u,v) + g(v))
```

`g(u)` 是当前确认值，`rhs(u)` 是根据邻居计算的一步前瞻值。两者相等表示局部一致，不相等
表示节点需要修复。机器人移动时不必换根，只用 `km` 修正优先级。

```text
best = min(g(u), rhs(u))
key(u) = [best + h(start,u) + km, best]
```

key 按字典序比较。4 邻域使用 Manhattan 启发，8 邻域使用 Octile 启发。

## 3. 修复规则

- `g(u) > rhs(u)`：找到了更便宜的后继，将 `g` 降到 `rhs`，再通知前驱。
- `g(u) < rhs(u)`：旧路径失效，先把 `g` 置为无穷，再重算自己和前驱。

新增障碍通常制造第二类不一致，删除障碍通常制造第一类不一致。修复持续到起点一致，且 OPEN
顶部不可能再改善起点为止。

本实现使用 `queue_version_` 做懒惰删除。节点每次更新都会让堆中的旧条目失效；否则已一致节点
可能被旧 key 再处理，产生振荡。

## 4. 代码对应

| 概念 | 代码 |
|---|---|
| `g/rhs/km` | `DStarLite` 成员 |
| key 计算 | `calculateKey()` |
| 局部一致性更新 | `updateVertex()` |
| 主修复循环 | `computeShortestPath()` |
| 地图变化 | `updateCell()` |
| 机器人移动 | `moveStart()` |
| 从 `g` 场提取路径 | `pathIds()` |

## 5. 运行与限制

```bash
rosrun search_tutorial advanced_search_demo
rosrun search_tutorial advanced_search_selfcheck 300
```

演示先首次规划，再在当前路径中间加入障碍。统计中的 `expanded` 是本轮实际修复的节点数。
局部变化时它通常比全新搜索小，但若变化切断关键窄通道，修复范围也可能很大。

- 边代价必须非负。
- 地图变化应通过 `updateCell()` 通知规划器。
- `setConnectivity()` 应在 `initialize()` 前调用；改变运动模型后应重新初始化。
- 当前是二值占据栅格。扩展 costmap 时需修改 `edgeCost()` 并通知所有受影响顶点。
