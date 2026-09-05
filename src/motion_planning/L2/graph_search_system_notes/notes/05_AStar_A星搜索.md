# 05 A* Search：A 星搜索

## 1. A* 是什么

A* 是非常经典的启发式搜索算法。

它把：

```text
已经付出的代价
```

和：

```text
未来预计需要的代价
```

结合起来。

---

## 2. 核心公式

```text
f(n) = g(n) + h(n)
```

其中：

### g(n)

从起点到当前节点的真实代价。

### h(n)

从当前节点到目标的估计代价。

### f(n)

经过当前节点最终到达目标的总代价估计。

---

## 3. 小白理解

假设你从家开车去学校。

有两条路线：

### 路线 B

已经走：

```text
g(B)=5 km
```

预计剩下：

```text
h(B)=10 km
```

所以：

```text
f(B)=15
```

### 路线 C

已经走：

```text
g(C)=10 km
```

预计剩下：

```text
h(C)=2 km
```

所以：

```text
f(C)=12
```

A* 会先扩展 C。

---

## 4. A* 和 UCS / Greedy 的关系

UCS：

```text
f(n) = g(n)
```

只看已经花了多少。

Greedy：

```text
f(n) = h(n)
```

只看离目标还多远。

A*：

```text
f(n) = g(n) + h(n)
```

两者一起考虑。

---

## 5. C++ 核心数据结构

优先队列中保存：

```text
(f值, node)
```

每次弹出：

```text
f值最小的节点
```

---

## 6. 核心模板

```cpp
vector<int> g(n, INF);

g[start] = 0;

pq.push({
    g[start] + heuristic[start],
    start
});

while (!pq.empty())
{
    auto [f, current] = pq.top();
    pq.pop();

    if (current == goal)
        break;

    for (auto [neighbor, weight] : graph[current])
    {
        int newG = g[current] + weight;

        if (newG < g[neighbor])
        {
            g[neighbor] = newG;

            int newF =
                newG + heuristic[neighbor];

            pq.push({
                newF,
                neighbor
            });
        }
    }
}
```

---

## 7. 好的 heuristic 有什么要求

理想情况下：

```text
h(n)
```

应该尽量接近真实剩余代价。

但如果想保证 A* 的最优性，常见要求之一是：

> h(n) 不高估实际剩余代价。

这称为：

```text
Admissible Heuristic
可采纳启发函数
```

---

## 8. 特殊情况

如果：

```text
h(n)=0
```

那么：

```text
f(n)=g(n)
```

A* 就退化成：

```text
UCS / Dijkstra
```

---

## 9. 常见用途

- 游戏寻路
- 地图导航
- 机器人路径规划
- AI 搜索
- 网格地图最短路

---

## 10. 一句话记忆

A*：

> 不只看已经花了多少，也不只看还剩多少，而是看“已经花的 + 预计还要花的”。
