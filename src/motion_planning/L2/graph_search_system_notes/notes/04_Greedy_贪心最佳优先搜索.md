# 04 Greedy Best-First Search：贪心最佳优先搜索

## 1. 核心思想

Greedy 搜索每次选择：

```text
h(n) 最小
```

的节点。

---

## 2. 什么是 h(n)

```text
h(n)
```

叫做：

> heuristic function，启发式函数

表示：

> 从当前节点到目标节点还需要多少代价的估计。

注意：

这是：

```text
估计值
```

不一定是真的。

---

## 3. 小白理解

假设你开车去一个城市。

导航显示：

```text
B 距终点直线距离 10 km
C 距终点直线距离 30 km
```

Greedy 会想：

> B 看起来更接近目标，先走 B。

它不太关心之前已经走了多少路。

---

## 4. Greedy 的评分函数

```text
Score(n) = h(n)
```

谁的 `h(n)` 最小：

```text
先扩展谁
```

---

## 5. C++ 基本结构

```cpp
priority_queue<
    pair<int, int>,
    vector<pair<int, int>>,
    greater<pair<int, int>>
> pq;
```

保存：

```text
(h值, node)
```

---

## 6. 核心模板

```cpp
pq.push({heuristic[start], start});

while (!pq.empty())
{
    auto [h, current] = pq.top();
    pq.pop();

    if (visited[current])
        continue;

    visited[current] = true;

    if (current == goal)
        break;

    for (auto neighbor : graph[current])
    {
        if (!visited[neighbor])
        {
            pq.push({
                heuristic[neighbor],
                neighbor
            });
        }
    }
}
```

---

## 7. 优点

Greedy 通常：

```text
搜索速度快
```

因为它很“目标导向”。

---

## 8. 缺点

它可能被错误的启发函数欺骗。

因为它只看：

```text
h(n)
```

不关心：

```text
g(n)
```

所以：

> Greedy 不保证一定得到最短路径。

---

## 9. 一句话记忆

Greedy：

> 看谁离终点“看起来最近”，就先走谁。
