# 03 Uniform Cost Search：一致代价搜索

## 1. 什么是 UCS

全称：

```text
Uniform Cost Search
```

中文常译：

> 一致代价搜索 / 均匀代价搜索

它每次选择：

```text
g(n) 最小
```

的节点。

---

## 2. 什么是 g(n)

```text
g(n)
```

表示：

> 从起点走到当前节点 n 的实际总代价。

例如：

```text
A --2--> B --3--> D
```

那么：

```text
g(B) = 2
g(D) = 2 + 3 = 5
```

---

## 3. 小白理解

想象你在打车。

不同道路价格不同：

```text
A → B：2 元
A → C：8 元
B → D：3 元
```

UCS 不关心“走了几步”。

它关心：

> 到目前为止总共花了多少钱。

所以它总是优先扩展：

```text
累计费用最低的节点
```

---

## 4. 为什么不能用普通 queue

因为我们不是按照进入顺序取节点。

而是按照：

```text
g(n)
```

最小取节点。

所以通常使用：

```cpp
priority_queue
```

---

## 5. 基本结构

```text
Priority Queue:

(g值, node)
```

例如：

```text
(2, B)
(5, D)
(8, C)
```

首先取：

```text
B
```

因为 `g(B)=2` 最小。

---

## 6. 和 Dijkstra 的关系

在常见图最短路场景中：

> UCS 的思想和 Dijkstra 基本一致。

它们都是：

```text
优先扩展当前累计路径代价最小的节点
```

---

## 7. C++ 核心模板

```cpp
priority_queue<
    pair<int, int>,
    vector<pair<int, int>>,
    greater<pair<int, int>>
> pq;

vector<int> dist(n, INF);

dist[start] = 0;

pq.push({0, start});

while (!pq.empty())
{
    auto [cost, current] = pq.top();
    pq.pop();

    if (cost != dist[current])
        continue;

    for (auto [neighbor, weight] : graph[current])
    {
        int newCost = cost + weight;

        if (newCost < dist[neighbor])
        {
            dist[neighbor] = newCost;

            pq.push({newCost, neighbor});
        }
    }
}
```

---

## 8. UCS 的特点

如果所有边权：

```text
>= 0
```

那么 UCS 能找到最低代价路径。

---

## 9. BFS 和 UCS 的区别

BFS：

```text
比较走了多少条边
```

UCS：

```text
比较总代价
```

例如：

```text
路径1：两条边，总费用100
路径2：五条边，总费用10
```

BFS 更喜欢路径1。

UCS 更喜欢路径2。
