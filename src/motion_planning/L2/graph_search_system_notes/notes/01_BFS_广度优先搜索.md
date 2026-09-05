# 01 BFS：广度优先搜索

## 1. 全称

BFS：

```text
Breadth-First Search
```

中文：

> 广度优先搜索

---

## 2. 小白理解

BFS 就像水波扩散：

```text
第一圈
第二圈
第三圈
...
```

它先把离起点近的节点访问完，再去更远的一层。

例如：

```text
        A
      /   \
     B     C
    / \     \
   D   E     F
```

从 A 开始：

```text
A
↓
B C
↓
D E F
```

可能的访问顺序：

```text
A → B → C → D → E → F
```

---

## 3. 使用的数据结构

BFS 使用：

```cpp
queue
```

队列特点：

```text
FIFO
First In First Out
先进先出
```

---

## 4. 基本流程

```text
1. 起点入队
2. 标记起点已经发现
3. 取出队头
4. 访问它
5. 把未访问邻居放到队尾
6. 重复
```

---

## 5. C++ 模板

```cpp
queue<int> q;
vector<bool> visited(n, false);

q.push(start);
visited[start] = true;

while (!q.empty())
{
    int current = q.front();
    q.pop();

    cout << current << " ";

    for (int neighbor : graph[current])
    {
        if (!visited[neighbor])
        {
            visited[neighbor] = true;
            q.push(neighbor);
        }
    }
}
```

---

## 6. BFS 为什么能找无权图最短路

因为 BFS 是：

```text
距离 0
→ 距离 1
→ 距离 2
→ 距离 3
```

一层一层访问。

因此第一次到达一个节点时，走过的边数就是最少的。

注意：

> 这里说的是无权图，或者所有边权相同。

---

## 7. 时间复杂度

邻接表情况下：

```text
O(V + E)
```

其中：

- `V` = 节点数
- `E` = 边数

---

## 8. 常见用途

- 无权图最短路
- 迷宫最少步数
- 社交网络层级
- 二叉树层序遍历
- 状态空间搜索
