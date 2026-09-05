# 02 DFS：深度优先搜索

## 1. 全称

```text
Depth-First Search
```

中文：

> 深度优先搜索

---

## 2. 小白理解

DFS 的思想是：

> 先沿着一条路一直走到底，走不通了再回来。

例如：

```text
        A
      /   \
     B     C
    / \
   D   E
```

可能顺序：

```text
A → B → D → E → C
```

---

## 3. 使用的数据结构

DFS 可以使用：

```cpp
stack
```

栈的特点：

```text
LIFO
Last In First Out
后进先出
```

也可以直接用递归。

---

## 4. 非递归模板

```cpp
stack<int> st;
vector<bool> visited(n, false);

st.push(start);

while (!st.empty())
{
    int current = st.top();
    st.pop();

    if (visited[current])
        continue;

    visited[current] = true;

    cout << current << " ";

    for (int neighbor : graph[current])
    {
        if (!visited[neighbor])
        {
            st.push(neighbor);
        }
    }
}
```

---

## 5. 递归版本

```cpp
void dfs(int current)
{
    visited[current] = true;

    cout << current << " ";

    for (int neighbor : graph[current])
    {
        if (!visited[neighbor])
        {
            dfs(neighbor);
        }
    }
}
```

---

## 6. BFS vs DFS

### BFS

```text
一层一层
```

### DFS

```text
一条路走到底
```

---

## 7. DFS 能保证最短路吗？

一般不能。

DFS 可能第一条路走得特别远，虽然目标其实就在旁边。

---

## 8. 时间复杂度

```text
O(V + E)
```

---

## 9. 常见用途

- 判断图是否连通
- 找连通分量
- 环检测
- 拓扑相关问题
- 回溯
- 迷宫搜索
- 树遍历
