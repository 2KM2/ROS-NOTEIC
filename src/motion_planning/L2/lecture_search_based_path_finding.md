# Lecture 2: Search-Based Path Finding

> 深蓝学院 - 机器人学课程 Lecture 2  
> 主讲人：Fei Gao  
> 课程：Search-Based Path Finding

## 目录

1. 配置空间与 C-Obstacle
2. 图搜索基础
3. Dijkstra 与 A*
4. Jump Point Search (JPS)
5. Homework

---

## 1. 配置空间与 C-Obstacle

### 1.1 配置空间 (C-space)

**机器人配置 (Configuration)**  
- 机器人所有点的位置集合  
- 常用表示：位姿 (pose) = (x, y, θ)

**自由度 (DOF)**  
- 描述机器人姿态所需的最小实数值坐标数  
- 2D 平面机器人：2 DOF (x, y)  
- 带旋转：3 DOF (x, y, θ)

**配置空间 (C-space)**  
- 所有可能配置的 n 维空间  
- 记作 **C-space**  
- 每个位姿对应 C-space 上的一个**点**

### 1.2 碰撞检测与 C-Obstacle

**工作空间障碍**  
- 真实世界中物体的形状与大小

**配置空间障碍 (C-Obstacle)**  
- 将工作空间障碍在配置空间中“膨胀”后得到的区域  
- 机器人被视为一个点后，C-Obstacle 就是机器人的“安全缓冲区”  
- C-space = (C-Obstacle) ∪ C-free

**保守表示**  
- 将机器人膨胀为半径 δ_r 的球体  
- 所有障碍向外膨胀 δ_r 后得到 C-Obstacle  
- 保证即使机器人有实际大小，也不会发生碰撞

---

## 2. 图搜索基础

### 2.1 状态空间图

**图搜索问题**  
- 起点状态 X_start  
- 目标状态 X_goal  
- 状态空间图：节点 + 边

**搜索树**  
- 从起点开始逐层扩展得到的树  
- 路径 = 从根到叶子节点的路径

### 2.2 通用搜索框架

所有图搜索算法的核心框架：

```text
建立 Container
放入起点 X_start

while Container 不为空:
    根据规则取出当前节点 current
    Visit current
    if current == X_goal:
        搜索成功
    
    Expansion:
        找到 current 的所有邻居
    
    Discover:
        找到尚未访问的邻居
    
    Push:
        把这些邻居加入 Container
```

### 2.3 不同算法的选择标准

| 算法 | Container | 选择规则 |
|------|-----------|----------|
| BFS  | Queue     | 先进先出 |
| DFS  | Stack     | 后进先出 |
| UCS  | Priority Queue | 选择 g(n) 最小 |
| Greedy | Priority Queue | 选择 h(n) 最小 |
| A*   | Priority Queue | 选择 f(n) = g(n) + h(n) 最小 |

---

## 3. Dijkstra 与 A*

### 3.1 Dijkstra 算法

**核心思想**  
- 总是扩展 g(n) 最小的节点  
- 保证找到从起点到当前节点的最优代价

**伪代码**

```text
初始化 priority queue
queue.push(start)          // g(start) = 0
g[start] = 0

while queue 不为空:
    current = queue.pop()  // 取出 g 最小的节点
    if current == goal: return path
    
    for neighbor in neighbors[current]:
        new_g = g[current] + cost(current, neighbor)
        if new_g < g[neighbor]:
            g[neighbor] = new_g
            queue.push(neighbor)
```

**特点**  
- ✅ 总是找到最优路径  
- ❌ 所有方向均匀扩展，效率低

### 3.2 A* 算法

**核心公式**

```text
f(n) = g(n) + h(n)
```

- g(n)：从起点到 n 的真实代价  
- h(n)：从 n 到目标的估计代价  
- f(n)：经过 n 的整条路径的估计代价

**算法步骤**

```text
priority queue 初始放入 start (f= h(start))
while queue 不为空:
    current = pop()  // 取出 f 最小的节点
    
    if current == goal:
        return path
    
    for neighbor in neighbors[current]:
        if neighbor 在 closed 中：跳过
        tentative_g = g[current] + cost(...)
        
        if neighbor 不在 open 中 或 tentative_g 更小：
            g[neighbor] = tentative_g
            f[neighbor] = g[neighbor] + h[neighbor]
            open.push(neighbor)
```

### 3.3 可采纳启发函数 (Admissible Heuristic)

**定义**  
h(n) 是**可采纳的** ⇔ 对所有 n：

```text
h(n) ≤ h*(n)
```

其中 h*(n) 是从 n 到目标的**真实**最短代价。

**重要结论**  
- 如果 h 是可采纳的，A* 一定能找到最优路径  
- 启发函数质量越高，A* 效率越高

### 3.4 常用启发函数

| 名称          | 公式                              | 4邻域 可采纳？ | 8邻域 可采纳？ |
|---------------|-----------------------------------|----------------|----------------|
| Zero          | h = 0                              | ✅             | ✅             |
| Manhattan     | h = |dx| + |dy|                   | ✅             | ❌ (高估)      |
| Euclidean     | h = √(dx² + dy²)                  | ✅             | ✅             |
| Diagonal      | h = √2·min(dx,dy) + \|dx-dy\|     | ✅             | ✅             |

**8邻域下 Manhattan 不是可采纳的**  
因为斜走一步真实代价 ≈1.414，而 Manhattan 算 2，高估了。

---

## 4. Jump Point Search (JPS)

### 4.1 问题

A* 在空旷区域会把大量对称路径放入 OPEN，导致节点数量爆炸。

### 4.2 核心思想

**只保留对称性被破坏的节点**  
- 沿一条直线或对角线前进时，中间节点对称  
- 遇到障碍或强迫邻居时停止跳跃  

**跳点 (Jump Point)**  
- 路径上保留的唯一节点  
- 所有中间对称节点被剪枝

### 4.3 跳跃规则 (Jump Rule)

```text
jump(x, y, dx, dy):
    while true:
        // 检查强迫邻居
        if hasForcedNeighbor(x, y, dx, dy):
            return (x, y)  // 当前格是跳点
        
        // 检查对角扫描
        if diagonal_scan(x, y, dx, dy):
            return (x, y)
        
        // 移动一步
        x += dx
        y += dy
        
        if obstacle(x, y):
            return false
```

### 4.4 优点

- 在空旷区域大幅减少 OPEN 节点  
- 保持最优性  
- 适用于均匀代价的网格地图

---

## 5. Homework

### 5.1 作业要求

**Basic**  
- 实现 2D A* 搜索  
- 输入：随机生成的 2D 栅格地图  
- 输出：从起点到目标的最优路径

**Advance**  
- 实现 3D A* (或 Dijkstra)  
- 比较 A* 与 JPS 在不同复杂环境下的表现  
- 可选：实现带动态可采纳性的 JPS

### 5.2 推荐学习顺序

1. `00_图搜索总览.md`
2. `01_BFS_广度优先搜索.md`
3. `02_DFS_深度优先搜索.md`
4. `03_UCS_一致代价搜索.md`
5. `04_Greedy_贪心最佳优先搜索.md`
6. `05_AStar_A星搜索.md`
7. `06_五种算法对比总结.md`

---

**参考资料**  
- [www.redblobgames.com/pathfinding/a-star/introduction.html](https://www.redblobgames.com/pathfinding/a-star/introduction.html)

---

**End of Lecture 2**  
**Thanks for listening!**