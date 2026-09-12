# A* 算法实现说明

## 文件位置
`src/hw_2/src/grid_path_searcher/src/Astar_searcher.cpp`

## 算法步骤详解

### STEP 1: 启发式函数 (getHeu) - 第124行

**位置**: `Astar_searcher.cpp:124-143`

**实现**: 欧几里得距离 + tie-breaker

```cpp
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
    // 使用欧几里得距离作为启发式函数
    double dx = node2->coord(0) - node1->coord(0);
    double dy = node2->coord(1) - node1->coord(1);
    double dz = node2->coord(2) - node1->coord(2);

    // 欧几里得距离
    double h = sqrt(dx * dx + dy * dy + dz * dz);

    // tie-breaker: 避免相同 f 值时的随机选择
    // 让启发式函数略微倾向于朝向目标的路径
    h = h * (1.0 + 1e-3);

    return h;
}
```

**说明**:
- **欧几里得距离**: 3D 空间中两点间的直线距离，满足 A* 的一致性要求
- **Tie-breaker**: 通过 `1.0 + 1e-3` 的系数，使得启发式函数略微高估，优先选择朝向目标的路径
- 其他可选启发式函数:
  - **曼哈顿距离**: `|dx| + |dy| + |dz|` (欠估计，搜索更多节点)
  - **对角线距离**: 考虑对角移动的代价
  - **Dijkstra**: `h = 0` (无启发，等价于 Dijkstra 算法)

---

### STEP 2: 初始化准备 - 第175行

**位置**: `Astar_searcher.cpp:175-182`

**实现**:
```cpp
// STEP 2: preparatory works before while loop
GridNodeMap[goalIdx(0)][goalIdx(1)][goalIdx(2)]->id = 0;
vector<GridNodePtr> neighborPtrSets;
vector<double> edgeCostSets;
```

**说明**:
- 将终点节点的 `id` 设置为 `0`（未访问状态）
- 初始化邻居节点和边代价的容器
- 起点已经在之前设置为 `id = 1`（在 open set 中）

---

### STEP 3: 从 Open Set 取出最小 f 值节点 - 第189行

**位置**: `Astar_searcher.cpp:185-197`

**实现**:
```cpp
// 从 open set 中取出 f 值最小的节点
currentPtr = openSet.begin()->second;
currentPtr->id = -1;  // 标记为 closed set
openSet.erase(openSet.begin());
```

**说明**:
- `openSet` 是 `multimap<double, GridNodePtr>` 类型，按 f 值自动排序
- `openSet.begin()` 返回 f 值最小的节点
- 取出后标记为 `id = -1`（closed set），表示已扩展

---

### STEP 4: 获取邻居节点 (AstarGetSucc) - 第112行

**位置**: `Astar_searcher.cpp:112-153`

**实现**: 26 方向邻居搜索（3D 空间）

```cpp
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, 
                                          vector<GridNodePtr>& neighborPtrSets, 
                                          vector<double>& edgeCostSets) {
    neighborPtrSets.clear();
    edgeCostSets.clear();

    Vector3i current_idx = currentPtr->index;

    // 26个方向：遍历 3x3x3 立方体（排除中心点）
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                // 跳过当前节点
                if (dx == 0 && dy == 0 && dz == 0) continue;

                Vector3i neighbor_idx = current_idx + Vector3i(dx, dy, dz);

                // 检查是否越界
                if (neighbor_idx(0) < 0 || neighbor_idx(0) >= GLX_SIZE ||
                    neighbor_idx(1) < 0 || neighbor_idx(1) >= GLY_SIZE ||
                    neighbor_idx(2) < 0 || neighbor_idx(2) >= GLZ_SIZE) {
                    continue;
                }

                // 检查是否为障碍物
                if (isOccupied(neighbor_idx)) {
                    continue;
                }

                // 添加邻居节点
                GridNodePtr neighbor = GridNodeMap[neighbor_idx(0)][neighbor_idx(1)][neighbor_idx(2)];
                neighborPtrSets.push_back(neighbor);

                // 计算边的代价（欧几里得距离）
                double edge_cost = sqrt(dx * dx + dy * dy + dz * dz) * resolution;
                edgeCostSets.push_back(edge_cost);
            }
        }
    }
}
```

**说明**:
- **26 方向**: 包括 6 个面邻居 + 12 条边邻居 + 8 个顶点邻居
- **边界检查**: 防止索引越界
- **障碍物检查**: 使用 `isOccupied()` 判断
- **边代价**: 使用欧几里得距离 `sqrt(dx² + dy² + dz²) * resolution`

---

### STEP 5-7: 处理邻居节点 - 第214行

**位置**: `Astar_searcher.cpp:214-275`

#### STEP 6: 处理新节点 (id = 0)

```cpp
if (neighborPtr->id == 0) {  // 新节点
    // 设置 g 值和 f 值
    neighborPtr->gScore = tentative_gScore;
    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);

    // 设置父节点
    neighborPtr->cameFrom = currentPtr;

    // 加入 open set
    neighborPtr->id = 1;
    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
}
```

**说明**:
- 计算 `gScore = currentPtr->gScore + edgeCost`
- 计算 `fScore = gScore + h(neighbor, goal)`
- 记录父节点用于路径回溯
- 插入 open set 并保存迭代器

#### STEP 7: 更新 Open Set 中的节点 (id = 1)

```cpp
else if (neighborPtr->id == 1) {  // 节点在 open set 中
    // 如果找到更短的路径，则更新
    if (tentative_gScore < neighborPtr->gScore) {
        // 更新 g 值和 f 值
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);

        // 更新父节点
        neighborPtr->cameFrom = currentPtr;

        // 从 open set 中删除旧的条目，插入新的条目
        openSet.erase(neighborPtr->nodeMapIt);
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
    }
}
```

**说明**:
- 比较新路径的 gScore 和原来的 gScore
- 如果新路径更短，更新节点信息
- **重要**: 必须从 openSet 中删除旧条目再插入新条目，因为 f 值改变了

#### Closed Set 中的节点 (id = -1)

```cpp
else {  // 节点在 closed set 中
    continue;  // 跳过
}
```

**说明**: 已经扩展过的节点不再处理

---

### STEP 8: 路径回溯 (getPath) - 第265行

**位置**: `Astar_searcher.cpp:260-283`

**实现**:
```cpp
vector<Vector3d> AstarPathFinder::getPath() {
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    // 从终点回溯到起点
    GridNodePtr currentNode = terminatePtr;
    while (currentNode != NULL) {
        gridPath.push_back(currentNode);
        currentNode = currentNode->cameFrom;
    }

    // 转换为坐标并反转
    for (auto ptr : gridPath) 
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());

    return path;
}
```

**说明**:
- 从 `terminatePtr`（目标节点）开始
- 沿着 `cameFrom` 指针回溯到起点
- 反转路径得到从起点到终点的顺序

---

## GridNode 数据结构

```cpp
struct GridNode {
    int id;                    // 0=未访问, 1=open set, -1=closed set
    Eigen::Vector3d coord;     // 世界坐标
    Eigen::Vector3i index;     // 栅格索引
    Eigen::Vector3i dir;       // 扩展方向
    
    double gScore, fScore;     // g(n) 和 f(n) = g(n) + h(n)
    GridNodePtr cameFrom;      // 父节点指针（用于路径回溯）
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;  // 在 openSet 中的迭代器
};
```

---

## A* 算法核心思想

### 核心公式
```
f(n) = g(n) + h(n)
```
- `g(n)`: 从起点到节点 n 的实际代价
- `h(n)`: 从节点 n 到终点的启发式估计代价
- `f(n)`: 总估计代价

### 算法流程
1. 初始化：起点加入 open set
2. 循环直到 open set 为空或找到目标：
   - 从 open set 取出 f 值最小的节点
   - 如果是目标节点，回溯路径并返回
   - 扩展邻居节点：
     - 新节点：计算代价，加入 open set
     - Open set 中的节点：如果找到更短路径则更新
     - Closed set 中的节点：跳过
   - 将当前节点加入 closed set

### 时间复杂度
- 最坏情况: O(b^d)，b 为分支因子，d 为深度
- 使用二叉堆优化 open set: O(E log V)

---

## 编译和测试

```bash
# 在容器中
cd /workspace
catkin build grid_path_searcher
source devel/setup.bash
roslaunch grid_path_searcher demo.launch
```

在 rviz 中使用 **Goal3DTool** 点击目标点，观察 A* 算法的路径规划结果。

---

## 调试技巧

1. **查看访问的节点数**:
   - 在代码中打印 `openSet.size()` 和访问的节点总数
   - 使用 `/demo_node/visited_nodes_vis` 可视化已访问节点

2. **检查启发式函数**:
   - 确保 h(n) 是可采纳的（admissible）：不高估实际代价
   - 确保 h(n) 是一致的（consistent）：满足三角不等式

3. **路径代价验证**:
   - 终端会打印路径代价和搜索时间
   - 格式: `[A*]{success} Time in A* is X ms, path cost if Y m`

---

## 扩展阅读

- **JPS (Jump Point Search)**: 在栅格地图上优化的 A* 变体
- **Theta***: 支持任意角度路径的 A* 扩展
- **D* Lite**: 支持动态环境的增量式路径规划
