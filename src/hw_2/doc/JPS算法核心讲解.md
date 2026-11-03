# JPS (Jump Point Search) 算法核心讲解

## 算法概述

JPS 是 A* 算法在**均匀代价栅格地图**上的优化版本。它通过**跳跃**而不是逐格扩展来大幅减少搜索节点数量，同时保证找到最优路径。

**核心思想**: 在对称的栅格地图中，很多节点的扩展是冗余的。JPS 通过识别"跳点"（Jump Points）来跳过这些对称路径，只扩展关键节点。

---

## 核心函数详解

### 1. JPSGetSucc - 获取跳点邻居

**位置**: `JPS_searcher.cpp:6-56`

```cpp
inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, 
                                      vector<GridNodePtr> & neighborPtrSets, 
                                      vector<double> & edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    
    // norm1 表示移动的维度：1=直线，2=对角线，3=三维对角
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));
    
    // 获取邻居数量和强制邻居数量
    int num_neib  = jn3d->nsz[norm1][0];  // 自然邻居数量
    int num_fneib = jn3d->nsz[norm1][1];  // 强制邻居数量
    
    // 根据移动方向计算索引 (3x3x3 = 27种方向)
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);
    
    // 遍历所有可能的邻居方向
    for(int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Vector3i neighborIdx;
        Vector3i expandDir;
        
        if(dev < num_neib) {
            // 处理自然邻居：沿当前方向继续跳跃
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            // 核心：调用 jump 函数找到跳点
            if(!jump(currentPtr->index, expandDir, neighborIdx))  
                continue;
        }
        else {
            // 处理强制邻居：检查是否有障碍物产生强制邻居
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            // 如果相邻位置有障碍物，沿强制方向跳跃
            if(isOccupied(nx, ny, nz)) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if(!jump(currentPtr->index, expandDir, neighborIdx)) 
                    continue;
            }
            else
                continue;
        }
        
        // 将找到的跳点加入邻居列表
        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;  // 记录扩展方向
        
        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))
        ));
    }
}
```

**关键点**：
- **norm1**: 移动类型
  - `1`: 直线移动（沿 x/y/z 轴）
  - `2`: 二维对角移动（沿 xy/xz/yz 平面）
  - `3`: 三维对角移动（沿 xyz 体对角线）
- **自然邻居**: 沿当前移动方向的延续
- **强制邻居**: 因障碍物而必须考虑的方向

---

### 2. jump - 跳跃函数（JPS 核心）

**位置**: `JPS_searcher.cpp:58-83`

```cpp
bool JPSPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    // 沿方向移动一步
    neiIdx = curIdx + expDir;
    
    // 终止条件1：遇到障碍物
    if(!isFree(neiIdx))
        return false;
    
    // 终止条件2：到达目标点（找到跳点）
    if(neiIdx == goalIdx)
        return true;
    
    // 终止条件3：发现强制邻居（找到跳点）
    if(hasForced(neiIdx, expDir))
        return true;
    
    // 对角移动时，需要先在各个轴向上递归搜索跳点
    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];
    
    // 对角移动时，检查所有主方向
    for(int k = 0; k < num_neib - 1; ++k) {
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        
        // 如果在主方向上找到跳点，当前位置也是跳点
        if(jump(neiIdx, newDir, newNeiIdx)) 
            return true;
    }
    
    // 继续沿当前方向跳跃（递归）
    return jump(neiIdx, expDir, neiIdx);
}
```

**跳跃逻辑**：

1. **直线跳跃**：
   ```
   当前 → → → 跳点
   ```
   沿直线方向一直跳，直到：
   - 遇到障碍物（失败）
   - 到达目标（成功）
   - 发现强制邻居（成功）

2. **对角跳跃**：
   ```
   当前 ↗ ↗ ↗ 跳点
      ↑ ↑ ↑
      → → →
   ```
   对角移动时，需要先在水平/垂直方向上搜索跳点

**递归终止条件**：
- `!isFree(neiIdx)`: 碰到障碍物
- `neiIdx == goalIdx`: 找到目标
- `hasForced(neiIdx, expDir)`: 发现强制邻居

---

### 3. hasForced - 检测强制邻居

**位置**: `JPS_searcher.cpp:85-127`

```cpp
inline bool JPSPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);
    
    switch(norm1) {
        case 1:
            // 1维移动（直线），检查8个邻居
            for(int fn = 0; fn < 8; ++fn) {
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if(isOccupied(nx, ny, nz))
                    return true;  // 发现障碍物，产生强制邻居
            }
            return false;
        
        case 2:
            // 2维移动（对角线），检查8个邻居
            // ... 类似逻辑
            
        case 3:
            // 3维移动（体对角线），检查6个邻居
            // ... 类似逻辑
    }
}
```

**强制邻居的概念**：

当移动路径附近有障碍物时，会产生"强制邻居"。

**示例**（2D 情况）：
```
. . .     . ▓ .
. X →  →  X ↗ ?
. . .     . . .
```
- `X` 是当前位置，向右移动
- `▓` 是障碍物
- `?` 成为强制邻居（因为绕过障碍物的最短路径必须经过这里）

**3D 扩展**：
- **直线移动** (norm1=1): 检查垂直于移动方向的 8 个邻居
- **平面对角** (norm1=2): 检查 8 个可能产生强制邻居的位置
- **空间对角** (norm1=3): 检查 6 个面邻居

---

## JPS vs A* 对比

| 特性 | A* | JPS |
|------|-----|-----|
| **扩展节点** | 逐格扩展所有邻居 | 只扩展跳点 |
| **搜索空间** | 大（所有可达节点） | 小（只有跳点） |
| **路径质量** | 最优 | 最优（等价于 A*） |
| **适用场景** | 通用图搜索 | 均匀代价栅格地图 |
| **时间复杂度** | O(b^d) | O(b'^d)，b' << b |

**性能提升**：
- 在开阔区域，JPS 可以减少 **80-90%** 的节点扩展
- 在复杂地图中，提升约 **2-10 倍**

---

## JPS 算法主流程

```cpp
void JPSPathFinder::JPSGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    // 1. 初始化起点和终点
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx, end_pt);
    
    // 2. 起点加入 open set
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->id = 1;
    openSet.insert(make_pair(startPtr->fScore, startPtr));
    
    // 3. 主循环（与 A* 类似）
    while(!openSet.empty()) {
        // 取出 f 值最小的节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1;
        openSet.erase(openSet.begin());
        
        // 检查是否到达目标
        if(currentPtr->index == goalIdx) {
            terminatePtr = currentPtr;
            return;
        }
        
        // 关键不同：获取跳点邻居而不是所有邻居
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
        
        // 处理邻居（与 A* 相同）
        for(int i = 0; i < neighborPtrSets.size(); i++) {
            neighborPtr = neighborPtrSets[i];
            double tentative_gScore = currentPtr->gScore + edgeCostSets[i];
            
            if(neighborPtr->id != 1) {  // 新节点
                // 加入 open set
            }
            else if(tentative_gScore <= neighborPtr->gScore) {  // 更新
                // 更新节点信息
                // 特别注意：更新扩展方向
                for(int i = 0; i < 3; i++) {
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if(neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs(neighborPtr->dir(i));
                }
            }
        }
    }
}
```

---

## JPS 优化原理图解

### 普通 A* 扩展

```
起点 S → → → → → → → 终点 G
     ↓ ↓ ↓ ↓ ↓ ↓ ↓
     ↓ ↓ ↓ ↓ ↓ ↓ ↓
```
需要扩展所有路径上的节点

### JPS 扩展

```
起点 S ======跳点====== 终点 G
          (跳过中间)
```
直接跳到关键节点（跳点）

---

## 跳点类型

1. **目标点**: 总是跳点
2. **强制邻居点**: 障碍物附近产生的跳点
3. **对角转折点**: 对角移动时在主方向上发现跳点的位置

---

## 实现细节：jn3d 查找表

`jn3d` 是预计算的查找表，存储了：
- `nsz[norm1][0]`: 自然邻居数量
- `nsz[norm1][1]`: 强制邻居数量
- `ns[id]`: 自然邻居方向
- `f1[id]`, `f2[id]`: 强制邻居检测位置和方向

这避免了运行时的复杂计算，提高了效率。

---

## 关键数据结构

```cpp
struct GridNode {
    int id;                // 1=open set, -1=closed set
    Vector3i index;        // 栅格索引
    Vector3i dir;          // 扩展方向（JPS 特有）
    Vector3d coord;        // 世界坐标
    
    double gScore, fScore;
    GridNodePtr cameFrom;
    multimap<...>::iterator nodeMapIt;
};
```

**JPS 特有字段**: `dir` 记录节点的扩展方向，用于：
- 确定自然邻居和强制邻居
- 正确执行跳跃操作

---

## 总结

JPS 的核心优势：
1. **保持最优性**: 找到的路径与 A* 完全相同
2. **大幅减少扩展**: 只扩展跳点，跳过冗余节点
3. **无需预处理**: 不需要构建额外的数据结构
4. **适用场景**: 均匀代价的栅格地图

JPS 的三个核心函数：
- `JPSGetSucc`: 获取跳点邻居
- `jump`: 递归跳跃直到找到跳点
- `hasForced`: 检测强制邻居

JPS 是 A* 在特定场景下的智能优化，通过"跳跃"减少搜索空间，在保证最优性的同时显著提升性能。
