// 四个算法的注释版实现。结构几乎一样，差别只在「每次从容器里取谁」。
//
//   BFS      FIFO 队列        步数最少
//   DFS      LIFO 栈          无最优保证
//   Dijkstra 小顶堆按 g       代价最小
//   A*       小顶堆按 g+h     代价最小，通常扩得更少
//
#include "grid_search_practice/search.h"

#include <queue>
#include <utility>
#include <vector>

namespace grid_search_practice {

namespace {

bool badEndpoint(const GridMap2D& map, Cell start, Cell goal) {
  return !map.inside(start.x, start.y) || !map.inside(goal.x, goal.y) ||
         map.isOccupied(start.x, start.y) || map.isOccupied(goal.x, goal.y);
}

// 拍一张 OPEN/CLOSED 快照给 RViz：
//   closed[id]==1               → 已经取出处理过（蓝）
//   discovered[id] && !closed   → 还在 frontier 里（绿）
SearchStep makeSnapshot(const GridMap2D& map, int cur, const std::vector<double>& g,
                        const std::vector<int>& depth, const std::vector<char>& closed,
                        const std::vector<char>& discovered) {
  SearchStep st;
  int cx = 0;
  int cy = 0;
  map.toXY(cur, &cx, &cy);
  st.current = {cx, cy};
  st.current_g = g[static_cast<std::size_t>(cur)];
  st.current_depth = depth[static_cast<std::size_t>(cur)];
  const int n = map.numCells();
  for (int id = 0; id < n; ++id) {
    int x = 0;
    int y = 0;
    map.toXY(id, &x, &y);
    if (closed[static_cast<std::size_t>(id)]) {
      st.closed.push_back({x, y});
    } else if (discovered[static_cast<std::size_t>(id)]) {
      st.open.push_back({x, y});
    }
  }
  return st;
}

void finishSuccess(SearchOutput* out, const GridMap2D& map, const std::vector<int>& parent, int sid,
                   int gid) {
  out->success = true;
  out->path = reconstructPath(parent, map, sid, gid);
  out->path_cost = pathCost(map, out->path);
  out->path_steps = out->path.empty() ? 0 : static_cast<int>(out->path.size()) - 1;
}

}  // namespace

SearchOutput bfs(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt) {
  SearchOutput out;

  // =====================================================================
  // 1. 输入检查：起/终点必须在地图内、必须空闲
  // =====================================================================
  if (badEndpoint(map, start, goal)) return out;

  // =====================================================================
  // 2. 二维格子压成一维 id：id = y * width + x
  //    队列、parent、visited 全部按下标操作。
  // =====================================================================
  const int n = map.numCells();
  const int sid = map.toId(start.x, start.y);
  const int gid = map.toId(goal.x, goal.y);

  // =====================================================================
  // 3. 数据结构（BFS 的关键：FIFO 队列 + 入队时就标记 vis）
  //
  //    q        待处理集合。队头 = 最早入队的 = 离起点步数最少的那一层
  //    vis      入队时就标 1。每个格子只进一次
  //    closed   出队并处理完才标 1。RViz 蓝色
  //    parent   从哪一格走过来的，最后沿它从终点回溯到起点
  //    g        几何代价。4 邻域下每条边都是 1，所以 g == depth
  //    depth    走了几步。BFS 保证这个量最小
  // =====================================================================
  std::vector<char> vis(n, 0);
  std::vector<char> closed(n, 0);
  std::vector<int> parent(n, -1);
  std::vector<double> g(n, kInf);
  std::vector<int> depth(n, -1);
  std::queue<int> q;

  q.push(sid);
  vis[sid] = 1;
  g[sid] = 0;
  depth[sid] = 0;
  out.pushes = 1;

  // =====================================================================
  // 4. 主循环：不断取出「最早入队的格子」，一层一层往外推
  //    空地图 4 邻域下，CLOSED 会铺成菱形（曼哈顿等距面）
  // =====================================================================
  while (!q.empty()) {
    // 4.1 FIFO：从队头取。这是 BFS 和另外三个算法唯一的本质差别。
    const int cur = q.front();
    q.pop();
    closed[cur] = 1;
    ++out.expanded;

    SearchStep st = makeSnapshot(map, cur, g, depth, closed, vis);

    // 4.2 队头第一次碰到终点：一定是步数最少的
    if (cur == gid) {
      out.steps.push_back(std::move(st));
      finishSuccess(&out, map, parent, sid, gid);
      return out;
    }

    // 4.3 扩展邻居。neighbors() 已经丢掉越界、障碍、斜穿墙角。
    int cx = 0;
    int cy = 0;
    map.toXY(cur, &cx, &cy);
    for (const Cell& nb : neighbors(map, cx, cy, opt)) {
      const int nid = map.toId(nb.x, nb.y);
      st.neighbors.push_back(nb);

      // 已经进过队就跳过。BFS 不松弛：先到的那条一定步数最少。
      if (vis[nid]) continue;

      vis[nid] = 1;
      parent[nid] = cur;
      g[nid] = g[cur] + edgeCost(cx, cy, nb.x, nb.y);
      depth[nid] = depth[cur] + 1;
      q.push(nid);
      ++out.pushes;
    }
    out.steps.push_back(std::move(st));
  }
  return out;
}

SearchOutput dfs(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt) {
  SearchOutput out;
  if (badEndpoint(map, start, goal)) return out;

  const int n = map.numCells();
  const int sid = map.toId(start.x, start.y);
  const int gid = map.toId(goal.x, goal.y);

  // =====================================================================
  // DFS 和 BFS 的差别只有「容器从队列换成栈」。
  //
  //   BFS 入队时标 vis，每个格子只进一次
  //   DFS 弹出时才标 closed，同一格可以压栈多次（每次父亲可能不同）
  //       真正认哪个爹：看它**第一次被弹出来**的那条条目
  //
  // 所以条目必须自带 parent/g/depth，不能只存一个 id。
  // =====================================================================
  struct Item {
    int id = -1;
    int parent = -1;
    double g = 0.0;
    int depth = 0;
  };

  std::vector<char> discovered(n, 0);
  std::vector<char> closed(n, 0);
  std::vector<int> parent(n, -1);
  std::vector<double> g(n, kInf);
  std::vector<int> depth(n, -1);
  std::vector<Item> st;  // LIFO：push_back / pop_back

  st.push_back({sid, -1, 0.0, 0});
  discovered[sid] = 1;
  out.pushes = 1;

  while (!st.empty()) {
    // 5.1 LIFO：栈顶 = 最晚压进去的 = 沿着一条路往深处扎
    const Item item = st.back();
    st.pop_back();

    // 5.2 懒惰去重：同一格可能被不同父亲压过好几次。
    //     第一次弹出已经 CLOSED，后面那些条目直接丢掉。
    if (closed[item.id]) continue;

    closed[item.id] = 1;
    parent[item.id] = item.parent;
    g[item.id] = item.g;
    depth[item.id] = item.depth;
    ++out.expanded;

    SearchStep snap = makeSnapshot(map, item.id, g, depth, closed, discovered);

    if (item.id == gid) {
      out.steps.push_back(std::move(snap));
      finishSuccess(&out, map, parent, sid, gid);
      return out;
    }

    int cx = 0;
    int cy = 0;
    map.toXY(item.id, &cx, &cy);
    for (const Cell& nb : neighbors(map, cx, cy, opt)) {
      const int nid = map.toId(nb.x, nb.y);
      snap.neighbors.push_back(nb);

      // 已经处理过的不再压。还在栈里的可以再压一份（换爹）。
      if (closed[nid]) continue;

      discovered[nid] = 1;
      st.push_back({nid, item.id, item.g + edgeCost(cx, cy, nb.x, nb.y), item.depth + 1});
      ++out.pushes;
    }
    out.steps.push_back(std::move(snap));
  }
  return out;
}

SearchOutput dijkstra(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt) {
  SearchOutput out;
  if (badEndpoint(map, start, goal)) return out;

  const int n = map.numCells();
  const int sid = map.toId(start.x, start.y);
  const int gid = map.toId(goal.x, goal.y);

  // =====================================================================
  // Dijkstra：每次取 g 最小的格子。
  //
  // 4 邻域边代价全是 1 → 和 BFS 求的是同一个量，路径代价必然相同。
  // 8 邻域斜边 √2 → 「步数少」和「代价小」分家，BFS 可能次优，Dijkstra 仍最优。
  //
  // 堆不能直接改里面某个元素的 g，所以用「懒惰删除」：
  //   松弛时再 push 一条新的 {g_new, id}，旧条目留在堆里。
  //   弹出后若该格已 CLOSED，或弹出的 g 比当前 g[] 差，丢掉即可。
  // =====================================================================
  std::vector<char> discovered(n, 0);
  std::vector<char> closed(n, 0);
  std::vector<int> parent(n, -1);
  std::vector<double> g(n, kInf);
  std::vector<int> depth(n, -1);

  using Item = std::pair<double, int>;  // {g, id}
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  g[sid] = 0;
  depth[sid] = 0;
  discovered[sid] = 1;
  open.push({0.0, sid});
  out.pushes = 1;

  while (!open.empty()) {
    const int cur = open.top().second;
    const double cg = open.top().first;
    open.pop();

    if (closed[cur]) continue;
    if (cg > g[cur] + 1e-12) continue;  // 堆里的旧票

    closed[cur] = 1;
    ++out.expanded;
    SearchStep snap = makeSnapshot(map, cur, g, depth, closed, discovered);

    // 第一次 CLOSED 到终点时，g 就是最短代价
    if (cur == gid) {
      out.steps.push_back(std::move(snap));
      finishSuccess(&out, map, parent, sid, gid);
      return out;
    }

    int cx = 0;
    int cy = 0;
    map.toXY(cur, &cx, &cy);
    for (const Cell& nb : neighbors(map, cx, cy, opt)) {
      const int nid = map.toId(nb.x, nb.y);
      snap.neighbors.push_back(nb);
      if (closed[nid]) continue;

      // 松弛：走 cur→nb 是否比 nb 目前记下的路更短
      const double g_new = g[cur] + edgeCost(cx, cy, nb.x, nb.y);
      if (g_new + 1e-12 < g[nid]) {
        g[nid] = g_new;
        parent[nid] = cur;
        depth[nid] = depth[cur] + 1;
        discovered[nid] = 1;
        open.push({g_new, nid});
        ++out.pushes;
      }
    }
    out.steps.push_back(std::move(snap));
  }
  return out;
}

SearchOutput astar(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt) {
  SearchOutput out;
  if (badEndpoint(map, start, goal)) return out;

  const int n = map.numCells();
  const int sid = map.toId(start.x, start.y);
  const int gid = map.toId(goal.x, goal.y);

  // =====================================================================
  // A* = Dijkstra，取出规则从 g 换成 f = g + h
  //
  // h 是到终点的估计，本包用 heuristic()：
  //   4 邻域：曼哈顿 |dx|+|dy|
  //   8 邻域：octile
  // 这两个 h 都不超过真实最短路（可采纳），所以 A* 仍然代价最优，
  // 只是优先朝终点方向扩，CLOSED 会少很多。
  //
  // h ≡ 0 时 f = g，A* 退化为 Dijkstra。
  // 和 Dijkstra 相比，真正不同的只有入堆那一行：push(g+h) 而不是 push(g)。
  // =====================================================================
  auto h_of = [&](int id) {
    int x = 0;
    int y = 0;
    map.toXY(id, &x, &y);
    return heuristic(x, y, goal.x, goal.y, opt.use_8_connected);
  };

  std::vector<char> discovered(n, 0);
  std::vector<char> closed(n, 0);
  std::vector<int> parent(n, -1);
  std::vector<double> g(n, kInf);
  std::vector<int> depth(n, -1);

  using Item = std::pair<double, int>;  // {f, id}
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  g[sid] = 0;
  depth[sid] = 0;
  discovered[sid] = 1;
  open.push({h_of(sid), sid});  // 起点 f = 0 + h(start)
  out.pushes = 1;

  while (!open.empty()) {
    const int cur = open.top().second;
    const double cf = open.top().first;
    open.pop();

    if (closed[cur]) continue;
    if (cf > g[cur] + h_of(cur) + 1e-12) continue;  // 过期 f

    closed[cur] = 1;
    ++out.expanded;
    SearchStep snap = makeSnapshot(map, cur, g, depth, closed, discovered);

    if (cur == gid) {
      out.steps.push_back(std::move(snap));
      finishSuccess(&out, map, parent, sid, gid);
      return out;
    }

    int cx = 0;
    int cy = 0;
    map.toXY(cur, &cx, &cy);
    for (const Cell& nb : neighbors(map, cx, cy, opt)) {
      const int nid = map.toId(nb.x, nb.y);
      snap.neighbors.push_back(nb);
      if (closed[nid]) continue;

      const double g_new = g[cur] + edgeCost(cx, cy, nb.x, nb.y);
      if (g_new + 1e-12 < g[nid]) {
        g[nid] = g_new;
        parent[nid] = cur;
        depth[nid] = depth[cur] + 1;
        discovered[nid] = 1;
        open.push({g_new + h_of(nid), nid});  // 唯一和 Dijkstra 不同的一行
        ++out.pushes;
      }
    }
    out.steps.push_back(std::move(snap));
  }
  return out;
}

}  // namespace grid_search_practice
