// 自检 / 交叉验证 —— 用来回答"凭什么相信这份实现是对的"。
//
// 教学代码最怕的就是"看起来在跑，其实是错的"。这个程序在几百张随机地图上，
// 用**互不相同的手段**互相验证同一批结论：
//
//   1. 路径本身合法吗？        起点/终点对得上、步步相邻、不穿障碍、不斜穿墙角、
//                              一步步累加出来的代价 == 报告的 path_cost
//   2. 步数和父链一致吗？      path_steps == depth[goal]（两个量算法完全不同）
//   3. BFS 真的步数最少吗？    和一个独立写的、只算层数的参考 BFS 逐格比 depth
//   4. Dijkstra 真的最优吗？   和隔壁包的 A*(h≡0) 以及 A*(octile) 比 path_cost
//                              —— 三份独立实现给出同一个数字，才敢说它是最优解
//   5. 三个算法对"有没有解"的判断一致吗？（它们都会遍历完整个连通分量，必须一致）
//   6. 回放器和算法逐位一致吗？ 把 trace 重放到底，state/g/depth/parent 四个数组
//                              必须和算法跑完后的 nodes() 完全相同
//   7. 那些"应该成立的不等式"成立吗？
//                              BFS.steps <= Dijkstra.steps，BFS.cost >= Dijkstra.cost
//                              4 邻域下两者必须完全相等
//   8. BFS 的 stale_pops 恒为 0 吗？（每个格子只入队一次，这是 BFS 的定义性质）
//
// 顺便统计"8 邻域下 BFS 的路径严格比最短路更长"出现的频率 —— 这是
// doc/01-bfs.md 里那个推导的实验证据。
//
// 编译：
//   cd src/search_tutorial
//   g++ -std=c++17 -O2 -I include -I ../astar_tutorial/include
//       ../astar_tutorial/src/grid_map_2d.cpp ../astar_tutorial/src/astar.cpp
//       src/graph_search.cpp src/trace_replayer.cpp src/search_selfcheck.cpp
//       -o /tmp/search_selfcheck            （四行接成一行执行）
//   /tmp/search_selfcheck            # 默认 300 个随机实例
//   /tmp/search_selfcheck 2000       # 跑久一点
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <string>
#include <vector>

#include "astar_tutorial/astar.h"
#include "astar_tutorial/grid_map_2d.h"
#include "search_tutorial/graph_search.h"
#include "search_tutorial/trace_replayer.h"

using namespace search_tutorial;

namespace {

int g_checks = 0;
int g_failures = 0;
std::string g_ctx;  // 当前实例的描述，失败时打出来好复现

void check(bool ok, const std::string& what) {
  ++g_checks;
  if (ok) return;
  ++g_failures;
  std::printf("  [FAIL] %s   <<< %s\n", what.c_str(), g_ctx.c_str());
}

// ---------------------------------------------------------------- 独立参考实现
// 故意写得和 GraphSearch **不一样**：一个只有队列和 depth 数组的裸 BFS，
// 没有 frontier 抽象、没有 trace、没有 g。它算出来的 depth 是"最少步数"的真值。
std::vector<int> referenceBfsDepth(const GridMap2D& map, int start_id, bool conn8,
                                   bool corner_cut) {
  static const int kDX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const int dirs = conn8 ? 8 : 4;

  std::vector<int> depth(map.numCells(), -1);
  std::queue<int> q;
  depth[start_id] = 0;
  q.push(start_id);
  while (!q.empty()) {
    const int id = q.front();
    q.pop();
    int x = 0, y = 0;
    map.toXY(id, &x, &y);
    for (int d = 0; d < dirs; ++d) {
      const int nx = x + kDX[d];
      const int ny = y + kDY[d];
      if (!map.inside(nx, ny) || map.isOccupied(nx, ny)) continue;
      if (kDX[d] != 0 && kDY[d] != 0 && !corner_cut) {
        if (map.isOccupied(x + kDX[d], y) || map.isOccupied(x, y + kDY[d])) continue;
      }
      const int nid = map.toId(nx, ny);
      if (depth[nid] >= 0) continue;
      depth[nid] = depth[id] + 1;
      q.push(nid);
    }
  }
  return depth;
}

// ---------------------------------------------------------------- 路径合法性
// 只看路径本身，完全不管它是哪个算法生成的 —— 所以三个算法能共用这一份检查。
void validatePath(const GridMap2D& map, const GraphSearch& s, int sx, int sy, int gx, int gy,
                  const char* tag) {
  const std::vector<std::pair<int, int>> p = s.pathGrid();
  const SearchStats& st = s.stats();
  const std::string t = std::string(tag) + ": ";

  check(!p.empty(), t + "成功时路径非空");
  if (p.empty()) return;
  check(p.front().first == sx && p.front().second == sy, t + "路径第一格是起点");
  check(p.back().first == gx && p.back().second == gy, t + "路径最后一格是终点");
  check(static_cast<int>(p.size()) == st.path_size, t + "path_size 和路径长度一致");
  check(st.path_steps == static_cast<int>(p.size()) - 1, t + "path_steps == 格子数-1");

  double cost = 0.0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    check(map.isFree(p[i].first, p[i].second), t + "路径上每一格都是空闲的");
    if (i == 0) continue;
    const int dx = p[i].first - p[i - 1].first;
    const int dy = p[i].second - p[i - 1].second;
    const int adx = std::abs(dx);
    const int ady = std::abs(dy);
    check(adx <= 1 && ady <= 1 && (adx + ady) > 0, t + "相邻两格必须真的相邻");
    const bool diagonal = (adx == 1 && ady == 1);
    if (diagonal) {
      check(s.config().use_8_connected, t + "4 邻域下不该出现斜线");
      if (!s.config().allow_corner_cutting) {
        // 禁止斜穿墙角时，两个正交邻居都必须空
        check(map.isFree(p[i - 1].first + dx, p[i - 1].second) &&
                  map.isFree(p[i - 1].first, p[i - 1].second + dy),
              t + "禁止斜穿墙角时，斜线两侧都得是空的");
      }
    }
    cost += diagonal ? kSqrt2 : 1.0;
  }
  // 逐边累加出来的代价，必须等于算法自己报的 path_cost（两条独立的算术路径）
  check(std::fabs(cost - st.path_cost) < 1e-6, t + "逐边累加的代价 == path_cost");
  // 也必须等于终点节点上落定的 g
  check(std::fabs(s.nodes()[map.toId(gx, gy)].g - st.path_cost) < 1e-6,
        t + "g[终点] == path_cost");
  // 步数也有两个独立来源：父链长度 和 depth[终点]
  check(s.nodes()[map.toId(gx, gy)].depth == st.path_steps, t + "depth[终点] == path_steps");
}

// ---------------------------------------------------------------- 回放器一致性
// 把 trace 重放到末尾，四个数组必须和算法跑完后的 nodes() 逐位相同。
// 这一条保证 rviz 里看到的中间过程不是"另一个算法"，而就是这个算法本身。
void validateReplay(const GraphSearch& s, int num_cells, const char* tag) {
  TraceReplayer rp;
  rp.bind(s, num_cells);
  rp.finish();
  const std::string t = std::string(tag) + ": ";

  bool state_ok = true, g_ok = true, depth_ok = true, parent_ok = true;
  for (int id = 0; id < num_cells; ++id) {
    const Node& n = s.nodes()[id];
    if (rp.states()[id] != n.state) state_ok = false;
    if (rp.parents()[id] != n.parent) parent_ok = false;
    if (rp.depths()[id] != n.depth) depth_ok = false;
    const double a = rp.gValues()[id];
    const double b = n.g;
    const bool both_inf = std::isinf(a) && std::isinf(b);
    if (!both_inf && std::fabs(a - b) > 1e-9) g_ok = false;
  }
  check(state_ok, t + "回放到底 state[] 和算法一致");
  check(g_ok, t + "回放到底 g[] 和算法一致");
  check(depth_ok, t + "回放到底 depth[] 和算法一致");
  check(parent_ok, t + "回放到底 parent[] 和算法一致");

  // 中途来回 seek 也不该把状态搞坏（往后退是从 0 重放实现的）
  if (rp.numSteps() >= 4) {
    const int mid = rp.numSteps() / 2;
    rp.seekTo(mid);
    const std::vector<NodeState> snap = rp.states();
    rp.finish();
    rp.seekTo(mid);  // 往回退
    check(rp.states() == snap, t + "seek 到同一步两次，状态相同");
  }
}

// ---------------------------------------------------------------- 单个实例
struct Tally {
  int solvable = 0;
  int bfs_worse_cost = 0;      // 8 邻域下 BFS 路径严格比最短路长
  int dij_more_steps = 0;      // 8 邻域下 Dijkstra 路径步数严格比 BFS 多
  int dfs_worse_cost = 0;
  double dfs_worst_ratio = 1.0;
  double bfs_worst_ratio = 1.0;
};

void runOnce(int idx, unsigned seed, int w, int h, bool conn8, bool corner_cut, Tally* tally) {
  GridMap2D map(w, h, 0.1, 0.0, 0.0);
  switch (idx % 4) {
    case 0: map.fillRandomObstacles(0.22, seed, 1); break;
    case 1: map.fillRandomObstacles(0.28, seed, 2); break;
    case 2: map.fillWalls(3, seed, 1); break;
    default: map.fillMaze(seed); break;
  }

  // 起终点：取地图对角附近的两个空格；找不到就跳过这个实例
  auto find_free = [&](int fx, int fy, int step, int* ox, int* oy) {
    for (int k = 0; k < map.numCells(); ++k) {
      const int x = fx + step * (k % w);
      const int y = fy + step * (k / w);
      if (!map.inside(x, y)) continue;
      if (map.isFree(x, y)) { *ox = x; *oy = y; return true; }
    }
    return false;
  };
  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!find_free(0, 0, 1, &sx, &sy)) return;
  if (!find_free(w - 1, h - 1, -1, &gx, &gy)) return;
  if (sx == gx && sy == gy) return;

  char buf[256];
  std::snprintf(buf, sizeof(buf), "实例#%d seed=%u %dx%d %d邻域 (%d,%d)->(%d,%d)", idx, seed, w, h,
                conn8 ? 8 : 4, sx, sy, gx, gy);
  g_ctx = buf;

  SearchConfig cfg;
  cfg.use_8_connected = conn8;
  cfg.allow_corner_cutting = corner_cut;
  cfg.record_trace = true;
  cfg.record_stale_pops = (idx % 2 == 0);  // 两种录制模式都要能通过检查

  GraphSearch bfs, dfs, dij;
  cfg.algorithm = Algorithm::kBfs;      bfs.setMap(&map); bfs.setConfig(cfg); bfs.search(sx, sy, gx, gy);
  cfg.algorithm = Algorithm::kDfs;      dfs.setMap(&map); dfs.setConfig(cfg); dfs.search(sx, sy, gx, gy);
  cfg.algorithm = Algorithm::kDijkstra; dij.setMap(&map); dij.setConfig(cfg); dij.search(sx, sy, gx, gy);

  const bool ok_bfs = bfs.stats().result == SearchResult::kSuccess;
  const bool ok_dfs = dfs.stats().result == SearchResult::kSuccess;
  const bool ok_dij = dij.stats().result == SearchResult::kSuccess;

  // ---- 检查 5：三个算法对"有没有解"必须给出同一个答案 ----
  check(ok_bfs == ok_dfs && ok_bfs == ok_dij, "三个算法对可达性的判断一致");

  // ---- 检查 3：BFS 的 depth 必须逐格等于独立参考 BFS ----
  {
    const std::vector<int> ref = referenceBfsDepth(map, map.toId(sx, sy), conn8, corner_cut);
    bool all_eq = true;
    for (int id = 0; id < map.numCells(); ++id) {
      // 只比较 BFS 真正 CLOSED 掉的格子：搜到终点就提前返回了，
      // 剩下的格子 BFS 还没定过 depth，拿来比没有意义。
      if (bfs.nodes()[id].state != NodeState::kClosed) continue;
      if (bfs.nodes()[id].depth != ref[id]) all_eq = false;
    }
    check(all_eq, "BFS 的 depth[] 逐格等于独立参考 BFS（即最少步数）");
    if (ok_bfs) {
      check(bfs.stats().path_steps == ref[map.toId(gx, gy)], "BFS 路径步数 == 参考 BFS 的层数");
    }
  }

  // ---- 检查 4：Dijkstra 的代价 == A*(h≡0) == A*(octile) ----
  if (ok_dij) {
    astar_tutorial::AStarConfig acfg;
    acfg.use_8_connected = conn8;
    acfg.allow_corner_cutting = corner_cut;
    acfg.record_trace = false;
    for (const astar_tutorial::Heuristic hh :
         {astar_tutorial::Heuristic::kZero, astar_tutorial::Heuristic::kDiagonal}) {
      astar_tutorial::AStar a;
      acfg.heuristic = hh;
      a.setMap(&map);
      a.setConfig(acfg);
      a.search(sx, sy, gx, gy);
      check(a.stats().result == astar_tutorial::SearchResult::kSuccess,
            "A* 和 Dijkstra 对可达性的判断一致");
      check(std::fabs(a.stats().path_cost - dij.stats().path_cost) < 1e-6,
            std::string("Dijkstra 的代价 == A*(") + astar_tutorial::toString(hh) + ") 的代价");
    }
  }

  if (ok_bfs) validatePath(map, bfs, sx, sy, gx, gy, "BFS");
  if (ok_dfs) validatePath(map, dfs, sx, sy, gx, gy, "DFS");
  if (ok_dij) validatePath(map, dij, sx, sy, gx, gy, "Dijkstra");

  validateReplay(bfs, map.numCells(), "BFS");
  validateReplay(dfs, map.numCells(), "DFS");
  validateReplay(dij, map.numCells(), "Dijkstra");

  // ---- 检查 8：BFS 每个格子只入队一次 ----
  check(bfs.stats().stale_pops == 0, "BFS 的 stale_pops 恒为 0");
  check(bfs.stats().pushes >= bfs.stats().expanded, "BFS 的入队次数 >= 扩展次数");
  // Dijkstra 的懒惰删除：堆里多出来的条目，最终都会以 stale_pop 的形式被丢掉，
  // 或者还留在堆里没弹出来 —— 无论如何 stale_pops 不可能超过重复入队的总数。
  check(dij.stats().stale_pops <= dij.stats().pushes - dij.stats().expanded ||
            dij.stats().result != SearchResult::kSuccess,
        "Dijkstra: stale_pops <= 多余的入队次数");

  // ---- 检查 7：那些必须成立的不等式 ----
  if (ok_bfs && ok_dij) {
    ++tally->solvable;
    const double bc = bfs.stats().path_cost;
    const double dc = dij.stats().path_cost;
    check(bfs.stats().path_steps <= dij.stats().path_steps, "BFS 的步数 <= Dijkstra 的步数");
    check(bc >= dc - 1e-6, "BFS 的代价 >= Dijkstra 的代价（Dijkstra 是代价下界）");
    if (bc > dc + 1e-6) {
      ++tally->bfs_worse_cost;
      tally->bfs_worst_ratio = std::max(tally->bfs_worst_ratio, bc / dc);
    }
    if (dij.stats().path_steps > bfs.stats().path_steps) ++tally->dij_more_steps;
    if (!conn8) {
      // 4 邻域（本包默认）：所有边代价都是 1，两个目标函数变成同一个 -> 必须完全相等
      check(std::fabs(bc - dc) < 1e-6, "4 邻域下 BFS 的代价 == Dijkstra 的代价");
      check(bfs.stats().path_steps == dij.stats().path_steps,
            "4 邻域下 BFS 的步数 == Dijkstra 的步数");
      // 而且 depth 和 g 是同一个数：每条边都算 1 步、代价也是 1
      bool depth_eq_g = true;
      for (int id = 0; id < map.numCells(); ++id) {
        const Node& n = dij.nodes()[id];
        if (n.state == NodeState::kUnvisited) continue;
        if (std::fabs(n.g - static_cast<double>(n.depth)) > 1e-9) depth_eq_g = false;
      }
      check(depth_eq_g, "4 邻域下逐格 g == depth（边代价全为 1 的直接后果）");

      // 单位边权下 Dijkstra 的**松弛永远不会成功**：
      // 出队的 g 单调不减，弹出 g=k 的节点时，已在 frontier 里的邻居 v 满足
      // g[v] <= k+1（它是从某个 g<=k 的节点发现的），而新路径给出 g_new = k+1 >= g[v]。
      // 所以既不会"换爹"，也不会重复入堆 —— 堆退化成了一个 FIFO 队列，
      // 这正是"4 邻域用 BFS 就够了"的机器可验证版本。
      int updated_better = 0, pushed_again = 0;
      for (const SearchStep& st : dij.trace()) {
        for (const NeighborEvent& ev : st.neighbors) {
          if (ev.action == NeighborAction::kUpdatedBetter) ++updated_better;
          if (ev.action == NeighborAction::kPushedAgain) ++pushed_again;
        }
      }
      check(updated_better == 0, "4 邻域下 Dijkstra 一次成功松弛都没有（不会换爹）");
      check(pushed_again == 0, "4 邻域下 Dijkstra 不会重复入堆");
      check(dij.stats().stale_pops == 0, "4 邻域下 Dijkstra 的 stale_pops 也是 0");
    }
  }
  if (ok_dfs && ok_dij) {
    const double fc = dfs.stats().path_cost;
    const double dc = dij.stats().path_cost;
    check(fc >= dc - 1e-6, "DFS 的代价 >= Dijkstra 的代价");
    check(dfs.stats().path_steps >= bfs.stats().path_steps, "DFS 的步数 >= BFS 的步数");
    if (fc > dc + 1e-6) {
      ++tally->dfs_worse_cost;
      tally->dfs_worst_ratio = std::max(tally->dfs_worst_ratio, fc / dc);
    }
  }

  // ---- 杂项：扩展数不可能超过格子数 ----
  for (const GraphSearch* s : {&bfs, &dfs, &dij}) {
    check(s->stats().expanded <= map.numCells(), "扩展节点数 <= 格子总数");
  }
}

// 边界情况：起点==终点、起终点在障碍上、地图没设、被墙彻底隔开
void runEdgeCases() {
  std::printf("\n[边界情况]\n");
  g_ctx = "边界情况";

  // 没设地图
  {
    GraphSearch s;
    check(s.search(0, 0, 1, 1) == SearchResult::kFailedBadInput, "没设地图 -> kFailedBadInput");
  }
  // 起点 == 终点
  {
    GridMap2D map(5, 5, 1.0, 0.0, 0.0);
    GraphSearch s;
    s.setMap(&map);
    for (const Algorithm a : {Algorithm::kBfs, Algorithm::kDfs, Algorithm::kDijkstra}) {
      SearchConfig cfg;
      cfg.algorithm = a;
      s.setConfig(cfg);
      check(s.search(2, 2, 2, 2) == SearchResult::kSuccess, "起点==终点 -> 成功");
      check(s.stats().path_size == 1, "起点==终点 -> 路径只有 1 格");
      check(s.stats().path_steps == 0, "起点==终点 -> 0 步");
      check(std::fabs(s.stats().path_cost) < 1e-12, "起点==终点 -> 代价 0");
      check(s.stats().expanded == 1, "起点==终点 -> 只扩展 1 个节点");
    }
  }
  // 起点在障碍上 / 越界
  {
    GridMap2D map(5, 5, 1.0, 0.0, 0.0);
    map.setOccupied(1, 1, true);
    GraphSearch s;
    s.setMap(&map);
    check(s.search(1, 1, 4, 4) == SearchResult::kFailedBadInput, "起点在障碍上 -> kFailedBadInput");
    check(s.search(0, 0, 1, 1) == SearchResult::kFailedBadInput, "终点在障碍上 -> kFailedBadInput");
    check(s.search(-1, 0, 4, 4) == SearchResult::kFailedBadInput, "起点越界 -> kFailedBadInput");
    check(s.search(0, 0, 5, 5) == SearchResult::kFailedBadInput, "终点越界 -> kFailedBadInput");
  }
  // 被墙彻底隔开：三个算法都必须报无解，而且都把左半边扫完
  {
    GridMap2D map(9, 5, 1.0, 0.0, 0.0);
    for (int y = 0; y < 5; ++y) map.setOccupied(4, y, true);
    for (const Algorithm a : {Algorithm::kBfs, Algorithm::kDfs, Algorithm::kDijkstra}) {
      GraphSearch s;
      SearchConfig cfg;
      cfg.algorithm = a;
      s.setMap(&map);
      s.setConfig(cfg);
      check(s.search(0, 0, 8, 0) == SearchResult::kFailedNoPath, "被墙隔开 -> kFailedNoPath");
      check(s.stats().expanded == 4 * 5, "无解时扫完整个连通分量(左半边 4x5=20 格)");
      check(s.pathIds().empty(), "无解时路径为空");
    }
  }
  // max_iterations 生效
  {
    GridMap2D map(30, 30, 1.0, 0.0, 0.0);
    GraphSearch s;
    SearchConfig cfg;
    cfg.max_iterations = 10;
    s.setMap(&map);
    s.setConfig(cfg);
    check(s.search(0, 0, 29, 29) == SearchResult::kFailedMaxIter, "撞上限 -> kFailedMaxIter");
    check(s.stats().expanded == 10, "撞上限时正好扩展了 10 个节点");
  }
  // 4 邻域下不能出现斜线；单格走廊
  {
    GridMap2D map;
    map.loadFromAscii({".#.", ".#.", "..."}, 1.0, 0.0, 0.0);
    for (const Algorithm a : {Algorithm::kBfs, Algorithm::kDfs, Algorithm::kDijkstra}) {
      SearchConfig cfg;
      cfg.algorithm = a;
      cfg.use_8_connected = false;
      GraphSearch s;
      s.setMap(&map);
      s.setConfig(cfg);
      // 注意 loadFromAscii 第一行是 y=height-1（图像上下翻转），这里只关心能不能通
      check(s.search(0, 0, 2, 0) == SearchResult::kSuccess, "4 邻域绕过竖墙 -> 成功");
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const int num_cases = (argc > 1) ? std::atoi(argv[1]) : 300;

  std::printf("search_tutorial 自检：%d 个随机实例 + 边界情况\n", num_cases);
  std::printf("交叉验证对象：独立参考 BFS、astar_tutorial 的 A*(h=0) 和 A*(octile)、回放器\n");

  Tally t8, t4;
  for (int i = 0; i < num_cases; ++i) {
    const unsigned seed = static_cast<unsigned>(i * 2654435761u % 100000u);
    const int w = 12 + (i % 17);
    const int h = 9 + (i % 11);
    const bool corner_cut = (i % 5 == 0);
    runOnce(i, seed, w, h, /*conn8=*/true, corner_cut, &t8);
    runOnce(i, seed, w, h, /*conn8=*/false, corner_cut, &t4);
  }
  runEdgeCases();

  std::printf("\n================ 统计（8 邻域，%d 个可解实例）================\n", t8.solvable);
  std::printf("BFS 的路径严格比最短路更长 : %d 次 (%.1f%%)，最差比最优解长 %.2f%%\n",
              t8.bfs_worse_cost,
              t8.solvable ? 100.0 * t8.bfs_worse_cost / t8.solvable : 0.0,
              100.0 * (t8.bfs_worst_ratio - 1.0));
  std::printf("Dijkstra 的步数严格比 BFS 多 : %d 次 (%.1f%%)\n", t8.dij_more_steps,
              t8.solvable ? 100.0 * t8.dij_more_steps / t8.solvable : 0.0);
  std::printf("DFS 的路径严格比最短路更长   : %d 次，最差长 %.1f%%\n", t8.dfs_worse_cost,
              100.0 * (t8.dfs_worst_ratio - 1.0));
  std::printf("=> \"BFS 步数最少 != 代价最短\" 不是理论上的吹毛求疵，8 邻域随机地图上就是这个频率。\n");
  std::printf("\n================ 统计（4 邻域 = 本包默认，%d 个可解实例）================\n",
              t4.solvable);
  std::printf("BFS 的路径严格比最短路更长 : %d 次（必须是 0）\n", t4.bfs_worse_cost);
  std::printf("Dijkstra 的步数严格比 BFS 多 : %d 次（必须是 0）\n", t4.dij_more_steps);
  std::printf("=> 4 邻域下每条边代价都是 1，「步数最少」和「代价最小」是同一个目标，\n");
  std::printf("   所以 BFS 和 Dijkstra 的 cost / steps 被逐个实例断言为完全相等。\n");
  std::printf("   这就是默认配置下可以放心用 BFS 的理由。\n");

  std::printf("\n================ 结果 ================\n");
  std::printf("检查 %d 项，失败 %d 项\n", g_checks, g_failures);
  if (g_failures == 0) std::printf("全部通过 ✓\n");
  return g_failures == 0 ? 0 : 1;
}
