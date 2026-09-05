#include "search_tutorial/graph_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace search_tutorial {

// 浮点比较用的容差。g 是一堆 1 和 √2 累加出来的，直接用 == / < 会被舍入误差咬。
static constexpr double kEps = 1e-9;

const char* toString(Algorithm a) {
  switch (a) {
    case Algorithm::kBfs: return "BFS(广度优先,FIFO队列)";
    case Algorithm::kDfs: return "DFS(深度优先,LIFO栈)";
    case Algorithm::kDijkstra: return "Dijkstra(小顶堆按g排序)";
  }
  return "unknown";
}

const char* shortName(Algorithm a) {
  switch (a) {
    case Algorithm::kBfs: return "BFS";
    case Algorithm::kDfs: return "DFS";
    case Algorithm::kDijkstra: return "Dijkstra";
  }
  return "?";
}

bool parseAlgorithm(const std::string& s, Algorithm* out) {
  if (s == "bfs" || s == "BFS") { *out = Algorithm::kBfs; return true; }
  if (s == "dfs" || s == "DFS") { *out = Algorithm::kDfs; return true; }
  if (s == "dijkstra" || s == "Dijkstra" || s == "ucs") { *out = Algorithm::kDijkstra; return true; }
  return false;
}

const char* toString(NeighborAction a) {
  switch (a) {
    case NeighborAction::kPushedNew: return "新发现,入frontier";
    case NeighborAction::kPushedAgain: return "已在栈里,DFS再压一次";
    case NeighborAction::kUpdatedBetter: return "更短,松弛g并重入";
    case NeighborAction::kSkippedSeen: return "跳过:已在队列里";
    case NeighborAction::kSkippedWorse: return "跳过:这条路更长";
    case NeighborAction::kSkippedClosed: return "跳过:已CLOSED";
    case NeighborAction::kSkippedOccupied: return "跳过:障碍";
    case NeighborAction::kSkippedOutside: return "跳过:越界";
    case NeighborAction::kSkippedCorner: return "跳过:斜穿墙角";
  }
  return "unknown";
}

const char* toString(SearchResult r) {
  switch (r) {
    case SearchResult::kSuccess: return "成功";
    case SearchResult::kFailedNoPath: return "失败:frontier耗尽,起终点不连通";
    case SearchResult::kFailedMaxIter: return "失败:达到迭代上限";
    case SearchResult::kFailedBadInput: return "失败:输入非法(无地图/起终点越界或在障碍上)";
  }
  return "unknown";
}

// ============================================================ 保证了什么
//
// 这两个谓词是本包的结论所在，值得单独盯着看：
//
//   * Dijkstra 优化 g（几何代价），所以永远给出**代价最小**的路径；
//     它顺带也步数最少 —— 但只在"所有边代价相等"时成立。8 邻域有 1 和 √2 两种边权，
//     代价最小的路可能比步数最少的路多走几格。
//   * BFS 优化步数，所以永远给出**步数最少**的路径；
//     它顺带也代价最小 —— 同样只在"所有边代价相等"时成立，也就是 4 邻域。
//   * DFS 两个都不优化。它只保证"如果有路，一定能找到一条"。
//
// 本包默认 4 邻域，于是默认配置下 BFS 和 Dijkstra 这两行**同时为 true**：
// 它们求的是同一个最优值。这不是巧合，是 "边代价全为 1" 的直接后果。
// 想让它们分开，把 use_8_connected 打开（rviz 里按 v）。
bool GraphSearch::guaranteesMinCost() const {
  switch (cfg_.algorithm) {
    case Algorithm::kDijkstra: return true;
    case Algorithm::kBfs: return !cfg_.use_8_connected;  // 4 邻域下边代价全是 1
    case Algorithm::kDfs: return false;
  }
  return false;
}

bool GraphSearch::guaranteesMinSteps() const {
  switch (cfg_.algorithm) {
    case Algorithm::kBfs: return true;
    case Algorithm::kDijkstra: return !cfg_.use_8_connected;
    case Algorithm::kDfs: return false;
  }
  return false;
}

std::string GraphSearch::optimalityNote() const {
  switch (cfg_.algorithm) {
    case Algorithm::kBfs:
      return cfg_.use_8_connected
                 ? std::string("BFS 保证步数最少，但 8 邻域下边代价有 1 和 √2 两种，"
                               "所以**不保证**几何代价最短（3 步斜线 3√2≈4.243 比 4 步直线更长）。"
                               "拿 Dijkstra 的 cost 一比就能看出来。")
                 : std::string("BFS + 4 邻域（本包默认）：所有边代价都是 1，"
                               "步数最少 == 代价最小，所以 BFS 就是**正确的最短路算法**，"
                               "cost 和 Dijkstra 逐位相同 —— 而且不用堆，省掉一个 log。"
                               "（出队顺序和具体路径可能不同：并列时谁先出队是各自的实现细节。）");
    case Algorithm::kDfs:
      return std::string("DFS 什么都不保证：它只保证\"有路就能找到一条\"。"
                         "路径通常又长又扭，因为它一头扎到底、只在撞墙时才回头。"
                         "它的价值在连通性/可达性判断，不在最短路。");
    case Algorithm::kDijkstra:
      return cfg_.use_8_connected
                 ? std::string("Dijkstra 保证几何代价最小；步数不一定最少"
                               "（宁可多走一格直线，也不肯多走一步 √2 的斜线时就会这样）。")
                 : std::string("Dijkstra + 4 邻域（本包默认）：边代价全是 1，"
                               "代价最小 == 步数最少，答案和 BFS 一模一样 —— 这种场合用 BFS "
                               "更划算（省掉堆的 log 和那些重复条目）。"
                               "Dijkstra 的价值在边代价**不相等**的时候：打开 8 邻域，"
                               "或者给格子加上地形代价。");
  }
  return std::string();
}

// ============================================================ 主搜索
void GraphSearch::resetNodes() {
  const int n = map_->numCells();
  nodes_.assign(n, Node{});
  for (int id = 0; id < n; ++id) {
    map_->toXY(id, &nodes_[id].x, &nodes_[id].y);
  }
  trace_.clear();
  stats_ = SearchStats{};
}

SearchResult GraphSearch::search(int sx, int sy, int gx, int gy) {
  const auto t0 = std::chrono::steady_clock::now();

  // ---------- 0. 输入检查 ----------
  if (map_ == nullptr || map_->numCells() == 0) {
    stats_ = SearchStats{};
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }
  resetNodes();
  if (!map_->inside(sx, sy) || !map_->inside(gx, gy) || map_->isOccupied(sx, sy) ||
      map_->isOccupied(gx, gy)) {
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }

  start_id_ = map_->toId(sx, sy);
  goal_id_ = map_->toId(gx, gy);

  // 方向表：前 4 个是直走(右/左/上/下，代价 1)，后 4 个是斜走(代价 √2)。
  // 默认 use_8_connected=false，只取前 4 个 —— **所以这个顺序不能改**，
  // 一旦把某个斜方向挪到前 4 个里，4 邻域就悄悄变成了"允许对角线"。
  //
  // 顺便说一句：这个方向表的**顺序**对 BFS 和 Dijkstra 的结果毫无影响（只影响出队顺序），
  // 但对 DFS 影响巨大 —— DFS 走的是"最后压进去的那个方向"，
  // 换一下 kDX/kDY 的顺序，DFS 的路径会完全变一个样。这本身就说明 DFS 有多不稳定。
  static const int kDX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const int num_dirs = cfg_.use_8_connected ? 8 : 4;

  Frontier frontier;
  frontier.reset(cfg_.algorithm);
  int open_count = 0;    // 状态为 OPEN 的**节点**数（不含容器里的重复条目）
  int closed_count = 0;

  // ---------- 1. 起点入 frontier ----------                       [伪代码 1]
  {
    Node& s = nodes_[start_id_];
    s.g = 0.0;      // 起点到自己，代价 0
    s.depth = 0;    // 也是 0 步
    s.parent = -1;  // 回溯的终止标志
    s.state = NodeState::kOpen;
    frontier.push({start_id_, -1, 0.0, 0});
    ++open_count;
    ++stats_.pushes;
    stats_.frontier_peak = 1;
    stats_.open_peak = 1;
  }

  // ---------- 2. 主循环 ----------                                [伪代码 2]
  while (!frontier.empty()) {
    if (cfg_.max_iterations > 0 && stats_.iterations >= cfg_.max_iterations) {
      stats_.result = SearchResult::kFailedMaxIter;
      break;
    }

    // 2.1 取出一个条目。★ 三个算法唯一的区别，全在 Frontier::pop() 里。 [伪代码 3]
    const FrontierItem item = frontier.pop();
    const int cur_id = item.id;

    // 2.2 重复条目的清理（懒惰去重）。                              [伪代码 4]
    //
    // 什么时候会出现重复条目？
    //   * DFS：同一个格子可以被不同的邻居各压一次，压进去时并不检查"是不是已经在栈里"
    //     —— 这正是标准 DFS 的写法，不是 bug；
    //   * Dijkstra：松弛成功时又 push 了一份更小的 g，旧条目还留在堆里。
    //     小顶堆保证**更好的那份先出堆**，等它把节点 CLOSED 掉之后，
    //     旧条目出堆时就在这里被丢掉。所以这一个 CLOSED 判断就够了，
    //     不需要再额外比 "item.g > g[cur]"。
    //   * BFS：一次都不会出现。入队时就把 state 标成 OPEN，
    //     后续邻居看到 OPEN 直接跳过 -> 每个格子恰好入队一次。
    //     （BFS 的 stale_pops 恒为 0，可以当成实现是否写对的一个自检。）
    if (nodes_[cur_id].state == NodeState::kClosed) {
      ++stats_.stale_pops;
      if (cfg_.record_trace && cfg_.record_stale_pops) {
        SearchStep step;
        step.iteration = static_cast<int>(trace_.size()) + 1;
        step.cur_id = cur_id;
        step.cur_x = nodes_[cur_id].x;
        step.cur_y = nodes_[cur_id].y;
        step.cur_g = item.g;          // 条目自己带的值，不是节点已落定的值
        step.cur_depth = item.depth;
        step.cur_parent = item.parent;
        step.is_stale_pop = true;
        step.frontier_size_after = frontier.size();
        step.open_size_after = open_count;
        step.closed_size_after = closed_count;
        trace_.push_back(step);
      }
      continue;
    }

    // 2.3 落定这个节点：认条目上带的 parent/g/depth，然后转 CLOSED。 [伪代码 5]
    //
    // 为什么 g/parent 是从**条目**上抄下来的，而不是节点自己存的？
    //   因为 DFS 允许同一个格子在栈里躺着好几份，每份的父亲不一样。
    //   到底认哪个爹，取决于**哪一份先被弹出来** —— 所以只能等弹出的这一刻才定。
    //   BFS/Dijkstra 下节点上存的值和有效条目一致，抄哪个都一样。
    Node& cur = nodes_[cur_id];
    cur.g = item.g;
    cur.depth = item.depth;
    cur.parent = item.parent;
    cur.state = NodeState::kClosed;
    --open_count;
    ++closed_count;
    ++stats_.iterations;
    stats_.expanded = stats_.iterations;

    SearchStep step;
    step.iteration = static_cast<int>(trace_.size()) + 1;
    step.cur_id = cur_id;
    step.cur_x = cur.x;
    step.cur_y = cur.y;
    step.cur_g = cur.g;
    step.cur_depth = cur.depth;
    step.cur_parent = cur.parent;

    // 2.4 到终点了吗？                                              [伪代码 6]
    //
    // 判断放在**出队之后**，而不是"发现终点就返回"。
    //   * 对 Dijkstra 这是**必须**的：终点第一次入堆时它的 g 只是某一条路的代价，
    //     不一定最优；只有等它以最小 g 出堆才能确认。提前返回 = 经典的"路径偏长"bug。
    //   * 对 BFS，提前返回其实不影响步数最优（第一次遇到终点时步数已经最少了），
    //     但统一成"出队才算到达"能让三个算法共用同一段代码，也少一个特例。
    //   * 对 DFS 无所谓，它本来就不保证任何东西。
    if (cur_id == goal_id_) {
      step.is_goal = true;
      step.frontier_size_after = frontier.size();
      step.open_size_after = open_count;
      step.closed_size_after = closed_count;
      if (cfg_.record_trace) trace_.push_back(step);
      stats_.result = SearchResult::kSuccess;
      stats_.path_cost = cur.g;
      stats_.path_length_m = cur.g * map_->resolution();
      stats_.path_size = static_cast<int>(pathIds().size());
      // path_steps 应当恒等于 cur.depth —— 两个独立算出来的量必须对得上，
      // 对不上就说明父链断了。search_selfcheck 会真的去断言这一条。
      stats_.path_steps = std::max(0, stats_.path_size - 1);
      const auto t1 = std::chrono::steady_clock::now();
      stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      return stats_.result;
    }

    // 2.5 遍历邻居                                                  [伪代码 7]
    for (int d = 0; d < num_dirs; ++d) {
      const int nx = cur.x + kDX[d];
      const int ny = cur.y + kDY[d];
      const bool diagonal = (kDX[d] != 0 && kDY[d] != 0);

      NeighborEvent ev;
      ev.x = nx;
      ev.y = ny;
      ev.edge_cost = diagonal ? kSqrt2 : 1.0;

      // (a) 越界 / 障碍 -> 不可通行                                 [伪代码 8]
      if (!map_->inside(nx, ny)) {
        ev.action = NeighborAction::kSkippedOutside;
        if (cfg_.record_trace) step.neighbors.push_back(ev);
        continue;
      }
      ev.id = map_->toId(nx, ny);
      if (map_->isOccupied(nx, ny)) {
        ev.action = NeighborAction::kSkippedOccupied;
        if (cfg_.record_trace) step.neighbors.push_back(ev);
        continue;
      }

      // (b) 斜穿墙角。两个正交邻居只要有一个是障碍，这一步就是"从缝里钻"。
      //     真机器人有体积，钻不过去。严格禁止(两个都要空)才安全。
      if (diagonal && !cfg_.allow_corner_cutting) {
        if (map_->isOccupied(cur.x + kDX[d], cur.y) || map_->isOccupied(cur.x, cur.y + kDY[d])) {
          ev.action = NeighborAction::kSkippedCorner;
          if (cfg_.record_trace) step.neighbors.push_back(ev);
          continue;
        }
      }

      Node& nb = nodes_[ev.id];
      const double g_new = cur.g + ev.edge_cost;
      const int d_new = cur.depth + 1;
      ev.g_old = nb.g;
      ev.g_new = g_new;
      ev.depth_new = d_new;

      // (c) 已经 CLOSED -> 跳过。三个算法在这里完全一致。            [伪代码 9]
      //
      // 对 Dijkstra，这一步的合法性是有证明的（出队瞬间 g 已最优，见 doc/03-dijkstra.md）。
      // 对 BFS 同理（出队瞬间 depth 已最小）。
      // 对 DFS 它只是"别绕回去"，没有任何最优性含义 —— 也正因为如此，
      // DFS 会因为"先到的那条路把格子占了"而彻底错过更好的路。
      if (nb.state == NodeState::kClosed) {
        ev.action = NeighborAction::kSkippedClosed;
        if (cfg_.record_trace) step.neighbors.push_back(ev);
        continue;
      }

      if (nb.state == NodeState::kUnvisited) {
        // (d) 第一次遇到 -> 入 frontier。三个算法也完全一致。       [伪代码 11]
        //
        // 节点上先把 g/depth/parent 记下来：一是给可视化用（绿色格子上要显示数字），
        // 二是 Dijkstra 下面松弛时要拿 nb.g 来比。
        nb.g = g_new;
        nb.depth = d_new;
        nb.parent = cur_id;
        nb.state = NodeState::kOpen;
        frontier.push({ev.id, cur_id, g_new, d_new});
        ++open_count;
        ++stats_.pushes;
        ev.action = NeighborAction::kPushedNew;
      } else {
        // (e) 已经在 frontier 里了。★ 三个算法在这里分道扬镳。      [伪代码 12]
        switch (cfg_.algorithm) {
          case Algorithm::kBfs:
            // BFS：已经进过队列，说明有人比我更早（或同层）发现它，
            // 那条路的步数一定 <= 我这条。既然只关心步数，就没必要再看一眼。
            // 这一行就是 BFS "每个格子只入队一次" 的全部原因。
            ev.action = NeighborAction::kSkippedSeen;
            break;

          case Algorithm::kDfs:
            // DFS：照压。标准 DFS 压栈时**不检查**是不是已经在栈里。
            // 效果：这个格子在栈里躺了两份，靠上的那份先弹出来 —— 也就是"最新发现的路径优先"，
            // 正是"一头扎到底"的来源。代价是栈里有重复（stats.stale_pops 就是它们）。
            //
            // 节点上的 g/depth/parent 也跟着更新成最新压进去的这份 —— 纯粹是为了让
            // 可视化显示的数字和栈顶那份一致；真正认哪一份，等它出栈时才定（见 2.3）。
            nb.g = g_new;
            nb.depth = d_new;
            nb.parent = cur_id;
            frontier.push({ev.id, cur_id, g_new, d_new});
            ++stats_.pushes;
            ev.action = NeighborAction::kPushedAgain;
            break;

          case Algorithm::kDijkstra:
            // Dijkstra：松弛（relax）。"我这条路更便宜"就改小 g、换爹、重新入堆。
            // g[nb] 本来是个上界（"我知道有条路要花这么多"），发现更便宜的走法就把它**放松**下来。
            //
            // 为什么用严格 < 而不是 <= ？相等时换爹不改变代价，只是白白多 push 一次，
            // 还会让搜索过程随实现细节抖动。
            if (g_new < nb.g - kEps) {
              nb.g = g_new;
              nb.depth = d_new;
              nb.parent = cur_id;
              frontier.push({ev.id, cur_id, g_new, d_new});
              ++stats_.pushes;
              ev.action = NeighborAction::kUpdatedBetter;
            } else {
              ev.action = NeighborAction::kSkippedWorse;
            }
            break;
        }
      }
      if (cfg_.record_trace) step.neighbors.push_back(ev);
    }

    // 峰值只可能出现在一步的末尾（一步里先弹 1 个，再压若干个）
    stats_.frontier_peak = std::max(stats_.frontier_peak, frontier.size());
    stats_.open_peak = std::max(stats_.open_peak, static_cast<std::size_t>(open_count));

    step.frontier_size_after = frontier.size();
    step.open_size_after = open_count;
    step.closed_size_after = closed_count;
    if (cfg_.record_trace) trace_.push_back(step);
  }

  // ---------- 3. frontier 空了还没到终点 ----------                [伪代码 13]
  //
  // 三个算法在这里也一致：它们都会把起点所在的**整个连通分量**访问完。
  // 所以"有解 / 无解"的结论三者必然相同 —— 只有"路径长什么样"不同。
  // search_selfcheck 把这条当成一个断言在查。
  if (stats_.result != SearchResult::kFailedMaxIter) {
    stats_.result = SearchResult::kFailedNoPath;
  }
  const auto t1 = std::chrono::steady_clock::now();
  stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return stats_.result;
}

// ============================================================ 回溯路径
std::vector<int> GraphSearch::pathIds() const {
  std::vector<int> path;
  if (stats_.result != SearchResult::kSuccess || goal_id_ < 0) return path;

  // 从终点顺着 parent 一路走回起点，再反转。                        [伪代码 6]
  // 起点的 parent 是 -1，是循环的天然终止条件。
  int id = goal_id_;
  // guard：正常情况下链长 <= 节点数；加个上限，父指针万一成环也不会死循环。
  for (int guard = 0; id >= 0 && guard <= static_cast<int>(nodes_.size()); ++guard) {
    path.push_back(id);
    if (id == start_id_) break;
    id = nodes_[id].parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<std::pair<int, int>> GraphSearch::pathGrid() const {
  std::vector<std::pair<int, int>> out;
  for (const int id : pathIds()) out.emplace_back(nodes_[id].x, nodes_[id].y);
  return out;
}

std::vector<std::pair<double, double>> GraphSearch::pathWorld() const {
  std::vector<std::pair<double, double>> out;
  if (map_ == nullptr) return out;
  for (const int id : pathIds()) {
    double wx = 0.0;
    double wy = 0.0;
    map_->gridToWorld(nodes_[id].x, nodes_[id].y, &wx, &wy);
    out.emplace_back(wx, wy);
  }
  return out;
}

}  // namespace search_tutorial
