#include "astar_tutorial/astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace astar_tutorial {

// 浮点比较用的容差。g 是一堆 1 和 √2 累加出来的，直接用 == / < 会被舍入误差咬。
static constexpr double kEps = 1e-9;

const char* toString(Heuristic h) {
  switch (h) {
    case Heuristic::kZero: return "zero(Dijkstra)";
    case Heuristic::kManhattan: return "manhattan";
    case Heuristic::kEuclidean: return "euclidean";
    case Heuristic::kDiagonal: return "diagonal(octile)";
  }
  return "unknown";
}

bool parseHeuristic(const std::string& s, Heuristic* out) {
  if (s == "zero" || s == "dijkstra") { *out = Heuristic::kZero; return true; }
  if (s == "manhattan") { *out = Heuristic::kManhattan; return true; }
  if (s == "euclidean") { *out = Heuristic::kEuclidean; return true; }
  if (s == "diagonal" || s == "octile") { *out = Heuristic::kDiagonal; return true; }
  return false;
}

const char* toString(NeighborAction a) {
  switch (a) {
    case NeighborAction::kPushedNew: return "新发现,入OPEN";
    case NeighborAction::kUpdatedBetter: return "更短,更新g和父节点";
    case NeighborAction::kSkippedWorse: return "跳过:这条路更长";
    case NeighborAction::kSkippedClosed: return "跳过:已在CLOSED";
    case NeighborAction::kSkippedOccupied: return "跳过:障碍";
    case NeighborAction::kSkippedOutside: return "跳过:越界";
    case NeighborAction::kSkippedCorner: return "跳过:斜穿墙角";
  }
  return "unknown";
}

const char* toString(SearchResult r) {
  switch (r) {
    case SearchResult::kSuccess: return "成功";
    case SearchResult::kFailedNoPath: return "失败:OPEN耗尽,起终点不连通";
    case SearchResult::kFailedMaxIter: return "失败:达到迭代上限";
    case SearchResult::kFailedBadInput: return "失败:输入非法(无地图/起终点越界或在障碍上)";
  }
  return "unknown";
}

// ============================================================ 启发函数
double AStar::rawHeuristic(int x, int y, int gx, int gy) const {
  const double dx = std::abs(static_cast<double>(gx - x));
  const double dy = std::abs(static_cast<double>(gy - y));
  switch (cfg_.heuristic) {
    case Heuristic::kZero:
      return 0.0;
    case Heuristic::kManhattan:
      // 只能横竖走时的真实步数
      return dx + dy;
    case Heuristic::kEuclidean:
      // 直线距离。永远 <= 真实栅格代价，所以永远可采纳，但在 4 邻域下明显偏小
      // （偏小 = 更保守 = 扩展更多节点，但结果仍最优）。
      return std::sqrt(dx * dx + dy * dy);
    case Heuristic::kDiagonal: {
      // octile 距离：先尽量斜着走 min(dx,dy) 步（每步 √2），剩下的直走。
      // 这正好是 8 邻域无障碍时的真实最短代价 -> 8 邻域的最佳启发函数。
      const double dmin = std::min(dx, dy);
      const double dmax = std::max(dx, dy);
      return kSqrt2 * dmin + (dmax - dmin);
    }
  }
  return 0.0;
}

double AStar::heuristicCost(int x, int y, int gx, int gy) const {
  // f = g + weight · tie_breaker · h_raw
  return cfg_.weight * cfg_.tie_breaker * rawHeuristic(x, y, gx, gy);
}

bool AStar::isHeuristicAdmissible() const {
  // 放大系数一旦 > 1，h 就可能超过真实代价 -> 不再可采纳
  if (cfg_.weight * cfg_.tie_breaker > 1.0 + kEps) return false;
  // 8 邻域 + 曼哈顿：斜着走时曼哈顿给 2，真实只要 √2≈1.41 -> 高估 -> 不可采纳
  if (cfg_.use_8_connected && cfg_.heuristic == Heuristic::kManhattan) return false;
  return true;
}

std::string AStar::admissibilityWarning() const {
  std::string msg;
  if (cfg_.use_8_connected && cfg_.heuristic == Heuristic::kManhattan) {
    msg += "8邻域下用曼哈顿距离: h 会高估真实代价(斜走一步真实√2,曼哈顿算2),"
           "不可采纳,A* 不再保证最短路。8邻域请用 diagonal。 ";
  }
  const double scale = cfg_.weight * cfg_.tie_breaker;
  if (scale > 1.0 + kEps) {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "h 被放大了 %.4f 倍(weight=%.3f × tie_breaker=%.5f): "
                  "扩展变少但路径长度最多为最优解的 %.4f 倍。",
                  scale, cfg_.weight, cfg_.tie_breaker, scale);
    msg += buf;
  }
  return msg;
}

// ============================================================ 主搜索
void AStar::resetNodes() {
  const int n = map_->numCells();
  nodes_.assign(n, Node{});
  for (int id = 0; id < n; ++id) {
    map_->toXY(id, &nodes_[id].x, &nodes_[id].y);
  }
  trace_.clear();
  stats_ = SearchStats{};
}

SearchResult AStar::search(int sx, int sy, int gx, int gy) {
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

  // 8 邻域的 8 个方向。前 4 个是直走(代价 1)，后 4 个是斜走(代价 √2)。
  // use_8_connected=false 时只用前 4 个 —— 所以顺序不能改。
  static const int kDX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const int num_dirs = cfg_.use_8_connected ? 8 : 4;

  OpenList open;
  int open_count = 0;    // OPEN 的"真实"大小（不含堆里的过期条目）
  int closed_count = 0;

  // ---------- 1. 起点入 OPEN ----------                            [伪代码 1]
  {
    Node& s = nodes_[start_id_];
    s.g = 0.0;                                     // 起点到自己，代价 0
    s.h = heuristicCost(sx, sy, gx, gy);
    s.f = s.g + s.h;
    s.parent = -1;                                 // 回溯的终止标志
    s.state = NodeState::kOpen;
    open.push({s.f, s.h, start_id_});
    ++open_count;
    ++stats_.pushes;
  }

  // ---------- 2. 主循环 ----------                                 [伪代码 2]
  while (!open.empty()) {
    if (cfg_.max_iterations > 0 && stats_.iterations >= cfg_.max_iterations) {
      stats_.result = SearchResult::kFailedMaxIter;
      break;
    }

    // 2.1 取出 f 最小的节点。堆顶就是它，O(log n)。         [伪代码 3]
    const QueueItem top = open.top();
    open.pop();
    const int cur_id = top.id;

    // 2.2 懒惰删除的清理：这两种条目是垃圾，直接丢。
    //   - 节点已经 closed：说明它更早以更小的 f 出堆过一次；
    //   - 条目里的 f 比节点当前 f 大：说明这条目 push 之后节点的 g 又被改小、
    //     并重新 push 过一份更好的，这一份过期了。
    // 注意：丢掉的条目不算一次迭代，也不动 open_count（它早就被算过了）。
    if (nodes_[cur_id].state == NodeState::kClosed || top.f > nodes_[cur_id].f + kEps) {
      ++stats_.stale_pops;
      continue;
    }

    // 2.3 从 OPEN 移到 CLOSED。                                     [伪代码 4]
    //
    // 关键性质：**只要 h 可采纳且一致，节点出堆的那一刻它的 g 就是最优的，永远不必再改。**
    // 直觉：堆顶的 f 是全场最小；若还存在一条更短的路通向 cur，那条路上必有某个
    // 节点还在 OPEN 里，而它的 f = g + h <= 真实总代价 <= f(cur)，
    // 于是它会先于（或同时）出堆 —— 矛盾。所以 CLOSED 里的节点可以放心跳过。
    // 这条性质就是下面 kSkippedClosed 的依据，也是 A* 只需扫一遍就最优的原因。
    Node& cur = nodes_[cur_id];
    cur.state = NodeState::kClosed;
    --open_count;
    ++closed_count;
    ++stats_.iterations;
    stats_.expanded = stats_.iterations;
    stats_.open_peak = std::max(stats_.open_peak, static_cast<std::size_t>(open_count + 1));

    SearchStep step;
    if (cfg_.record_trace) {
      step.iteration = stats_.iterations;
      step.cur_id = cur_id;
      step.cur_x = cur.x;
      step.cur_y = cur.y;
      step.cur_g = cur.g;
      step.cur_h = cur.h;
      step.cur_f = cur.f;
      step.cur_parent = cur.parent;
    }

    // 2.4 到终点了吗？                                              [伪代码 5]
    //
    // 为什么判断放在**出堆之后**而不是"发现终点就返回"？
    // 因为第一次把终点放进 OPEN 时，它的 g 只是"某条路"的代价，不一定最优。
    // 只有等它以最小 f 出堆，才能确认没有更短的路了。提前返回就是常见的
    // "A* 跑出来的路径偏长" bug。
    if (cur_id == goal_id_) {
      step.is_goal = true;
      step.open_size_after = open_count;
      step.closed_size_after = closed_count;
      if (cfg_.record_trace) trace_.push_back(step);
      stats_.result = SearchResult::kSuccess;
      stats_.path_cost = cur.g;
      stats_.path_length_m = cur.g * map_->resolution();
      stats_.path_size = static_cast<int>(pathIds().size());
      const auto t1 = std::chrono::steady_clock::now();
      stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      return stats_.result;
    }

    // 2.5 遍历邻居                                                  [伪代码 6]
    for (int d = 0; d < num_dirs; ++d) {
      const int nx = cur.x + kDX[d];
      const int ny = cur.y + kDY[d];
      const bool diagonal = (kDX[d] != 0 && kDY[d] != 0);

      NeighborEvent ev;
      ev.x = nx;
      ev.y = ny;
      ev.edge_cost = diagonal ? kSqrt2 : 1.0;

      // (a) 越界 / 障碍 -> 不可通行                                 [伪代码 7]
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

      const int nb_id = ev.id;
      Node& nb = nodes_[nb_id];

      // (c) 已在 CLOSED -> 跳过。依据见 2.3 的说明。                [伪代码 7]
      if (nb.state == NodeState::kClosed) {
        ev.action = NeighborAction::kSkippedClosed;
        ev.g_old = nb.g;
        ev.g_new = cur.g + ev.edge_cost;
        ev.h = nb.h;
        ev.f_new = ev.g_new + nb.h;
        if (cfg_.record_trace) step.neighbors.push_back(ev);
        continue;
      }

      // (d) 试探代价：从起点经过 cur 走到 nb 要多少                 [伪代码 8]
      const double tentative_g = cur.g + ev.edge_cost;
      ev.g_old = nb.g;
      ev.g_new = tentative_g;

      if (nb.state == NodeState::kUnvisited) {
        // 第一次遇到。它的 g 还是 +inf，所以必然满足 g_new < g[nb]。 [伪代码 9-11]
        nb.g = tentative_g;
        nb.h = heuristicCost(nx, ny, gx, gy);   // h 只依赖坐标，一辈子只算一次
        nb.f = nb.g + nb.h;
        nb.parent = cur_id;
        nb.state = NodeState::kOpen;
        open.push({nb.f, nb.h, nb_id});
        ++open_count;
        ++stats_.pushes;

        ev.action = NeighborAction::kPushedNew;
        ev.h = nb.h;
        ev.f_new = nb.f;
      } else if (tentative_g < nb.g - kEps) {
        // 已在 OPEN，但绕 cur 走更短 -> 改小 g、换父亲、重新入堆。   [伪代码 9-11]
        // 这就是"decrease-key"，此处用懒惰删除实现（旧条目留在堆里，出堆时丢弃）。
        //
        // 为什么用严格 < 而不是 <= ？相等时换父亲不改变代价，只是白白多 push 一次，
        // 还会让搜索过程随实现细节抖动。hw_1 参考实现里写的是 <=，
        // 想看两者差别可以自己改这一行。
        nb.g = tentative_g;
        nb.f = nb.g + nb.h;
        nb.parent = cur_id;
        open.push({nb.f, nb.h, nb_id});
        ++stats_.pushes;

        ev.action = NeighborAction::kUpdatedBetter;
        ev.h = nb.h;
        ev.f_new = nb.f;
      } else {
        // 已在 OPEN，且这条路不更短 -> 什么都不做。                  [伪代码 9 判假]
        ev.action = NeighborAction::kSkippedWorse;
        ev.h = nb.h;
        ev.f_new = tentative_g + nb.h;
      }
      if (cfg_.record_trace) step.neighbors.push_back(ev);
    }

    step.open_size_after = open_count;
    step.closed_size_after = closed_count;
    if (cfg_.record_trace) trace_.push_back(step);
  }

  // ---------- 3. OPEN 空了还没到终点 ----------                    [伪代码 12]
  if (stats_.result != SearchResult::kFailedMaxIter) {
    stats_.result = SearchResult::kFailedNoPath;
  }
  const auto t1 = std::chrono::steady_clock::now();
  stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return stats_.result;
}

// ============================================================ 回溯路径
std::vector<int> AStar::pathIds() const {
  std::vector<int> path;
  if (stats_.result != SearchResult::kSuccess || goal_id_ < 0) return path;

  // 从终点顺着 parent 一路走回起点，再反转。                        [伪代码 5]
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

std::vector<std::pair<int, int>> AStar::pathGrid() const {
  std::vector<std::pair<int, int>> out;
  for (const int id : pathIds()) out.emplace_back(nodes_[id].x, nodes_[id].y);
  return out;
}

std::vector<std::pair<double, double>> AStar::pathWorld() const {
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

}  // namespace astar_tutorial
