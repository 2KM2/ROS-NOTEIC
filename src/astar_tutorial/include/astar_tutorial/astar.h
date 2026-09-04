// 手写 A* —— 核心算法。
//
// ============================ 先把伪代码摆出来 ============================
//
//   1  g[start] = 0;  f[start] = h(start);  OPEN = { start }
//   2  while OPEN 非空:
//   3      cur = OPEN 中 f 最小的节点        <- 这一行决定了它是 A* 而不是 BFS/DFS
//   4      把 cur 从 OPEN 移到 CLOSED
//   5      if cur == goal:  沿 parent 回溯出路径，结束
//   6      for nb in cur 的邻居:
//   7          if nb 是障碍 or nb 在 CLOSED:  跳过
//   8          g_new = g[cur] + cost(cur, nb)
//   9          if g_new < g[nb]:              <- "找到更短的走法"
//  10              g[nb] = g_new;  f[nb] = g_new + h(nb);  parent[nb] = cur
//  11              把 nb 放进（或更新到）OPEN
//  12  OPEN 空了还没到终点 => 无解
//
// 全部代码就是这 12 行的展开。astar.cpp 里每个环节都标了对应行号（[伪代码 N]）。
//
// ============================ 三个必须搞清的概念 ============================
//
// g(n): 从起点到 n 的**已知**最短代价。是"事实"，会随着搜索被不断改小。
// h(n): 从 n 到终点的**估计**代价。是"猜测"，永远不变。
// f(n) = g(n) + h(n): 经过 n 的整条路径的估计总代价。OPEN 按它排序。
//
//   h ≡ 0        -> 退化成 Dijkstra（只看已走的，四面八方均匀铺开，最优但慢）
//   h 偏小       -> 更保守，扩展多，仍然最优
//   h = 真实值   -> 一条直线冲向终点，扩展数最少
//   h 偏大       -> 快，但可能不是最短路（不再"可采纳 admissible"）
//
// 「可采纳（admissible）」= h(n) 永不超过 n 到终点的真实最短代价。
// 这是 A* 保证最优解的**唯一**前提。所以启发函数不能乱选：
//   * 4 邻域（只能上下左右）-> 曼哈顿距离 dx+dy 正好是真实值，最优选择；
//   * 8 邻域（能走斜线，斜线代价 √2）-> 曼哈顿会**高估**（比如斜着走 3 格，
//     真实代价 3√2≈4.24，曼哈顿给 6），于是不可采纳，A* 会返回次优路径！
//     8 邻域该用 octile / 对角距离：√2·min(dx,dy) + |dx-dy|，那才是真实值。
// 代码里 checkHeuristicAdmissible() 会在你选错时打警告 —— 这个坑值得亲眼看一次。
#ifndef ASTAR_TUTORIAL_ASTAR_H
#define ASTAR_TUTORIAL_ASTAR_H

#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "astar_tutorial/grid_map_2d.h"

namespace astar_tutorial {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309504880;

// ---------------------------------------------------------------- 启发函数
enum class Heuristic {
  kZero = 0,   // h ≡ 0，A* 退化成 Dijkstra —— 用来做"最优性基准"
  kManhattan,  // |dx| + |dy|          4 邻域下是真实值；8 邻域下高估
  kEuclidean,  // sqrt(dx² + dy²)      永远可采纳，但在栅格上偏小（偏保守）
  kDiagonal,   // √2·min + |dx-dy|     8 邻域下是真实值（octile 距离）
};

const char* toString(Heuristic h);
bool parseHeuristic(const std::string& s, Heuristic* out);

// ---------------------------------------------------------------- 配置
struct AStarConfig {
  Heuristic heuristic = Heuristic::kDiagonal;

  // true = 8 邻域（含斜线，代价 √2）；false = 4 邻域（只有上下左右，代价 1）
  bool use_8_connected = true;

  // 8 邻域时，是否允许"贴着墙角斜穿"。
  //   . #        从 c 斜着走到 X，路径正好从两个障碍的夹缝里钻过去。
  //   c X        真实机器人有体积，钻不过去 -> 默认禁止（false）。
  bool allow_corner_cutting = false;

  // tie_breaker：f 相同的节点谁先出堆？乘一个 1+ε 到 h 上，
  // 让"离终点更近"的那个稍微占优，能显著减少空旷地图上的无效扩展。
  // 代价：h 被放大了 (1+ε) 倍，严格来说不再可采纳，但 ε 很小时误差可忽略。
  double tie_breaker = 1.0;

  // weight > 1 即 Weighted A*：f = g + w·h。
  // 扩展数大幅下降，代价是路径长度最多为最优解的 w 倍（有界次优）。
  double weight = 1.0;

  // 迭代上限，0 = 不限。防止在超大地图上跑飞。
  int max_iterations = 0;

  // 是否录制 trace（rviz 单步回放靠它）。只关心性能时置 false。
  bool record_trace = true;
};

// ---------------------------------------------------------------- 节点
enum class NodeState : uint8_t {
  kUnvisited = 0,  // 还没碰到过（g 仍是 +inf）
  kOpen = 1,       // 在 OPEN 里：已发现，但还没确定最优 g
  kClosed = 2,     // 在 CLOSED 里：g 已经确定，不会再变
};

struct Node {
  int x = 0;
  int y = 0;
  double g = kInf;  // 起点 -> 本节点 已知最短代价
  double h = 0.0;   // 本节点 -> 终点 估计代价（已含 weight·tie_breaker）
  double f = kInf;  // g + h
  int parent = -1;  // 父节点的一维下标；-1 = 无（起点，或还没被发现）
  NodeState state = NodeState::kUnvisited;
};

// ---------------------------------------------------------------- trace
// 每一步（= 一次出堆 + 对它所有邻居的处理）都记下来，用于逐步回放。
// 这是本包"能看懂每一步"的关键：算法跑完之后，过程仍然完整保留着。

// 对某个邻居做出的判决
enum class NeighborAction : uint8_t {
  kPushedNew,        // 第一次遇到 -> 算 g/h/f，入 OPEN            [伪代码 9-11]
  kUpdatedBetter,    // 已在 OPEN，这次绕过来更短 -> 改小 g，换父亲 [伪代码 9-11]
  kSkippedWorse,     // 已在 OPEN，但这次不划算 -> 丢掉             [伪代码 9 判假]
  kSkippedClosed,    // 已在 CLOSED -> 丢掉                        [伪代码 7]
  kSkippedOccupied,  // 是障碍                                     [伪代码 7]
  kSkippedOutside,   // 越出地图边界                               [伪代码 7]
  kSkippedCorner,    // 斜穿墙角，被 allow_corner_cutting=false 拦下
};

const char* toString(NeighborAction a);

struct NeighborEvent {
  int x = 0;
  int y = 0;
  int id = -1;  // 一维下标；越界时为 -1（回放器靠它更新状态数组）
  NeighborAction action = NeighborAction::kSkippedOutside;
  double edge_cost = 0.0;  // cur -> nb 的一步代价（1 或 √2）
  double g_old = kInf;     // 处理之前 nb 的 g
  double g_new = kInf;     // g[cur] + edge_cost（试探值）
  double h = 0.0;
  double f_new = kInf;
};

struct SearchStep {
  int iteration = 0;  // 第几次出堆，从 1 开始
  int cur_id = -1;    // 本步出堆的节点
  int cur_x = 0;
  int cur_y = 0;
  double cur_g = 0.0;
  double cur_h = 0.0;
  double cur_f = 0.0;
  int cur_parent = -1;
  bool is_goal = false;  // 本步弹出的就是终点 -> 搜索成功结束
  std::vector<NeighborEvent> neighbors;
  int open_size_after = 0;    // 这一步做完时 OPEN 的真实大小
  int closed_size_after = 0;  // 同上，CLOSED
};

enum class SearchResult {
  kSuccess = 0,
  kFailedNoPath,   // OPEN 抽干了还没到终点：起终点被墙彻底隔开
  kFailedMaxIter,  // 撞到 max_iterations
  kFailedBadInput, // 没设地图 / 起终点越界或落在障碍上
};

const char* toString(SearchResult r);

struct SearchStats {
  SearchResult result = SearchResult::kFailedBadInput;
  int iterations = 0;        // 出堆次数（= 被 close 的节点数）
  int expanded = 0;          // 同上，习惯叫法
  int pushes = 0;            // 入堆次数（含被更新后重复入堆的）
  int stale_pops = 0;        // 出堆后发现是过期条目而丢弃的次数（懒惰删除的开销）
  std::size_t open_peak = 0; // OPEN 的历史最大真实大小
  double path_cost = 0.0;    // 路径代价，单位=格（斜线算 √2）
  double path_length_m = 0.0;// 换成米
  int path_size = 0;         // 路径经过的格子数
  double time_ms = 0.0;
};

// ---------------------------------------------------------------- A*
class AStar {
 public:
  // 只存指针，不拷贝地图。map 的生命周期必须长于本对象。
  void setMap(const GridMap2D* map) { map_ = map; }
  void setConfig(const AStarConfig& cfg) { cfg_ = cfg; }
  const AStarConfig& config() const { return cfg_; }
  AStarConfig& mutableConfig() { return cfg_; }

  // 跑一次搜索。起终点用栅格索引给。
  SearchResult search(int sx, int sy, int gx, int gy);

  // ---- 结果 ----
  const SearchStats& stats() const { return stats_; }
  const std::vector<Node>& nodes() const { return nodes_; }
  const std::vector<SearchStep>& trace() const { return trace_; }
  int startId() const { return start_id_; }
  int goalId() const { return goal_id_; }

  // 路径：起点 -> 终点。失败时返回空。
  std::vector<int> pathIds() const;
  std::vector<std::pair<int, int>> pathGrid() const;
  std::vector<std::pair<double, double>> pathWorld() const;

  // 原始启发值（不含 weight / tie_breaker），单位=格
  double rawHeuristic(int x, int y, int gx, int gy) const;
  // 实际塞进 f 的那个 h（含 weight / tie_breaker）
  double heuristicCost(int x, int y, int gx, int gy) const;

  // 当前 heuristic + 邻域组合是否可采纳（不可采纳 => 不保证最短路）
  bool isHeuristicAdmissible() const;
  // 不可采纳时返回一句人话说明，否则返回空串
  std::string admissibilityWarning() const;

 private:
  // OPEN 的实现：二叉堆 + 懒惰删除（lazy deletion）。
  //
  // 标准 A* 需要 decrease-key（节点 g 变小时把它在堆里往上调）。
  // std::priority_queue 不支持，业界两种常见做法：
  //   A. 懒惰删除：直接再 push 一份新的 (f, id)，旧条目留在堆里。出堆时若发现
  //      该节点已 closed，或条目里的 f 比节点当前 f 大，就当垃圾丢掉。
  //      —— 本文件用这个：简单、常数小、不会写错。堆里条目数最多 O(边数)。
  //   B. std::multimap<f, ptr> + 保存迭代器，更新时 erase + insert。
  //      —— hw_1 的 grid_path_searcher/graph_searcher.cpp 用的是这个，可以对照看。
  // 两者结果完全一致，只是内存/常数不同。stats_.stale_pops 就是 A 的额外开销。
  struct QueueItem {
    double f;
    double h;  // f 相同时先扩展 h 小的（更靠近终点），纯粹为了确定性和效率
    int id;
  };
  struct QueueGreater {
    bool operator()(const QueueItem& a, const QueueItem& b) const {
      if (a.f != b.f) return a.f > b.f;  // priority_queue 是大顶堆，反过来写才是小顶堆
      if (a.h != b.h) return a.h > b.h;
      return a.id > b.id;  // 最后按 id，保证同一输入每次跑出完全相同的过程
    }
  };
  using OpenList = std::priority_queue<QueueItem, std::vector<QueueItem>, QueueGreater>;

  void resetNodes();

  const GridMap2D* map_ = nullptr;
  AStarConfig cfg_;

  std::vector<Node> nodes_;       // 大小 = 格子数，按一维下标索引
  std::vector<SearchStep> trace_;
  SearchStats stats_;
  int start_id_ = -1;
  int goal_id_ = -1;
};

}  // namespace astar_tutorial

#endif  // ASTAR_TUTORIAL_ASTAR_H
