// 手写 BFS / DFS / Dijkstra —— 三个算法，一个循环。
//
// 这个包想说明的**唯一**一件事：
//
//     BFS、DFS、Dijkstra（以及隔壁包的 A*）是同一段代码。
//     它们的区别只有一处 —— 每一轮从"待处理集合"里**取哪一个**出来。
//
// ============================ 统一伪代码 ============================
//
//   1  frontier = { (start, parent=-1, g=0, depth=0) };   state[start] = OPEN
//   2  while frontier 非空:
//   3      item = frontier.取出一个                 <- ★ 唯一的算法差别就在这一行
//   4      if state[item.id] == CLOSED:  丢掉（重复条目），continue
//   5      落定 item: state[cur]=CLOSED, 把 g/depth/parent 写进节点
//   6      if cur == goal:  沿 parent 回溯出路径，结束
//   7      for nb in cur 的邻居:
//   8          if nb 越界 / 是障碍 / 斜穿墙角:  跳过
//   9          if state[nb] == CLOSED:  跳过
//  10          g_new = g[cur] + cost(cur,nb);   d_new = depth[cur] + 1
//  11          if state[nb] == UNVISITED:  入 frontier，state[nb] = OPEN
//  12          else  /* 已经在 frontier 里了 */:  按算法规则处理
//  13  frontier 空了还没到终点 => 无解
//
// 三个算法在第 3 行和第 12 行的取值：
//
//   算法       第 3 行取哪个条目      第 12 行怎么办        保证什么
//   --------------------------------------------------------------------------
//   BFS        最早入队的  (FIFO)     跳过                  步数最少
//   DFS        最晚入栈的  (LIFO)     再入一次              什么都不保证
//   Dijkstra   g 最小的    (小顶堆)   更短就松弛并重入      代价最小
//   (A*        g+h 最小的  (小顶堆)   同 Dijkstra           代价最小 —— 见 astar_tutorial)
//
// graph_search.cpp 里每个环节都标了对应行号（[伪代码 N]），第 3/12 行的三个分支
// 各自单独成段，可以并排读。**先把这张表看懂，再去读代码。**
//
// ============================ 三个必须分清的量 ============================
//
// depth(n): 从起点到 n 的**步数**（走了几格）。每条边都算 1 步。BFS 优化这个。
// g(n)     : 从起点到 n 的**几何代价**。4 邻域下每条边都是 1；8 邻域下斜边是 √2。
//            Dijkstra 优化这个。
// h(n)     : 到终点的估计代价。本包**没有** h —— 加上它就是 A*，在隔壁包。
//
// ============================ 本包默认 4 邻域 ============================
//
// 机器人只能上下左右走，不走对角线（SearchConfig::use_8_connected 默认 false）。
// 这带来一个很舒服的性质：**每条边的代价都是 1**，于是
//
//     depth(n) == g(n)     对每个节点都成立
//
// 「步数最少」和「代价最小」变成同一件事 —— 所以 4 邻域下
// **BFS 和 Dijkstra 给出完全相同的路径和代价**，BFS 就是一个正确的最短路算法，
// 而且不用堆，O(V+E) 就够。selfcheck 会逐格断言这个等式。
//
// 那为什么还要学 Dijkstra？因为这个等式一旦边代价不全为 1 就崩：
//   - 打开 8 邻域（rviz 里按 v）：斜边代价 √2，3 步斜线 3√2≈4.243 比 4 步直线 4.000
//     **步数更少却更长**，BFS 立刻开始给出次优路径 —— 路径看起来还完全正常。
//   - 或者加上地形代价（草地 2、公路 1）：BFS 彻底没救。
// 量化推导见 doc/01-bfs.md 的 §1.4，实验证据见 search_selfcheck。
#ifndef SEARCH_TUTORIAL_GRAPH_SEARCH_H
#define SEARCH_TUTORIAL_GRAPH_SEARCH_H

#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <string>
#include <vector>

// 栅格地图直接复用 astar_tutorial 的那份，不重复实现一遍：
// 两个包是配套的，共用同一张地图才能拿 A* 去交叉验证本包的 Dijkstra
// （见 search_selfcheck.cpp）。
#include "astar_tutorial/grid_map_2d.h"

namespace search_tutorial {

using GridMap2D = astar_tutorial::GridMap2D;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309504880;

// ---------------------------------------------------------------- 算法
enum class Algorithm {
  kBfs = 0,   // 广度优先：FIFO 队列。一层一层往外推，**步数**最少
  kDfs,       // 深度优先：LIFO 栈。一头扎到底再回头，什么都不保证
  kDijkstra,  // 一致代价：按 g 排序的小顶堆。**代价**最小
};

const char* toString(Algorithm a);
// 纯 ASCII 的短名。printf 的字段宽度按**字节**算，中文名字会让对比表整体错位，
// 所以凡是要对齐的表格都用这个。
const char* shortName(Algorithm a);
bool parseAlgorithm(const std::string& s, Algorithm* out);

// ---------------------------------------------------------------- 配置
struct SearchConfig {
  Algorithm algorithm = Algorithm::kDijkstra;

  // false = **4 邻域**（只有上下左右，每条边代价都是 1）—— 本包的默认；
  // true  = 8 邻域（含斜线，斜边代价 √2）。
  //
  // 这个开关对本包**格外重要**，它决定了两个目标函数是不是同一个：
  //
  //   4 邻域：所有边代价 = 1  =>  「步数」和「几何代价」是同一个量
  //                           =>  BFS 和 Dijkstra 的结果**完全相同**，BFS 就是最短路算法
  //   8 邻域：边代价有 1 和 √2 =>  两个目标分道扬镳
  //                           =>  BFS 仍然步数最少，但路径可能比最短路更长
  //
  // 默认 4 邻域，所以默认配置下 BFS 是**正确的**最短路算法，Dijkstra 只是把同一个
  // 答案用另一种方式算出来。想亲眼看 BFS 在 8 邻域下失效，把它打开（rviz 里按 v）。
  // 推导见 doc/01-bfs.md §1.4，实测频率见 search_selfcheck。
  bool use_8_connected = false;

  // 只在 8 邻域下有意义：是否允许"贴着墙角斜穿"。
  //   . #        从 c 斜着走到 X，路径正好从两个障碍的夹缝里钻过去。
  //   c X        真实机器人有体积，钻不过去 -> 默认禁止（false）。
  // 4 邻域下压根没有斜线，这个开关不起任何作用。
  bool allow_corner_cutting = false;

  // 迭代上限，0 = 不限。DFS 在大地图上会走出极长的路径，想中途叫停时用它。
  int max_iterations = 0;

  // 是否录制 trace（rviz / 控制台的单步回放靠它）。只关心性能时置 false。
  bool record_trace = true;

  // 把"从容器里弹出一个已经 CLOSED 的重复条目"也录成一步。
  // DFS 会产生大量这种条目（同一个格子可能被压进栈好几次），
  // 录下来才能在回放里看清"懒惰去重"到底在干什么。
  // 关掉它，回放的步数就等于真正被扩展的节点数，看起来更干净。
  bool record_stale_pops = true;
};

// ---------------------------------------------------------------- 节点
enum class NodeState : uint8_t {
  kUnvisited = 0,  // 还没碰到过
  kOpen = 1,       // 在 frontier 里：已发现，还没被取出来处理
  kClosed = 2,     // 已经被取出来处理过：g/depth/parent 已落定，不再改
};

struct Node {
  int x = 0;
  int y = 0;
  double g = kInf;  // 起点 -> 本节点 的几何代价（直 1 斜 √2）
  int depth = -1;   // 起点 -> 本节点 的步数
  int parent = -1;  // 父节点的一维下标；-1 = 无（起点，或还没被发现）
  NodeState state = NodeState::kUnvisited;
};

// ---------------------------------------------------------------- frontier
// 「frontier」= 伪代码里那个待处理集合。BFS 叫它队列，DFS 叫它栈，
// Dijkstra 叫它 OPEN 表 —— 三个名字，一个东西，只是取出的规则不同。
//
// 条目自带 parent / g / depth，而不是只存一个 id。为什么？
//   DFS 允许同一个格子被压进栈**多次**，每次的父节点还不一样。
//   父节点只能记在条目上；等它真的被弹出来的那一刻，才知道该认哪个爹。
//   （BFS 和 Dijkstra 每个格子在 frontier 里最多留一份有效条目，
//     所以记在节点上也行 —— 但统一记在条目上，三个算法就能共用同一段代码。）
struct FrontierItem {
  int id = -1;
  int parent = -1;
  double g = 0.0;
  int depth = 0;
};

class Frontier {
 public:
  void reset(Algorithm a) {
    algo_ = a;
    deque_.clear();
    heap_ = Heap();
  }

  // 三个算法都是"从尾部加入"，加入这一侧没有区别。
  void push(const FrontierItem& it) {
    if (algo_ == Algorithm::kDijkstra) {
      heap_.push(it);
    } else {
      deque_.push_back(it);
    }
  }

  // ★★★ 整个包的核心就是这个函数 ★★★                              [伪代码 3]
  // 三行代码，三个算法。别的地方一个字都不用改。
  FrontierItem pop() {
    if (algo_ == Algorithm::kDijkstra) {
      const FrontierItem it = heap_.top();  // g 最小的     -> Dijkstra
      heap_.pop();
      return it;
    }
    if (algo_ == Algorithm::kBfs) {
      const FrontierItem it = deque_.front();  // 最早进来的 -> BFS（FIFO 队列）
      deque_.pop_front();
      return it;
    }
    const FrontierItem it = deque_.back();  // 最晚进来的   -> DFS（LIFO 栈）
    deque_.pop_back();
    return it;
  }

  bool empty() const {
    return algo_ == Algorithm::kDijkstra ? heap_.empty() : deque_.empty();
  }
  // 容器里的**条目**数。DFS 下会明显大于"真实 OPEN 节点数"（有重复）。
  std::size_t size() const {
    return algo_ == Algorithm::kDijkstra ? heap_.size() : deque_.size();
  }

 private:
  // Dijkstra 的小顶堆。std::priority_queue 默认是大顶堆，比较器要反着写。
  //
  // 这里没有做 decrease-key（节点 g 变小时把它在堆里往上调），用的是**懒惰删除**：
  // 松弛成功就再 push 一份新条目，旧条目留在堆里，等它出堆时发现节点已 CLOSED 再丢掉。
  // 代价是堆里条目变多（stats.stale_pops 就是这份开销），换来的是代码短、不会写错。
  // 另一种写法是 std::multimap<g, ptr> + 保存迭代器，见 hw_1 的 graph_searcher.cpp。
  struct ItemGreater {
    bool operator()(const FrontierItem& a, const FrontierItem& b) const {
      if (a.g != b.g) return a.g > b.g;
      return a.id > b.id;  // g 相同时按 id，保证同一输入每次跑出完全相同的过程
    }
  };
  using Heap = std::priority_queue<FrontierItem, std::vector<FrontierItem>, ItemGreater>;

  Algorithm algo_ = Algorithm::kDijkstra;
  std::deque<FrontierItem> deque_;  // BFS 从头取、DFS 从尾取，都往尾部塞
  Heap heap_;                       // 只有 Dijkstra 用
};

// ---------------------------------------------------------------- trace
// 每一步（= 一次出队 + 对它所有邻居的处理）都记下来，用于逐步回放。
// 这是本包"能看懂每一步"的关键：算法跑完之后，过程仍然完整保留着。

// 对某个邻居做出的判决
enum class NeighborAction : uint8_t {
  kPushedNew,       // 第一次遇到 -> 入 frontier                    [伪代码 11]
  kPushedAgain,     // 已在 frontier 里，DFS 仍然再压一次            [伪代码 12] DFS
  kUpdatedBetter,   // 已在 frontier 里，这次更短 -> 松弛+重入        [伪代码 12] Dijkstra
  kSkippedSeen,     // 已在 frontier 里 -> 跳过                     [伪代码 12] BFS
  kSkippedWorse,    // 已在 frontier 里，但这次不更短 -> 跳过         [伪代码 12] Dijkstra
  kSkippedClosed,   // 已经 CLOSED -> 跳过                          [伪代码 9]
  kSkippedOccupied, // 是障碍                                        [伪代码 8]
  kSkippedOutside,  // 越出地图边界                                  [伪代码 8]
  kSkippedCorner,   // 斜穿墙角，被 allow_corner_cutting=false 拦下
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
  int depth_new = -1;      // depth[cur] + 1
};

struct SearchStep {
  int iteration = 0;  // 第几次出队（含被丢弃的重复条目），从 1 开始
  int cur_id = -1;    // 本步弹出的条目指向的节点
  int cur_x = 0;
  int cur_y = 0;
  double cur_g = 0.0;
  int cur_depth = 0;
  int cur_parent = -1;
  bool is_goal = false;  // 本步弹出的就是终点 -> 搜索成功结束
  // 本步弹出的条目指向一个**已经 CLOSED** 的节点 -> 什么都不做，直接丢。 [伪代码 4]
  // DFS 下这种步会很多；BFS 下一次都不会有（每个格子只入队一次）。
  bool is_stale_pop = false;
  std::vector<NeighborEvent> neighbors;
  std::size_t frontier_size_after = 0;  // 容器里的条目数（含重复）
  int open_size_after = 0;              // 状态为 OPEN 的**节点**数
  int closed_size_after = 0;
};

enum class SearchResult {
  kSuccess = 0,
  kFailedNoPath,    // frontier 抽干了还没到终点：起终点被墙彻底隔开
  kFailedMaxIter,   // 撞到 max_iterations
  kFailedBadInput,  // 没设地图 / 起终点越界或落在障碍上
};

const char* toString(SearchResult r);

struct SearchStats {
  SearchResult result = SearchResult::kFailedBadInput;
  int iterations = 0;   // 有效出队次数（= 被 CLOSED 的节点数）
  int expanded = 0;     // 同上，习惯叫法
  int pushes = 0;       // 入队次数（含重复入队的）
  int stale_pops = 0;   // 弹出后发现节点已 CLOSED 而丢弃的次数
  std::size_t frontier_peak = 0;  // 容器条目数的历史峰值（DFS 的栈深看这个）
  std::size_t open_peak = 0;      // 真实 OPEN 节点数的历史峰值
  double path_cost = 0.0;         // 路径几何代价，单位=格（斜线算 √2）
  double path_length_m = 0.0;     // 换成米
  int path_steps = 0;             // 路径步数 = 经过的格子数 - 1
  int path_size = 0;              // 路径经过的格子数
  double time_ms = 0.0;
};

// ---------------------------------------------------------------- 搜索器
class GraphSearch {
 public:
  // 只存指针，不拷贝地图。map 的生命周期必须长于本对象。
  void setMap(const GridMap2D* map) { map_ = map; }
  void setConfig(const SearchConfig& cfg) { cfg_ = cfg; }
  const SearchConfig& config() const { return cfg_; }
  SearchConfig& mutableConfig() { return cfg_; }

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

  // ---- 这一次搜索"保证"了什么 ----
  // 三个算法的全部区别，最后都体现在这两个谓词上。
  //   Dijkstra : 代价最小，永远成立
  //   BFS      : 步数最少，永远成立；**代价**最小只在"所有边代价相等"时成立（4 邻域）
  //   DFS      : 两个都不保证
  bool guaranteesMinCost() const;
  bool guaranteesMinSteps() const;
  // 一句人话，说明当前配置下这次搜索到底能信什么
  std::string optimalityNote() const;

 private:
  void resetNodes();

  const GridMap2D* map_ = nullptr;
  SearchConfig cfg_;

  std::vector<Node> nodes_;  // 大小 = 格子数，按一维下标索引
  std::vector<SearchStep> trace_;
  SearchStats stats_;
  int start_id_ = -1;
  int goal_id_ = -1;
};

}  // namespace search_tutorial

#endif  // SEARCH_TUTORIAL_GRAPH_SEARCH_H
