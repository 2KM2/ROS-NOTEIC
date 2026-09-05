// 40x40 栅格路径搜索练习。
//
// 你要写的只有四个函数：bfs / dfs / dijkstra / astar。
// 地图、邻居、边代价、启发函数、RViz 回放都已经写好。
//
// 每个函数都要填进 SearchOutput：
//   steps  —— 每扩展一格记一步，RViz 靠它单步回放
//   path   —— 起点到终点的格子序列
//   success / expanded / pushes / path_cost / path_steps
#ifndef GRID_SEARCH_PRACTICE_SEARCH_H
#define GRID_SEARCH_PRACTICE_SEARCH_H

#include <limits>
#include <string>
#include <vector>

#include "astar_tutorial/grid_map_2d.h"

namespace grid_search_practice {

using GridMap2D = astar_tutorial::GridMap2D;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309504880;

struct Cell {
  int x = 0;
  int y = 0;
};

inline bool operator==(const Cell& a, const Cell& b) { return a.x == b.x && a.y == b.y; }

enum class Algorithm {
  kBfs = 0,
  kDfs,
  kDijkstra,
  kAstar,
};

const char* toString(Algorithm a);
const char* shortName(Algorithm a);
bool parseAlgorithm(const std::string& s, Algorithm* out);

struct SearchOptions {
  // false = 4 邻域（上下左右，边代价全是 1）—— 默认，先把基础算法学扎实
  // true  = 8 邻域（含斜线，斜边代价 √2）
  bool use_8_connected = false;
  // 只在 8 邻域有意义：禁止贴着两个障碍的夹缝斜穿
  bool allow_corner_cutting = false;
};

// 搜索的一步：RViz 用它画出当前格 / OPEN / CLOSED
struct SearchStep {
  Cell current;
  double current_g = 0.0;
  int current_depth = 0;
  std::vector<Cell> open;       // 本步之后还在 frontier 里的格子
  std::vector<Cell> closed;     // 本步之后已经扩展过的格子
  std::vector<Cell> neighbors;  // 本步检查过的邻居（可选）
};

struct SearchOutput {
  bool success = false;
  std::vector<Cell> path;         // 起点 -> 终点
  std::vector<SearchStep> steps;  // 每扩展一格一步
  int expanded = 0;               // 真正取出并处理的节点数
  int pushes = 0;                 // 入队/入栈/入堆次数
  double path_cost = 0.0;         // 几何代价（格）
  int path_steps = 0;             // 走了几步 = path.size()-1
};

// -------------------- 辅助函数（已经实现，直接调用） --------------------

// 可走邻居。越界、障碍、斜穿墙角都会被滤掉。
// 方向顺序固定：先 4 个直角，再 4 个斜向。
std::vector<Cell> neighbors(const GridMap2D& map, int x, int y, const SearchOptions& opt);

// 直角边 = 1，斜边 = √2
double edgeCost(int x0, int y0, int x1, int y1);

// 4 邻域：曼哈顿；8 邻域：octile。保证不超过真实最短路（可采纳）
double heuristic(int x, int y, int gx, int gy, bool use_8_connected);

// parent[id] = 父节点一维下标，起点的 parent 为 -1。失败返回空。
std::vector<Cell> reconstructPath(const std::vector<int>& parent, const GridMap2D& map, int start_id,
                                  int goal_id);

double pathCost(const GridMap2D& map, const std::vector<Cell>& path);

int countOccupied(const GridMap2D& map);

// 造一张练习地图：width x height，分辨率 resolution 米/格，恰好 num_obstacles 个单格障碍。
// 起终点及其 4 邻格不放障碍；4 连通失败会换种子重试。
bool makePracticeMap(GridMap2D* map, int width, int height, double resolution, int num_obstacles,
                     unsigned seed, Cell start, Cell goal);

SearchOutput run(Algorithm a, const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt);

// -------------------- 四个算法：写在 src/your_search.cpp --------------------

SearchOutput bfs(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt);
SearchOutput dfs(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt);
SearchOutput dijkstra(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt);
SearchOutput astar(const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt);

}  // namespace grid_search_practice

#endif  // GRID_SEARCH_PRACTICE_SEARCH_H
