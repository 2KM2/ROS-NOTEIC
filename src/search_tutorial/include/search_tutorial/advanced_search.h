#ifndef SEARCH_TUTORIAL_ADVANCED_SEARCH_H
#define SEARCH_TUTORIAL_ADVANCED_SEARCH_H

#include <cstddef>
#include <queue>
#include <utility>
#include <vector>

#include "search_tutorial/graph_search.h"

namespace search_tutorial {

struct AdvancedSearchStats {
  SearchResult result = SearchResult::kFailedBadInput;
  int expanded = 0;
  int queue_pops = 0;
  int pushes = 0;
  int path_steps = 0;
  double path_cost = 0.0;
  double time_ms = 0.0;
};

// D* Lite 从终点反向维护 g/rhs。地图局部变化时只修复不一致节点，
// 不需要像 Dijkstra/A* 那样把整张图重新搜一遍。
class DStarLite {
 public:
  void setMap(GridMap2D* map) { map_ = map; }
  void setConnectivity(bool use_8_connected, bool allow_corner_cutting = false);

  SearchResult initialize(int sx, int sy, int gx, int gy);
  SearchResult replan();
  SearchResult moveStart(int sx, int sy);
  SearchResult updateCell(int x, int y, bool occupied);

  const AdvancedSearchStats& stats() const { return stats_; }
  std::vector<int> pathIds() const;
  std::vector<std::pair<int, int>> pathGrid() const;
  int startId() const { return start_id_; }
  int goalId() const { return goal_id_; }
  const std::vector<double>& gValues() const { return g_; }
  const std::vector<double>& rhsValues() const { return rhs_; }

 private:
  struct Key {
    double first = kInf;
    double second = kInf;
  };
  struct QueueItem {
    int id = -1;
    Key key;
    std::size_t version = 0;
  };
  struct QueueGreater {
    bool operator()(const QueueItem& a, const QueueItem& b) const;
  };

  bool validEndpoint(int id) const;
  double heuristic(int a, int b) const;
  double edgeCost(int from, int to) const;
  std::vector<int> neighbors(int id) const;
  Key calculateKey(int id) const;
  void push(int id);
  void updateVertex(int id);
  SearchResult computeShortestPath();
  void finalizeStats(double elapsed_ms);

  GridMap2D* map_ = nullptr;
  bool use_8_connected_ = false;
  bool allow_corner_cutting_ = false;
  int start_id_ = -1;
  int goal_id_ = -1;
  int last_start_id_ = -1;
  double km_ = 0.0;
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<std::size_t> queue_version_;
  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueGreater> open_;
  AdvancedSearchStats stats_;
};

// 经典 JPS：在 8 邻域、均匀栅格代价、允许对角穿角的模型上，
// 跳过直线/对角线上的对称中间节点，只把跳点放进 OPEN。
class JumpPointSearch {
 public:
  void setMap(const GridMap2D* map) { map_ = map; }
  SearchResult search(int sx, int sy, int gx, int gy);

  const AdvancedSearchStats& stats() const { return stats_; }
  std::vector<int> jumpPointIds() const;
  std::vector<int> pathIds() const;
  std::vector<std::pair<int, int>> pathGrid() const;

 private:
  struct OpenItem {
    int id = -1;
    double f = kInf;
    double g = kInf;
  };
  struct OpenGreater {
    bool operator()(const OpenItem& a, const OpenItem& b) const;
  };

  bool walkable(int x, int y) const;
  double heuristic(int id) const;
  int jump(int x, int y, int dx, int dy) const;
  bool hasForcedNeighbor(int x, int y, int dx, int dy) const;
  std::vector<std::pair<int, int>> prunedDirections(int id) const;
  double segmentCost(int a, int b) const;

  const GridMap2D* map_ = nullptr;
  int start_id_ = -1;
  int goal_id_ = -1;
  std::vector<double> g_;
  std::vector<int> parent_;
  std::vector<char> closed_;
  AdvancedSearchStats stats_;
};

}  // namespace search_tutorial

#endif  // SEARCH_TUTORIAL_ADVANCED_SEARCH_H
