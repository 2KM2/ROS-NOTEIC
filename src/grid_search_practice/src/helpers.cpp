#include "grid_search_practice/search.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

namespace grid_search_practice {

namespace {

constexpr int kDX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

bool protectedCell(int x, int y, Cell start, Cell goal) {
  if (x == start.x && y == start.y) return true;
  if (x == goal.x && y == goal.y) return true;
  for (int i = 0; i < 4; ++i) {
    if (x == start.x + kDX[i] && y == start.y + kDY[i]) return true;
    if (x == goal.x + kDX[i] && y == goal.y + kDY[i]) return true;
  }
  return false;
}

}  // namespace

const char* toString(Algorithm a) {
  switch (a) {
    case Algorithm::kBfs:
      return "BFS 广度优先";
    case Algorithm::kDfs:
      return "DFS 深度优先";
    case Algorithm::kDijkstra:
      return "Dijkstra 一致代价";
    case Algorithm::kAstar:
      return "A*";
  }
  return "?";
}

const char* shortName(Algorithm a) {
  switch (a) {
    case Algorithm::kBfs:
      return "BFS";
    case Algorithm::kDfs:
      return "DFS";
    case Algorithm::kDijkstra:
      return "Dijkstra";
    case Algorithm::kAstar:
      return "A*";
  }
  return "?";
}

bool parseAlgorithm(const std::string& s, Algorithm* out) {
  if (s == "bfs" || s == "BFS") {
    *out = Algorithm::kBfs;
    return true;
  }
  if (s == "dfs" || s == "DFS") {
    *out = Algorithm::kDfs;
    return true;
  }
  if (s == "dijkstra" || s == "Dijkstra" || s == "ucs") {
    *out = Algorithm::kDijkstra;
    return true;
  }
  if (s == "astar" || s == "a*" || s == "A*" || s == "Astar") {
    *out = Algorithm::kAstar;
    return true;
  }
  return false;
}

std::vector<Cell> neighbors(const GridMap2D& map, int x, int y, const SearchOptions& opt) {
  std::vector<Cell> out;
  const int n = opt.use_8_connected ? 8 : 4;
  out.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const int nx = x + kDX[i];
    const int ny = y + kDY[i];
    if (!map.inside(nx, ny) || map.isOccupied(nx, ny)) continue;
    const bool diagonal = (kDX[i] != 0 && kDY[i] != 0);
    if (diagonal && !opt.allow_corner_cutting) {
      if (map.isOccupied(x + kDX[i], y) || map.isOccupied(x, y + kDY[i])) continue;
    }
    out.push_back({nx, ny});
  }
  return out;
}

double edgeCost(int x0, int y0, int x1, int y1) {
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  if (dx + dy == 0) return 0.0;
  if (dx == 1 && dy == 1) return kSqrt2;
  return 1.0;
}

double heuristic(int x, int y, int gx, int gy, bool use_8_connected) {
  const int dx = std::abs(gx - x);
  const int dy = std::abs(gy - y);
  if (!use_8_connected) return static_cast<double>(dx + dy);
  const int m = std::min(dx, dy);
  const int M = std::max(dx, dy);
  return static_cast<double>(M - m) + kSqrt2 * static_cast<double>(m);
}

std::vector<Cell> reconstructPath(const std::vector<int>& parent, const GridMap2D& map, int start_id,
                                  int goal_id) {
  std::vector<Cell> path;
  if (goal_id < 0 || goal_id >= static_cast<int>(parent.size())) return path;
  std::vector<char> seen(parent.size(), 0);
  int id = goal_id;
  while (id >= 0) {
    if (id >= static_cast<int>(parent.size()) || seen[static_cast<std::size_t>(id)]) {
      return {};
    }
    seen[static_cast<std::size_t>(id)] = 1;
    int x = 0;
    int y = 0;
    map.toXY(id, &x, &y);
    path.push_back({x, y});
    if (id == start_id) break;
    id = parent[static_cast<std::size_t>(id)];
  }
  if (path.empty() || map.toId(path.back().x, path.back().y) != start_id) return {};
  std::reverse(path.begin(), path.end());
  return path;
}

double pathCost(const GridMap2D& /*map*/, const std::vector<Cell>& path) {
  if (path.size() < 2) return 0.0;
  double c = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    c += edgeCost(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
  }
  return c;
}

int countOccupied(const GridMap2D& map) {
  int n = 0;
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.isOccupied(x, y)) ++n;
    }
  }
  return n;
}

bool makePracticeMap(GridMap2D* map, int width, int height, double resolution, int num_obstacles,
                     unsigned seed, Cell start, Cell goal) {
  if (map == nullptr || width <= 0 || height <= 0 || num_obstacles < 0) return false;

  constexpr int kMaxAttempts = 40;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    map->resize(width, height, resolution, 0.0, 0.0);
    if (!map->inside(start.x, start.y) || !map->inside(goal.x, goal.y)) return false;

    std::vector<Cell> candidates;
    candidates.reserve(static_cast<std::size_t>(width * height));
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (protectedCell(x, y, start, goal)) continue;
        candidates.push_back({x, y});
      }
    }
    if (static_cast<int>(candidates.size()) < num_obstacles) return false;

    std::mt19937 rng(seed + static_cast<unsigned>(attempt));
    std::shuffle(candidates.begin(), candidates.end(), rng);
    for (int i = 0; i < num_obstacles; ++i) {
      map->setOccupied(candidates[static_cast<std::size_t>(i)].x, candidates[static_cast<std::size_t>(i)].y,
                       true);
    }

    if (map->isFree(start.x, start.y) && map->isFree(goal.x, goal.y) &&
        map->isConnected(start.x, start.y, goal.x, goal.y, /*use_8_connected=*/false,
                         /*allow_corner_cutting=*/false)) {
      return true;
    }
  }
  return false;
}

SearchOutput run(Algorithm a, const GridMap2D& map, Cell start, Cell goal, const SearchOptions& opt) {
  switch (a) {
    case Algorithm::kBfs:
      return bfs(map, start, goal, opt);
    case Algorithm::kDfs:
      return dfs(map, start, goal, opt);
    case Algorithm::kDijkstra:
      return dijkstra(map, start, goal, opt);
    case Algorithm::kAstar:
      return astar(map, start, goal, opt);
  }
  return {};
}

}  // namespace grid_search_practice
