#include <cstdio>
#include <vector>

#include "search_tutorial/advanced_search.h"

using namespace search_tutorial;

namespace {
GridMap2D makeMap() {
  GridMap2D map;
  map.loadFromAscii({"....................", "....#.........#.....",
                     "....#.........#.....", "....#.........#.....",
                     "....#...............", "....#.........#.....",
                     "..............#.....", "....#.........#.....",
                     "....#...............", "...................."},
                    1.0, 0.0, 0.0);
  return map;
}

void printPath(const GridMap2D& map, const std::vector<int>& path,
               const std::vector<int>& highlights = {}) {
  std::vector<char> overlay(map.numCells(), '\0');
  for (const int id : path) overlay[id] = '*';
  for (const int id : highlights) overlay[id] = 'J';
  if (!path.empty()) { overlay[path.front()] = 'S'; overlay[path.back()] = 'G'; }
  std::printf("%s", map.toAscii(overlay).c_str());
}

void printStats(const char* name, const AdvancedSearchStats& s) {
  std::printf("%-16s result=%s expanded=%d pops=%d pushes=%d steps=%d cost=%.4f %.3f ms\n",
              name, toString(s.result), s.expanded, s.queue_pops, s.pushes,
              s.path_steps, s.path_cost, s.time_ms);
}
}  // namespace

int main() {
  GridMap2D map = makeMap();
  const int sx = 1, sy = 1, gx = map.width() - 2, gy = map.height() - 2;
  std::printf("=== D* Lite: 首次规划 ===\n");
  DStarLite dstar;
  dstar.setMap(&map);
  dstar.setConnectivity(true, false);
  if (dstar.initialize(sx, sy, gx, gy) != SearchResult::kSuccess) return 1;
  const std::vector<int> first = dstar.pathIds();
  printPath(map, first);
  printStats("D* Lite initial", dstar.stats());

  const int blocked_id = first[first.size() / 2];
  int bx = 0, by = 0;
  map.toXY(blocked_id, &bx, &by);
  std::printf("\n=== D* Lite: 在当前路径 (%d,%d) 新增障碍后增量修复 ===\n", bx, by);
  if (dstar.updateCell(bx, by, true) != SearchResult::kSuccess) return 1;
  printPath(map, dstar.pathIds());
  printStats("D* Lite repair", dstar.stats());

  std::printf("\n=== JPS: 跳点剪枝（经典 8 邻域模型，允许对角穿角）===\n");
  JumpPointSearch jps;
  jps.setMap(&map);
  if (jps.search(sx, sy, gx, gy) != SearchResult::kSuccess) return 1;
  printPath(map, jps.pathIds(), jps.jumpPointIds());
  printStats("JPS", jps.stats());
  std::printf("图例: S=起点 G=终点 *=完整路径 J=跳点 #=障碍\n");
  return 0;
}
