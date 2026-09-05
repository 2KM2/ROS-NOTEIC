#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

#include "search_tutorial/advanced_search.h"

using namespace search_tutorial;

namespace {
int failures = 0;
int checks = 0;

void check(bool ok, const std::string& text) {
  ++checks;
  if (ok) return;
  ++failures;
  std::printf("[FAIL] %s\n", text.c_str());
}

SearchResult runDijkstra(const GridMap2D& map, int sx, int sy, int gx, int gy,
                         bool corner_cut, double* cost) {
  GraphSearch search;
  SearchConfig cfg;
  cfg.algorithm = Algorithm::kDijkstra;
  cfg.use_8_connected = true;
  cfg.allow_corner_cutting = corner_cut;
  cfg.record_trace = false;
  search.setMap(&map);
  search.setConfig(cfg);
  const SearchResult result = search.search(sx, sy, gx, gy);
  *cost = search.stats().path_cost;
  return result;
}

void deterministicCases() {
  GridMap2D map(20, 14, 1.0, 0.0, 0.0);
  map.fillWalls(3, 17, 1);
  const int sx = 1, sy = 1, gx = 18, gy = 12;
  double reference = 0.0;

  DStarLite dstar;
  dstar.setMap(&map);
  dstar.setConnectivity(true, false);
  check(dstar.initialize(sx, sy, gx, gy) == SearchResult::kSuccess, "D* Lite 首次规划成功");
  check(runDijkstra(map, sx, sy, gx, gy, false, &reference) == SearchResult::kSuccess,
        "Dijkstra 基准成功");
  check(std::fabs(dstar.stats().path_cost - reference) < 1e-6, "D* Lite 首次代价最优");

  const std::vector<int> before = dstar.pathIds();
  int bx = 0, by = 0;
  map.toXY(before[before.size() / 2], &bx, &by);
  check(dstar.updateCell(bx, by, true) == SearchResult::kSuccess, "D* Lite 加障碍后仍可重规划");
  check(runDijkstra(map, sx, sy, gx, gy, false, &reference) == SearchResult::kSuccess,
        "变化后 Dijkstra 基准成功");
  check(std::fabs(dstar.stats().path_cost - reference) < 1e-6, "D* Lite 修复后代价最优");

  JumpPointSearch jps;
  jps.setMap(&map);
  check(jps.search(sx, sy, gx, gy) == SearchResult::kSuccess, "JPS 搜索成功");
  check(runDijkstra(map, sx, sy, gx, gy, true, &reference) == SearchResult::kSuccess,
        "JPS 对应运动模型的 Dijkstra 成功");
  check(std::fabs(jps.stats().path_cost - reference) < 1e-6, "JPS 代价最优");
}

void randomCases(int count) {
  std::mt19937 rng(20260905);
  std::uniform_int_distribution<int> coord(0, 23);
  for (int i = 0; i < count; ++i) {
    GridMap2D map(24, 24, 1.0, 0.0, 0.0);
    map.fillRandomObstacles(0.18, static_cast<unsigned>(1000 + i), 1);
    const int sx = coord(rng), sy = coord(rng), gx = coord(rng), gy = coord(rng);
    map.setOccupied(sx, sy, false);
    map.setOccupied(gx, gy, false);

    double ref = 0.0;
    const SearchResult expected_dstar = runDijkstra(map, sx, sy, gx, gy, false, &ref);
    DStarLite dstar;
    dstar.setMap(&map);
    dstar.setConnectivity(true, false);
    const SearchResult actual_dstar = dstar.initialize(sx, sy, gx, gy);
    check(actual_dstar == expected_dstar, "随机图 D* Lite 与 Dijkstra 可达性一致");
    if (actual_dstar == SearchResult::kSuccess) {
      check(std::fabs(dstar.stats().path_cost - ref) < 1e-6,
            "随机图 D* Lite 与 Dijkstra 代价一致");
      const std::vector<int> old_path = dstar.pathIds();
      if (old_path.size() > 2) {
        int bx = 0, by = 0;
        map.toXY(old_path[old_path.size() / 2], &bx, &by);
        const SearchResult repaired = dstar.updateCell(bx, by, true);
        const SearchResult changed_reference = runDijkstra(map, sx, sy, gx, gy, false, &ref);
        check(repaired == changed_reference, "随机障碍变化后 D* Lite 与 Dijkstra 可达性一致");
        if (repaired == SearchResult::kSuccess) {
          check(std::fabs(dstar.stats().path_cost - ref) < 1e-6,
                "随机障碍变化后 D* Lite 与 Dijkstra 代价一致");
        }
      }
    }

    const SearchResult expected_jps = runDijkstra(map, sx, sy, gx, gy, true, &ref);
    JumpPointSearch jps;
    jps.setMap(&map);
    const SearchResult actual_jps = jps.search(sx, sy, gx, gy);
    check(actual_jps == expected_jps, "随机图 JPS 与 Dijkstra 可达性一致");
    if (actual_jps == SearchResult::kSuccess) {
      check(std::fabs(jps.stats().path_cost - ref) < 1e-6,
            "随机图 JPS 与 Dijkstra 代价一致");
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
  const int random_count = argc > 1 ? std::max(0, std::atoi(argv[1])) : 300;
  deterministicCases();
  randomCases(random_count);
  std::printf("advanced_search_selfcheck: %d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
