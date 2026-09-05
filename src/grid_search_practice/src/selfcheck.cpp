// 检查地图生成和辅助函数，不检查四个算法（那是你要写的）。
#include "grid_search_practice/search.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

using grid_search_practice::Cell;
using grid_search_practice::GridMap2D;
using grid_search_practice::SearchOptions;
using grid_search_practice::countOccupied;
using grid_search_practice::edgeCost;
using grid_search_practice::heuristic;
using grid_search_practice::kSqrt2;
using grid_search_practice::makePracticeMap;
using grid_search_practice::neighbors;

namespace {

int g_fails = 0;

void expect(bool ok, const char* msg) {
  if (ok) {
    std::printf("  OK  %s\n", msg);
  } else {
    std::printf("  FAIL  %s\n", msg);
    ++g_fails;
  }
}

bool approx(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

}  // namespace

int main() {
  std::printf("=== grid_search_practice selfcheck (helpers only) ===\n");

  GridMap2D map;
  const Cell start{0, 0};
  const Cell goal{39, 39};
  const bool made = makePracticeMap(&map, 40, 40, 1.0, 45, 2026, start, goal);
  expect(made, "makePracticeMap 成功");
  expect(map.width() == 40 && map.height() == 40, "地图 40x40");
  expect(std::fabs(map.resolution() - 1.0) < 1e-12, "分辨率 1m");
  expect(countOccupied(map) == 45, "恰好 45 个障碍格");
  expect(map.isFree(0, 0) && map.isFree(39, 39), "起终点空闲");
  expect(map.isConnected(0, 0, 39, 39, false, false), "4 连通");

  GridMap2D empty(10, 10, 1.0, 0.0, 0.0);
  SearchOptions four;
  SearchOptions eight;
  eight.use_8_connected = true;
  eight.allow_corner_cutting = false;
  expect(neighbors(empty, 5, 5, four).size() == 4, "空地图中心 4 邻域 = 4");
  expect(neighbors(empty, 5, 5, eight).size() == 8, "空地图中心 8 邻域 = 8");
  expect(neighbors(empty, 0, 0, four).size() == 2, "角落 4 邻域 = 2");

  expect(approx(edgeCost(0, 0, 1, 0), 1.0), "直角边代价 1");
  expect(approx(edgeCost(0, 0, 1, 1), kSqrt2), "斜边代价 sqrt(2)");
  expect(approx(heuristic(0, 0, 3, 4, false), 7.0), "曼哈顿 (0,0)->(3,4) = 7");
  // octile: dx=3, dy=4 → min=3 max=4 → (4-3) + sqrt2*3
  expect(approx(heuristic(0, 0, 3, 4, true), 1.0 + kSqrt2 * 3.0),
         "octile (0,0)->(3,4) = 1 + 3*sqrt(2)");

  GridMap2D corner(3, 3, 1.0, 0.0, 0.0);
  corner.setOccupied(1, 0, true);
  corner.setOccupied(0, 1, true);
  SearchOptions no_cut;
  no_cut.use_8_connected = true;
  no_cut.allow_corner_cutting = false;
  SearchOptions yes_cut = no_cut;
  yes_cut.allow_corner_cutting = true;
  expect(neighbors(corner, 0, 0, no_cut).empty(), "禁止穿墙角时 (0,0) 不能斜到 (1,1)");
  const auto with_cut = neighbors(corner, 0, 0, yes_cut);
  bool has_diag = false;
  for (const auto& c : with_cut) {
    if (c.x == 1 && c.y == 1) has_diag = true;
  }
  expect(has_diag, "允许穿墙角时 (0,0) 可以斜到 (1,1)");

  if (g_fails == 0) {
    std::printf("全部通过\n");
    return 0;
  }
  std::printf("%d 项失败\n", g_fails);
  return 1;
}
