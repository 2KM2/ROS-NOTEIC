// 自检程序 —— 用 Dijkstra 交叉验证手写 A* 的正确性。
//
// 为什么需要它？因为"路径看起来对"骗不了人，但"路径是最短的"肉眼看不出来。
// 而 A* 最容易写错的地方恰恰不影响"能到终点"，只影响"是不是最短"：
//   * 终点一入 OPEN 就返回（而不是等它出堆）
//   * CLOSED 里的节点也去更新
//   * 8 邻域配曼哈顿启发
//   * 斜线代价写成 1 而不是 √2
// 这些错误都会让路径变长一点点，只有和 Dijkstra 比数值才能抓出来。
//
// 检查项：
//   1. h≡0 的 A* 就是 Dijkstra，其代价视作真值；
//   2. 所有**可采纳**的启发函数必须给出和真值完全相同的代价；
//   3. 不可采纳的组合（8邻域+曼哈顿）代价 >= 真值，且**允许**更大 —— 顺便统计它错了多少次；
//   4. 路径本身合法：首尾正确、相邻格真的相邻、不穿障碍、代价累加对得上；
//   5. 无解的情况两者必须都判无解。
//
// 编译（不需要 ROS）：
//   g++ -std=c++17 -O2 -I include src/grid_map_2d.cpp src/astar.cpp
//       src/astar_selfcheck.cpp -o /tmp/astar_selfcheck && /tmp/astar_selfcheck
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "astar_tutorial/astar.h"
#include "astar_tutorial/grid_map_2d.h"

using namespace astar_tutorial;

namespace {

int g_failures = 0;

void fail(const std::string& what) {
  std::printf("  [FAIL] %s\n", what.c_str());
  ++g_failures;
}

// 校验路径本身是否自洽，并返回按几何算出来的代价
bool validatePath(const GridMap2D& map, const AStar& a, int sx, int sy, int gx, int gy,
                  bool conn8, bool corner_cut, double* geo_cost) {
  const auto path = a.pathGrid();
  *geo_cost = 0.0;
  if (path.empty()) return fail("成功了但路径是空的"), false;
  if (path.front().first != sx || path.front().second != sy) return fail("路径没从起点开始"), false;
  if (path.back().first != gx || path.back().second != gy) return fail("路径没到终点"), false;

  for (std::size_t i = 0; i < path.size(); ++i) {
    if (map.isOccupied(path[i].first, path[i].second)) return fail("路径穿过障碍"), false;
    if (i == 0) continue;
    const int dx = std::abs(path[i].first - path[i - 1].first);
    const int dy = std::abs(path[i].second - path[i - 1].second);
    if (dx > 1 || dy > 1 || (dx == 0 && dy == 0)) return fail("路径相邻两格不相邻(父指针断了)"), false;
    const bool diag = (dx == 1 && dy == 1);
    if (diag && !conn8) return fail("4邻域却走了斜线"), false;
    if (diag && !corner_cut) {
      // 禁止斜穿墙角时，路径上不该出现"两个正交邻居有障碍"的斜步
      if (map.isOccupied(path[i].first, path[i - 1].second) ||
          map.isOccupied(path[i - 1].first, path[i].second)) {
        return fail("禁止斜穿墙角，但路径斜穿了"), false;
      }
    }
    *geo_cost += diag ? kSqrt2 : 1.0;
  }
  if (std::abs(*geo_cost - a.stats().path_cost) > 1e-6) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "路径几何代价 %.6f 和 g[goal]=%.6f 不一致", *geo_cost,
                  a.stats().path_cost);
    return fail(buf), false;
  }
  return true;
}

double runOnce(GridMap2D& map, const AStarConfig& cfg, int sx, int sy, int gx, int gy,
               SearchResult* out_result) {
  AStar a;
  a.setMap(&map);
  a.setConfig(cfg);
  *out_result = a.search(sx, sy, gx, gy);
  if (*out_result != SearchResult::kSuccess) return -1.0;
  double geo = 0.0;
  validatePath(map, a, sx, sy, gx, gy, cfg.use_8_connected, cfg.allow_corner_cutting, &geo);
  return a.stats().path_cost;
}

}  // namespace

int main(int argc, char** argv) {
  const int num_maps = (argc > 1) ? std::atoi(argv[1]) : 300;
  std::printf("=== A* 自检：%d 张随机地图，每张 4/8 邻域 × 5 种启发配置 ===\n", num_maps);

  std::mt19937 rng(20260904);
  int solved = 0;
  int unsolvable = 0;
  int inadmissible_suboptimal = 0;
  int inadmissible_total = 0;
  double worst_ratio = 1.0;

  for (int t = 0; t < num_maps; ++t) {
    std::uniform_int_distribution<int> dim(8, 45);
    std::uniform_real_distribution<double> ratio(0.05, 0.42);
    const int w = dim(rng);
    const int h = dim(rng);

    GridMap2D map(w, h, 0.5, 0.0, 0.0);
    // 混着来：一半纯随机撒点，一半带墙（后者更容易造出"必须绕远路"的局面）
    if (t % 2 == 0) {
      map.fillRandomObstacles(ratio(rng), rng(), 1 + (t % 3));
    } else {
      map.fillWalls(1 + t % 5, rng(), t % 3);
      map.fillRandomObstacles(ratio(rng) * 0.5, rng(), 1);
    }

    std::uniform_int_distribution<int> px(0, w - 1);
    std::uniform_int_distribution<int> py(0, h - 1);
    int sx = px(rng), sy = py(rng), gx = px(rng), gy = py(rng);
    if (map.isOccupied(sx, sy) || map.isOccupied(gx, gy)) continue;  // 起终点压在障碍上，跳过
    if (sx == gx && sy == gy) continue;

    for (const bool conn8 : {false, true}) {
      for (const bool corner_cut : {false, true}) {
        if (!conn8 && corner_cut) continue;  // 4 邻域没有斜线，这个开关无意义

        // ---- 1. 基准：h≡0 的 A* = Dijkstra，它的代价就是真值 ----
        AStarConfig base;
        base.heuristic = Heuristic::kZero;
        base.use_8_connected = conn8;
        base.allow_corner_cutting = corner_cut;
        base.record_trace = false;
        SearchResult base_r;
        const double truth = runOnce(map, base, sx, sy, gx, gy, &base_r);

        if (base_r == SearchResult::kFailedNoPath) ++unsolvable;
        else if (base_r == SearchResult::kSuccess) ++solved;

        // ---- 2. 各种启发配置 ----
        struct Case { Heuristic h; double w; double tie; };
        const std::vector<Case> cases = {
            {Heuristic::kEuclidean, 1.0, 1.0},
            {Heuristic::kManhattan, 1.0, 1.0},
            {Heuristic::kDiagonal, 1.0, 1.0},
            {Heuristic::kDiagonal, 1.0, 1.0001},
            {Heuristic::kDiagonal, 2.5, 1.0},
        };
        for (const Case& c : cases) {
          AStarConfig cfg = base;
          cfg.heuristic = c.h;
          cfg.weight = c.w;
          cfg.tie_breaker = c.tie;
          SearchResult r;
          const double cost = runOnce(map, cfg, sx, sy, gx, gy, &r);

          // 2a. 有解/无解的判断必须一致 —— 启发函数只影响搜索顺序，不影响连通性
          if ((r == SearchResult::kSuccess) != (base_r == SearchResult::kSuccess)) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "map#%d %d邻域 h=%s: 可解性和 Dijkstra 不一致 (%s vs %s)", t,
                          conn8 ? 8 : 4, toString(c.h), toString(r), toString(base_r));
            fail(buf);
            continue;
          }
          if (r != SearchResult::kSuccess) continue;

          AStar probe;  // 只为了问一句"这组配置可采纳吗"
          probe.setConfig(cfg);
          const bool admissible = probe.isHeuristicAdmissible();

          if (admissible) {
            // 2b. 可采纳 => 必须和真值**完全相等**。这是 A* 最优性的核心断言。
            if (std::abs(cost - truth) > 1e-6) {
              char buf[256];
              std::snprintf(buf, sizeof(buf),
                            "map#%d(%dx%d) %d邻域 corner_cut=%d h=%s w=%.2f tie=%.5f: "
                            "代价 %.6f != Dijkstra 真值 %.6f  (起(%d,%d)->终(%d,%d))",
                            t, w, h, conn8 ? 8 : 4, corner_cut ? 1 : 0, toString(c.h), c.w, c.tie,
                            cost, truth, sx, sy, gx, gy);
              fail(buf);
            }
          } else {
            // 2c. 不可采纳 => 只保证不比真值短（短了就说明真值算错了）
            ++inadmissible_total;
            if (cost < truth - 1e-6) {
              char buf[256];
              std::snprintf(buf, sizeof(buf), "map#%d: 代价 %.6f 比 Dijkstra 真值 %.6f 还小?!", t,
                            cost, truth);
              fail(buf);
            } else if (cost > truth + 1e-6) {
              ++inadmissible_suboptimal;
              if (truth > 1e-9) worst_ratio = std::max(worst_ratio, cost / truth);
            }
          }
        }
      }
    }
  }

  std::printf("\n统计:\n");
  std::printf("  有解的搜索      : %d 次\n", solved);
  std::printf("  无解的搜索      : %d 次 (两种算法都判无解)\n", unsolvable);
  std::printf("  不可采纳的配置  : 跑了 %d 次, 其中 %d 次真的给出了比最优解更长的路径"
              " (最差 %.4f 倍)\n",
              inadmissible_total, inadmissible_suboptimal, worst_ratio);
  std::printf("     ^ 这一行不是 bug: 它正是\"h 高估 => 丢掉最优性\"的实验证据。\n");

  if (g_failures == 0) {
    std::printf("\n全部通过：可采纳的启发函数下，手写 A* 的代价和 Dijkstra 逐位相同。\n");
    return 0;
  }
  std::printf("\n失败 %d 项。\n", g_failures);
  return 1;
}
