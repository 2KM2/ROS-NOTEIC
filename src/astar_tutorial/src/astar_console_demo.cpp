// A* 控制台演示 —— 不依赖 ROS，直接在终端里逐步打印。
//
// 想真正"看懂每一步"，先在这里跑。它把每一次出堆、每个邻居的判决、
// 以及当时的地图状态全都打出来，比 rviz 更适合抠细节。
//
// 编译（不需要 catkin / roscore）：
//   g++ -std=c++17 -O2 -I include src/grid_map_2d.cpp src/astar.cpp
//       src/astar_console_demo.cpp -o /tmp/astar_console
//
// 常用玩法：
//   /tmp/astar_console --map tiny --step            # 8x6 小地图，回车一步步走
//   /tmp/astar_console --compare                    # 各启发函数扩展数对比表
//   /tmp/astar_console --map walls --h zero         # 看 Dijkstra 怎么摊大饼
//   /tmp/astar_console --map walls --weight 3       # Weighted A*，路径变长但快
//   /tmp/astar_console --map random --conn 4 --h manhattan
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "astar_tutorial/astar.h"
#include "astar_tutorial/grid_map_2d.h"
#include "astar_tutorial/trace_replayer.h"

using namespace astar_tutorial;

namespace {

struct Options {
  std::string map_name = "tiny";
  int width = 40;
  int height = 20;
  unsigned seed = 7;
  double ratio = 0.2;
  Heuristic heuristic = Heuristic::kDiagonal;
  int conn = 8;
  double weight = 1.0;
  double tie = 1.0;
  bool corner_cut = false;
  bool step_mode = false;   // 每步等回车
  bool quiet = false;       // 只打最后结果
  bool compare = false;     // 启发函数对比表
  bool show_skips = true;   // 打不打"跳过:障碍/越界"这类事件
  int from_step = 1;
  int to_step = -1;         // -1 = 到最后
  int sx = -1, sy = -1, gx = -1, gy = -1;
};

void printUsage() {
  std::printf(
      "用法: astar_console [选项]\n"
      "  --map tiny|empty|walls|random|maze|corner   地图类型 (默认 tiny)\n"
      "  --size W H            地图尺寸 (默认 40 20，tiny/corner 忽略)\n"
      "  --seed N              随机种子 (默认 7)\n"
      "  --ratio R             random 地图的障碍占据率 (默认 0.2)\n"
      "  --h zero|manhattan|euclidean|diagonal   启发函数 (默认 diagonal)\n"
      "  --conn 4|8            邻域 (默认 8)\n"
      "  --weight W            Weighted A* 的 w (默认 1)\n"
      "  --tie T               tie_breaker，如 1.0001 (默认 1)\n"
      "  --corner-cut          允许斜穿墙角\n"
      "  --start X Y / --goal X Y   起终点栅格坐标\n"
      "  --step                每步等回车（单步调试模式）\n"
      "  --range A B           只详细打印第 A..B 步\n"
      "  --no-skips            不打印\"跳过\"类事件，输出更干净\n"
      "  --quiet               只打印最终结果\n"
      "  --compare             跑一遍所有启发函数，输出对比表\n");
}

// 8x6 手算规模的小地图 —— 拿纸笔跟着算一遍，最能建立直觉
GridMap2D makeTinyMap() {
  GridMap2D m;
  m.loadFromAscii({"........",
                   "..###...",
                   "..#.....",
                   "..#.###.",
                   "....#...",
                   "........"},
                  1.0, 0.0, 0.0);
  return m;
}

// 专门用来演示"斜穿墙角"的地图
GridMap2D makeCornerMap() {
  GridMap2D m;
  m.loadFromAscii({".......",
                   "...#...",
                   "..#....",
                   ".......",
                   "......."},
                  1.0, 0.0, 0.0);
  return m;
}

// 用 overlay 把当前搜索状态叠在地图上打印
//   S 起点  G 终点  @ 当前扩展的节点  o OPEN  x CLOSED  * 最终路径  # 障碍  . 空闲
std::string renderState(const GridMap2D& map, const TraceReplayer& rp,
                        const std::vector<int>& final_path) {
  std::vector<char> overlay(map.numCells(), '\0');
  const auto& st = rp.states();
  for (int id = 0; id < map.numCells(); ++id) {
    if (st[id] == NodeState::kClosed) overlay[id] = 'x';
    else if (st[id] == NodeState::kOpen) overlay[id] = 'o';
  }
  if (rp.atEnd()) {
    for (const int id : final_path) overlay[id] = '*';
  }
  const int cur = rp.currentNodeId();
  if (cur >= 0) overlay[cur] = '@';
  if (rp.startId() >= 0) overlay[rp.startId()] = 'S';
  if (rp.goalId() >= 0) overlay[rp.goalId()] = 'G';
  return map.toAscii(overlay);
}

void printStepDetail(const SearchStep& s, int total, bool show_skips) {
  std::printf("\n=== 第 %d/%d 步 ===\n", s.iteration, total);
  std::printf("① 从 OPEN 里取出 f 最小的节点: (%d,%d)  g=%.3f  h=%.3f  f=%.3f%s\n", s.cur_x,
              s.cur_y, s.cur_g, s.cur_h, s.cur_f,
              s.is_goal ? "   <<<< 它就是终点，搜索结束" : "");
  if (s.is_goal) {
    std::printf("② 是终点 -> 沿 parent 回溯出路径，返回。\n");
    return;
  }
  std::printf("② 把它标记为 CLOSED（g 已确定，以后不再改）\n");
  std::size_t hidden = 0;
  if (!show_skips) {
    for (const NeighborEvent& ev : s.neighbors) {
      if (ev.action != NeighborAction::kPushedNew &&
          ev.action != NeighborAction::kUpdatedBetter) {
        ++hidden;
      }
    }
  }
  std::printf("③ 逐个检查 %zu 个邻居", s.neighbors.size());
  if (hidden > 0) std::printf("（其中 %zu 个被跳过，--no-skips 已隐藏）", hidden);
  std::printf(":\n");
  for (const NeighborEvent& ev : s.neighbors) {
    const bool is_skip = (ev.action != NeighborAction::kPushedNew &&
                          ev.action != NeighborAction::kUpdatedBetter);
    if (is_skip && !show_skips) continue;
    std::printf("     (%2d,%2d) 边代价%.3f  %-22s", ev.x, ev.y, ev.edge_cost,
                toString(ev.action));
    switch (ev.action) {
      case NeighborAction::kPushedNew:
        std::printf("  g: inf -> %.3f   h=%.3f   f=%.3f", ev.g_new, ev.h, ev.f_new);
        break;
      case NeighborAction::kUpdatedBetter:
        std::printf("  g: %.3f -> %.3f   f=%.3f  (换爹)", ev.g_old, ev.g_new, ev.f_new);
        break;
      case NeighborAction::kSkippedWorse:
        std::printf("  g_new=%.3f 不比现有 g=%.3f 小", ev.g_new, ev.g_old);
        break;
      case NeighborAction::kSkippedClosed:
        std::printf("  已定 g=%.3f，不可能更优", ev.g_old);
        break;
      default:
        break;
    }
    std::printf("\n");
  }
  std::printf("④ 本步结束: OPEN=%d  CLOSED=%d\n", s.open_size_after, s.closed_size_after);
}

void printStats(const AStar& a, const GridMap2D& map) {
  const SearchStats& st = a.stats();
  std::printf("\n---------------- 结果 ----------------\n");
  std::printf("状态       : %s\n", toString(st.result));
  std::printf("启发函数   : %s   邻域: %d   weight=%.3f   tie_breaker=%.5f   斜穿墙角: %s\n",
              toString(a.config().heuristic), a.config().use_8_connected ? 8 : 4,
              a.config().weight, a.config().tie_breaker,
              a.config().allow_corner_cutting ? "允许" : "禁止");
  const std::string warn = a.admissibilityWarning();
  if (!warn.empty()) std::printf("!! 警告     : %s\n", warn.c_str());
  std::printf("扩展节点数 : %d   (地图共 %d 格)\n", st.expanded, map.numCells());
  std::printf("入堆次数   : %d   其中过期出堆被丢弃 %d 次 (懒惰删除的开销)\n", st.pushes,
              st.stale_pops);
  std::printf("OPEN 峰值  : %zu\n", st.open_peak);
  std::printf("路径代价   : %.4f 格 = %.4f m   经过 %d 个格子\n", st.path_cost, st.path_length_m,
              st.path_size);
  std::printf("耗时       : %.3f ms\n", st.time_ms);
  std::printf("--------------------------------------\n");
}

// 同一张图、同一对起终点，把所有启发函数各跑一遍，看"扩展数 vs 路径代价"的取舍
void runCompare(GridMap2D& map, int sx, int sy, int gx, int gy, int conn, bool corner_cut) {
  struct Row {
    const char* name;
    Heuristic h;
    double w;
    double tie;
  };
  const std::vector<Row> rows = {
      {"zero (=Dijkstra)", Heuristic::kZero, 1.0, 1.0},
      {"euclidean", Heuristic::kEuclidean, 1.0, 1.0},
      {"manhattan", Heuristic::kManhattan, 1.0, 1.0},
      {"diagonal(octile)", Heuristic::kDiagonal, 1.0, 1.0},
      {"diagonal + tie 1.0001", Heuristic::kDiagonal, 1.0, 1.0001},
      {"diagonal, weight 1.5", Heuristic::kDiagonal, 1.5, 1.0},
      {"diagonal, weight 3.0", Heuristic::kDiagonal, 3.0, 1.0},
  };

  std::printf("\n同一张地图 (%dx%d, %d邻域), 起点(%d,%d) -> 终点(%d,%d)\n", map.width(),
              map.height(), conn, sx, sy, gx, gy);
  // 注意: printf 的字段宽度按**字节**算，中文表头会让整张表错位，所以表头用 ASCII
  std::printf("%-24s %9s %9s %12s %8s %7s\n", "heuristic", "expanded", "pushes", "cost", "ms",
              "admis.");
  std::printf("%s\n", std::string(80, '-').c_str());

  double best_cost = -1.0;
  for (const Row& r : rows) {
    AStar a;
    AStarConfig cfg;
    cfg.heuristic = r.h;
    cfg.weight = r.w;
    cfg.tie_breaker = r.tie;
    cfg.use_8_connected = (conn == 8);
    cfg.allow_corner_cutting = corner_cut;
    cfg.record_trace = false;  // 对比只看数字，不录 trace，跑得快
    a.setMap(&map);
    a.setConfig(cfg);
    a.search(sx, sy, gx, gy);
    const SearchStats& st = a.stats();
    if (best_cost < 0.0 && r.h == Heuristic::kZero) best_cost = st.path_cost;  // Dijkstra = 最优
    const bool optimal = (best_cost < 0.0) || (st.path_cost <= best_cost + 1e-6);
    std::printf("%-24s %9d %9d %12.4f %8.3f %7s%s\n", r.name, st.expanded, st.pushes,
                st.path_cost, st.time_ms, a.isHeuristicAdmissible() ? "yes" : "NO",
                optimal ? "" : "   <- 路径比最优解长!");
  }
  std::printf("%s\n", std::string(80, '-').c_str());
  std::printf("读法: 扩展数越小越快; 路径代价必须等于第一行 Dijkstra 的值才是最短路。\n"
              "      可采纳=否 的行, 路径代价可能变大 —— 这就是\"快\"换来的代价。\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](int n) { return i + n < argc; };
    if (a == "--help" || a == "-h") { printUsage(); return 0; }
    else if (a == "--map" && need(1)) o.map_name = argv[++i];
    else if (a == "--size" && need(2)) { o.width = std::atoi(argv[++i]); o.height = std::atoi(argv[++i]); }
    else if (a == "--seed" && need(1)) o.seed = static_cast<unsigned>(std::atoi(argv[++i]));
    else if (a == "--ratio" && need(1)) o.ratio = std::atof(argv[++i]);
    else if (a == "--h" && need(1)) {
      if (!parseHeuristic(argv[++i], &o.heuristic)) {
        std::printf("未知启发函数 '%s'\n", argv[i]);
        return 1;
      }
    }
    else if (a == "--conn" && need(1)) o.conn = std::atoi(argv[++i]);
    else if (a == "--weight" && need(1)) o.weight = std::atof(argv[++i]);
    else if (a == "--tie" && need(1)) o.tie = std::atof(argv[++i]);
    else if (a == "--corner-cut") o.corner_cut = true;
    else if (a == "--start" && need(2)) { o.sx = std::atoi(argv[++i]); o.sy = std::atoi(argv[++i]); }
    else if (a == "--goal" && need(2)) { o.gx = std::atoi(argv[++i]); o.gy = std::atoi(argv[++i]); }
    else if (a == "--step") o.step_mode = true;
    else if (a == "--range" && need(2)) { o.from_step = std::atoi(argv[++i]); o.to_step = std::atoi(argv[++i]); }
    else if (a == "--no-skips") o.show_skips = false;
    else if (a == "--quiet") o.quiet = true;
    else if (a == "--compare") o.compare = true;
    else { std::printf("未知参数 '%s'\n\n", a.c_str()); printUsage(); return 1; }
  }

  // ---- 造地图 ----
  GridMap2D map;
  if (o.map_name == "tiny") {
    map = makeTinyMap();
  } else if (o.map_name == "corner") {
    map = makeCornerMap();
  } else {
    map.resize(o.width, o.height, 1.0, 0.0, 0.0);
    if (o.map_name == "random") map.fillRandomObstacles(o.ratio, o.seed, 2);
    // 先撒随机障碍再造墙：fillWalls 会把口子强制置空，保证墙上的通道不会被堵死
    else if (o.map_name == "walls") { map.fillRandomObstacles(o.ratio * 0.4, o.seed + 3, 1); map.fillWalls(4, o.seed, 1); }
    else if (o.map_name == "maze") map.fillMaze(o.seed);
    else if (o.map_name != "empty") { std::printf("未知地图 '%s'\n", o.map_name.c_str()); return 1; }
  }

  // ---- 起终点：默认左下角 -> 右上角，压到障碍上就往外找最近的空格 ----
  int sx = (o.sx >= 0) ? o.sx : 0;
  int sy = (o.sy >= 0) ? o.sy : 0;
  int gx = (o.gx >= 0) ? o.gx : map.width() - 1;
  int gy = (o.gy >= 0) ? o.gy : map.height() - 1;
  auto nudge = [&](int* x, int* y) {
    if (map.isFree(*x, *y)) return true;
    for (int r = 1; r < std::max(map.width(), map.height()); ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (map.isFree(*x + dx, *y + dy)) { *x += dx; *y += dy; return true; }
        }
      }
    }
    return false;
  };
  if (!nudge(&sx, &sy) || !nudge(&gx, &gy)) {
    std::printf("地图上找不到空闲格\n");
    return 1;
  }

  if (o.compare) {
    runCompare(map, sx, sy, gx, gy, o.conn, o.corner_cut);
    return 0;
  }

  // ---- 跑搜索 ----
  AStarConfig cfg;
  cfg.heuristic = o.heuristic;
  cfg.use_8_connected = (o.conn == 8);
  cfg.allow_corner_cutting = o.corner_cut;
  cfg.weight = o.weight;
  cfg.tie_breaker = o.tie;
  cfg.record_trace = true;

  AStar astar;
  astar.setMap(&map);
  astar.setConfig(cfg);

  std::printf("地图 %dx%d  起点(%d,%d) -> 终点(%d,%d)\n", map.width(), map.height(), sx, sy, gx, gy);
  std::printf("图例: S起点 G终点 @当前扩展 o=OPEN x=CLOSED *最终路径 #障碍\n");
  astar.search(sx, sy, gx, gy);

  const std::vector<int> final_path = astar.pathIds();

  if (!o.quiet) {
    TraceReplayer rp;
    rp.bind(astar, map.numCells());
    const int total = rp.numSteps();
    const int to = (o.to_step < 0) ? total : std::min(o.to_step, total);

    std::printf("\n【第 0 步】初始状态: g[起点]=0, f[起点]=h(起点), OPEN={起点}\n");
    std::printf("%s", renderState(map, rp, final_path).c_str());

    while (rp.stepForward()) {
      const int k = rp.cursor();
      if (k < o.from_step || k > to) continue;  // 只详细打印指定区间
      printStepDetail(*rp.currentStep(), total, o.show_skips);
      std::printf("%s", renderState(map, rp, final_path).c_str());
      if (o.step_mode) {
        std::printf("[回车=下一步, q+回车=跳到结尾] ");
        std::fflush(stdout);
        std::string line;
        if (!std::getline(std::cin, line)) break;
        if (!line.empty() && (line[0] == 'q' || line[0] == 'Q')) o.from_step = total + 1;
      }
    }
    // 结尾再完整打一次（包含最终路径）
    rp.finish();
    std::printf("\n【最终状态】\n%s", renderState(map, rp, final_path).c_str());
  }

  printStats(astar, map);
  return 0;
}
