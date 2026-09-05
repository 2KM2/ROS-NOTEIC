// BFS / DFS / Dijkstra 控制台演示 —— 不依赖 ROS，直接在终端里逐步打印。
//
// 想真正"看懂每一步"，先在这里跑。它把每一次出队、每个邻居的判决、
// 以及当时的地图状态全都打出来，比 rviz 更适合抠细节。
//
// 编译（不需要 catkin / roscore）：
//   cd src/search_tutorial
//   g++ -std=c++17 -O2 -I include -I ../astar_tutorial/include
//       ../astar_tutorial/src/grid_map_2d.cpp ../astar_tutorial/src/astar.cpp
//       src/graph_search.cpp src/trace_replayer.cpp src/search_console_demo.cpp
//       -o /tmp/search_console
//   （上面四行接成一行执行）
//
// 默认 **4 邻域**（只上下左右，不走对角线）。这一点很重要：4 邻域下每条边代价都是 1，
// 「步数最少」和「代价最小」是同一件事，所以 BFS 和 Dijkstra 的 cost 必然相同。
// 加 --conn 8 才会出现斜线（代价 √2），两个目标才分开。
//
// 常用玩法：
//   /tmp/search_console --algo bfs --map tiny --step      # 回车一步步走 BFS
//   /tmp/search_console --algo dfs --map tiny --step      # 同一张图看 DFS 有多离谱
//   /tmp/search_console --compare                        # 三个算法 + A* 横向对比表（4 邻域）
//   /tmp/search_console --compare --conn 8               # 8 邻域：BFS 那行的 cost 开始变大
//   /tmp/search_console --counterexample                 # 自动找"BFS 路径比最短路更长"的实例
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "astar_tutorial/astar.h"
#include "astar_tutorial/grid_map_2d.h"
#include "search_tutorial/graph_search.h"
#include "search_tutorial/trace_replayer.h"

using namespace search_tutorial;

namespace {

struct Options {
  std::string map_name = "tiny";
  int width = 40;
  int height = 20;
  unsigned seed = 7;
  double ratio = 0.2;
  Algorithm algo = Algorithm::kDijkstra;
  int conn = 4;  // 默认 4 邻域：只上下左右，不走对角线
  bool corner_cut = false;
  bool step_mode = false;    // 每步等回车
  bool quiet = false;        // 只打最后结果
  bool compare = false;      // 三算法对比表
  bool counterexample = false;  // 自动找 BFS 次优的实例
  bool show_skips = true;    // 打不打"跳过:障碍/越界"这类事件
  bool show_stale = true;    // 打不打"弹出重复条目"这类空步
  int from_step = 1;
  int to_step = -1;          // -1 = 到最后
  int sx = -1, sy = -1, gx = -1, gy = -1;
};

void printUsage() {
  std::printf(
      "用法: search_console [选项]\n"
      "  --algo bfs|dfs|dijkstra   选算法 (默认 dijkstra)\n"
      "  --map tiny|empty|walls|random|maze|corner   地图类型 (默认 tiny)\n"
      "  --size W H            地图尺寸 (默认 40 20，tiny/corner 忽略)\n"
      "  --seed N              随机种子 (默认 7)\n"
      "  --ratio R             random 地图的障碍占据率 (默认 0.2)\n"
      "  --conn 4|8            邻域 (默认 4，即只上下左右)。4 邻域下 BFS 和 Dijkstra 等价，\n"
      "                        切到 8 才能看到 BFS 给出比最短路更长的路径\n"
      "  --corner-cut          允许斜穿墙角\n"
      "  --start X Y / --goal X Y   起终点栅格坐标\n"
      "  --step                每步等回车（单步调试模式）\n"
      "  --range A B           只详细打印第 A..B 步\n"
      "  --no-skips            不打印\"跳过\"类事件，输出更干净\n"
      "  --no-stale            不打印\"弹出重复条目\"的空步(DFS 下很多)\n"
      "  --quiet               只打印最终结果\n"
      "  --compare             BFS/DFS/Dijkstra/A* 横向对比表\n"
      "  --counterexample      自动搜一个\"BFS 步数最少但路径更长\"的地图并打印\n");
}

// 8x6 手算规模的小地图 —— 和 astar_tutorial 用的是同一张，方便四个算法对着看
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

bool buildMap(const Options& o, GridMap2D* map) {
  if (o.map_name == "tiny") {
    *map = makeTinyMap();
  } else if (o.map_name == "corner") {
    *map = makeCornerMap();
  } else {
    map->resize(o.width, o.height, 1.0, 0.0, 0.0);
    if (o.map_name == "random") {
      map->fillRandomObstacles(o.ratio, o.seed, 2);
    } else if (o.map_name == "walls") {
      // 先撒随机障碍再造墙：fillWalls 会把口子强制置空，保证墙上的通道不会被堵死
      map->fillRandomObstacles(o.ratio * 0.4, o.seed + 3, 1);
      map->fillWalls(4, o.seed, 1);
    } else if (o.map_name == "maze") {
      map->fillMaze(o.seed);
    } else if (o.map_name != "empty") {
      std::printf("未知地图 '%s'\n", o.map_name.c_str());
      return false;
    }
  }
  return true;
}

// 起终点压在障碍上就往外找最近的空格
bool nudge(const GridMap2D& map, int* x, int* y) {
  if (map.isFree(*x, *y)) return true;
  for (int r = 1; r < std::max(map.width(), map.height()); ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (map.isFree(*x + dx, *y + dy)) {
          *x += dx;
          *y += dy;
          return true;
        }
      }
    }
  }
  return false;
}

// 用 overlay 把当前搜索状态叠在地图上打印
//   S 起点  G 终点  @ 当前出队的节点  o 在 frontier 里  x CLOSED  * 最终路径  # 障碍  . 空闲
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

// 按**显示宽度**右侧补空格。printf 的 %-24s 数的是字节，
// 中文一个字占 3 个字节但只占 2 列，直接用 %-24s 整列都会歪。
std::string padWide(const std::string& s, int width) {
  int cols = 0;
  for (std::size_t i = 0; i < s.size();) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) { cols += 1; i += 1; }
    else if (c < 0xE0) { cols += 1; i += 2; }   // 2 字节：拉丁扩展等，占 1 列
    else if (c < 0xF0) { cols += 2; i += 3; }   // 3 字节：CJK，占 2 列
    else { cols += 2; i += 4; }
  }
  std::string out = s;
  for (int k = cols; k < width; ++k) out += ' ';
  return out;
}

const char* popDescription(Algorithm a) {
  switch (a) {
    case Algorithm::kBfs: return "取出**最早入队**的条目 (FIFO 队列头)";
    case Algorithm::kDfs: return "取出**最晚入栈**的条目 (LIFO 栈顶)";
    case Algorithm::kDijkstra: return "取出 **g 最小**的条目 (小顶堆堆顶)";
  }
  return "";
}

void printStepDetail(const SearchStep& s, int total, Algorithm algo, bool show_skips) {
  std::printf("\n=== 第 %d/%d 步 ===\n", s.iteration, total);
  std::printf("① %s: (%d,%d)  g=%.3f  步数=%d\n", popDescription(algo), s.cur_x, s.cur_y, s.cur_g,
              s.cur_depth);
  if (s.is_stale_pop) {
    std::printf("   ⚠ 这个格子已经 CLOSED 了 —— 这是一份**重复条目**，直接丢掉，本步什么都不做。\n");
    std::printf("④ 本步结束: frontier条目=%zu  OPEN节点=%d  CLOSED=%d  (都没变)\n",
                s.frontier_size_after, s.open_size_after, s.closed_size_after);
    return;
  }
  if (s.is_goal) {
    std::printf("   <<<< 它就是终点\n");
    std::printf("② 沿 parent 回溯出路径，返回。\n");
    return;
  }
  std::printf("② 标记 CLOSED（g / 步数 / 父亲就此定死，以后不再改）\n");
  std::size_t hidden = 0;
  if (!show_skips) {
    for (const NeighborEvent& ev : s.neighbors) {
      if (ev.action != NeighborAction::kPushedNew &&
          ev.action != NeighborAction::kPushedAgain &&
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
                          ev.action != NeighborAction::kPushedAgain &&
                          ev.action != NeighborAction::kUpdatedBetter);
    if (is_skip && !show_skips) continue;
    std::printf("     (%2d,%2d) 边代价%.3f  %s", ev.x, ev.y, ev.edge_cost,
                padWide(toString(ev.action), 22).c_str());
    switch (ev.action) {
      case NeighborAction::kPushedNew:
        std::printf("  g: inf -> %.3f   步数=%d", ev.g_new, ev.depth_new);
        break;
      case NeighborAction::kPushedAgain:
        std::printf("  g: %.3f -> %.3f   步数=%d  (栈里现在有两份)", ev.g_old, ev.g_new,
                    ev.depth_new);
        break;
      case NeighborAction::kUpdatedBetter:
        std::printf("  g: %.3f -> %.3f   步数=%d  (换爹)", ev.g_old, ev.g_new, ev.depth_new);
        break;
      case NeighborAction::kSkippedSeen:
        std::printf("  BFS 只关心步数，先到的那条不会更差");
        break;
      case NeighborAction::kSkippedWorse:
        std::printf("  g_new=%.3f 不比现有 g=%.3f 小", ev.g_new, ev.g_old);
        break;
      case NeighborAction::kSkippedClosed:
        std::printf("  已定 g=%.3f", ev.g_old);
        break;
      default:
        break;
    }
    std::printf("\n");
  }
  std::printf("④ 本步结束: frontier条目=%zu  OPEN节点=%d  CLOSED=%d\n", s.frontier_size_after,
              s.open_size_after, s.closed_size_after);
}

void printStats(const GraphSearch& s, const GridMap2D& map) {
  const SearchStats& st = s.stats();
  std::printf("\n---------------- 结果 ----------------\n");
  std::printf("算法       : %s\n", toString(s.config().algorithm));
  std::printf("状态       : %s\n", toString(st.result));
  std::printf("邻域       : %d   斜穿墙角: %s\n", s.config().use_8_connected ? 8 : 4,
              s.config().allow_corner_cutting ? "允许" : "禁止");
  std::printf("保证       : 代价最小 %s   步数最少 %s\n", s.guaranteesMinCost() ? "是" : "否",
              s.guaranteesMinSteps() ? "是" : "否");
  std::printf("             %s\n", s.optimalityNote().c_str());
  std::printf("扩展节点数 : %d   (地图共 %d 格)\n", st.expanded, map.numCells());
  std::printf("入队次数   : %d   其中弹出后发现是重复条目 %d 次\n", st.pushes, st.stale_pops);
  std::printf("峰值       : frontier 条目 %zu   真实 OPEN 节点 %zu\n", st.frontier_peak,
              st.open_peak);
  std::printf("路径       : 代价 %.4f 格 = %.4f m   步数 %d   经过 %d 个格子\n", st.path_cost,
              st.path_length_m, st.path_steps, st.path_size);
  std::printf("耗时       : %.3f ms\n", st.time_ms);
  std::printf("--------------------------------------\n");
}

// ---------------------------------------------------------------- 对比表
// 同一张图、同一对起终点，三个算法各跑一遍，再加上隔壁包的 A* 当参照。
void runCompare(GridMap2D& map, int sx, int sy, int gx, int gy, int conn, bool corner_cut) {
  std::printf("\n同一张地图 (%dx%d, %d邻域), 起点(%d,%d) -> 终点(%d,%d)\n", map.width(),
              map.height(), conn, sx, sy, gx, gy);
  // 注意: printf 的字段宽度按**字节**算，中文表头会让整张表错位，所以表头和算法名都用 ASCII
  std::printf("%-10s %9s %8s %8s %9s %6s %10s %8s\n", "algo", "expanded", "pushes", "stale",
              "peak", "steps", "cost", "ms");
  std::printf("%s\n", std::string(78, '-').c_str());

  double best_cost = -1.0;
  int best_steps = -1;
  struct Row {
    std::string name;
    int expanded, pushes, stale, steps;
    std::size_t peak;
    double cost, ms;
    bool ok;
  };
  std::vector<Row> rows;

  for (const Algorithm a : {Algorithm::kBfs, Algorithm::kDfs, Algorithm::kDijkstra}) {
    GraphSearch s;
    SearchConfig cfg;
    cfg.algorithm = a;
    cfg.use_8_connected = (conn == 8);
    cfg.allow_corner_cutting = corner_cut;
    cfg.record_trace = false;  // 对比只看数字，不录 trace，跑得快
    s.setMap(&map);
    s.setConfig(cfg);
    s.search(sx, sy, gx, gy);
    const SearchStats& st = s.stats();
    const bool ok = (st.result == SearchResult::kSuccess);
    rows.push_back({shortName(a), st.expanded, st.pushes, st.stale_pops, st.path_steps,
                    st.frontier_peak, st.path_cost, st.time_ms, ok});
    if (ok && a == Algorithm::kDijkstra) best_cost = st.path_cost;  // Dijkstra = 代价真值
    if (ok && a == Algorithm::kBfs) best_steps = st.path_steps;     // BFS = 步数真值
  }

  // 再跑一次隔壁包的 A*，证明"加个 h 就是 A*"，代价必须和 Dijkstra 一样。
  // 启发函数要和邻域配对（这就是那个包的主题）：
  //   4 邻域 -> manhattan（|dx|+|dy| 就是真实最少代价）
  //   8 邻域 -> diagonal/octile
  // 配错了要么不可采纳（丢最优性），要么太保守（白扩展一堆节点）。
  {
    astar_tutorial::AStar a;
    astar_tutorial::AStarConfig cfg;
    cfg.heuristic = (conn == 8) ? astar_tutorial::Heuristic::kDiagonal
                                : astar_tutorial::Heuristic::kManhattan;
    cfg.use_8_connected = (conn == 8);
    cfg.allow_corner_cutting = corner_cut;
    cfg.record_trace = false;
    a.setMap(&map);
    a.setConfig(cfg);
    a.search(sx, sy, gx, gy);
    const astar_tutorial::SearchStats& st = a.stats();
    const bool ok = (st.result == astar_tutorial::SearchResult::kSuccess);
    rows.push_back({(conn == 8) ? "A*(octile)" : "A*(manhat)", st.expanded, st.pushes,
                    st.stale_pops, std::max(0, st.path_size - 1), st.open_peak, st.path_cost,
                    st.time_ms, ok});
  }

  for (const Row& r : rows) {
    if (!r.ok) {
      std::printf("%-10s %9s\n", r.name.c_str(), "  (无解)");
      continue;
    }
    std::printf("%-10s %9d %8d %8d %9zu %6d %10.4f %8.3f", r.name.c_str(), r.expanded, r.pushes,
                r.stale, r.peak, r.steps, r.cost, r.ms);
    if (best_cost > 0.0 && r.cost > best_cost + 1e-6) {
      std::printf("   <- 路径比最短路长 %.1f%%", 100.0 * (r.cost / best_cost - 1.0));
    }
    if (best_steps > 0 && r.steps > best_steps) {
      std::printf("   [步数比 BFS 多 %d]", r.steps - best_steps);
    }
    std::printf("\n");
  }
  std::printf("%s\n", std::string(78, '-').c_str());
  std::printf(
      "读法:\n"
      "  cost  必须等于 Dijkstra 那一行才是最短路（A* 一定相等；BFS 在 8 邻域下可能更大）\n"
      "  steps 必须等于 BFS 那一行才是步数最少（Dijkstra 在 8 邻域下可能更多）\n"
      "  DFS   两列通常都难看得多 —— 它压根不优化这两个量\n"
      "  stale 弹出后发现是重复条目的次数: BFS 恒为 0, DFS 很多, Dijkstra 少量\n"
      "  peak  容器里条目数的峰值 = 内存开销\n");
  if (conn == 4) {
    std::printf(
        "  4 邻域（本包默认）下所有边代价都是 1 -> BFS 和 Dijkstra 的 cost / steps\n"
        "  必然完全相同。此时 BFS 就是正确的最短路算法，而且不用堆。\n"
        "  想看它们分开，加 --conn 8。\n");
  } else {
    std::printf(
        "  8 邻域：斜边代价 √2，边代价不再全为 1 -> BFS 的 cost 可以严格大于 Dijkstra 的。\n"
        "  这正是 4 邻域下学不到的那一课，见 doc/01-bfs.md §1.4。\n");
  }
}

// ---------------------------------------------------------------- 反例搜索
// BFS 求的是"步数最少"，8 邻域下这**不等于**"代价最短"。
// 空地图上两者恰好一致（所以测不出来），必须有障碍逼出"多走一步、少走几个斜线"的局面。
// 与其手搓一张地图，不如让程序自己去找 —— 找到的实例带种子，随时可复现。
void findCounterexample(int max_tries, int w, int h, double ratio, bool corner_cut) {
  std::printf("在 %dx%d 随机地图上搜 \"BFS 步数最少、但路径比最短路更长\" 的实例 "
              "(8 邻域, 障碍率 %.2f)...\n", w, h, ratio);
  std::printf("注意这里**强制用 8 邻域**（--conn 8）：本包默认的 4 邻域下边代价全是 1，\n"
              "BFS 就是最短路算法，这样的反例根本不存在 —— 搜也是白搜。\n\n");

  for (int t = 0; t < max_tries; ++t) {
    GridMap2D map(w, h, 1.0, 0.0, 0.0);
    // block=2 和 "--map random" 完全一致，这样打出来的复现命令才真的能复现同一张图
    map.fillRandomObstacles(ratio, static_cast<unsigned>(1000 + t), 2);
    int sx = 0, sy = 0, gx = w - 1, gy = h - 1;
    if (!nudge(map, &sx, &sy) || !nudge(map, &gx, &gy)) continue;
    if (sx == gx && sy == gy) continue;

    SearchConfig cfg;
    cfg.use_8_connected = true;
    cfg.allow_corner_cutting = corner_cut;
    cfg.record_trace = false;

    GraphSearch bfs;
    cfg.algorithm = Algorithm::kBfs;
    bfs.setMap(&map);
    bfs.setConfig(cfg);
    if (bfs.search(sx, sy, gx, gy) != SearchResult::kSuccess) continue;

    GraphSearch dij;
    cfg.algorithm = Algorithm::kDijkstra;
    dij.setMap(&map);
    dij.setConfig(cfg);
    if (dij.search(sx, sy, gx, gy) != SearchResult::kSuccess) continue;

    if (bfs.stats().path_cost <= dij.stats().path_cost + 1e-6) continue;

    // 找到了。把两条路径画出来。
    std::printf("找到了（试了 %d 张图，seed=%d）\n\n", t + 1, 1000 + t);
    auto render = [&](const GraphSearch& s, const char* title) {
      std::vector<char> overlay(map.numCells(), '\0');
      for (const auto& xy : s.pathGrid()) overlay[map.toId(xy.first, xy.second)] = '*';
      overlay[map.toId(sx, sy)] = 'S';
      overlay[map.toId(gx, gy)] = 'G';
      std::printf("%s: 步数 %d, 代价 %.4f\n%s\n", title, s.stats().path_steps,
                  s.stats().path_cost, map.toAscii(overlay).c_str());
    };
    render(bfs, "BFS      ");
    render(dij, "Dijkstra ");
    std::printf("BFS 的路径步数 %d <= Dijkstra 的 %d（BFS 在这一项永远不输），\n"
                "但代价 %.4f > %.4f，长了 %.2f%% —— BFS 不是最短路算法，就是这个意思。\n\n",
                bfs.stats().path_steps, dij.stats().path_steps, bfs.stats().path_cost,
                dij.stats().path_cost,
                100.0 * (bfs.stats().path_cost / dij.stats().path_cost - 1.0));
    std::printf("复现（--conn 8 不能省，默认是 4 邻域）:\n"
                "  search_console --algo bfs      --conn 8 --map random --size %d %d --seed %d "
                "--start %d %d --goal %d %d --quiet\n"
                "  search_console --algo dijkstra --conn 8 --map random --size %d %d --seed %d "
                "--start %d %d --goal %d %d --quiet\n",
                w, h, 1000 + t, sx, sy, gx, gy, w, h, 1000 + t, sx, sy, gx, gy);
    std::printf("（去掉 --quiet 就能一步步看这两次搜索有什么不同）\n");
    return;
  }
  std::printf("试了 %d 张图都没找到。把 --size 调大或 --ratio 调高再试。\n", max_tries);
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](int n) { return i + n < argc; };
    if (a == "--help" || a == "-h") { printUsage(); return 0; }
    else if (a == "--algo" && need(1)) {
      if (!parseAlgorithm(argv[++i], &o.algo)) {
        std::printf("未知算法 '%s'，可选 bfs / dfs / dijkstra\n", argv[i]);
        return 1;
      }
    }
    else if (a == "--map" && need(1)) o.map_name = argv[++i];
    else if (a == "--size" && need(2)) { o.width = std::atoi(argv[++i]); o.height = std::atoi(argv[++i]); }
    else if (a == "--seed" && need(1)) o.seed = static_cast<unsigned>(std::atoi(argv[++i]));
    else if (a == "--ratio" && need(1)) o.ratio = std::atof(argv[++i]);
    else if (a == "--conn" && need(1)) o.conn = std::atoi(argv[++i]);
    else if (a == "--corner-cut") o.corner_cut = true;
    else if (a == "--start" && need(2)) { o.sx = std::atoi(argv[++i]); o.sy = std::atoi(argv[++i]); }
    else if (a == "--goal" && need(2)) { o.gx = std::atoi(argv[++i]); o.gy = std::atoi(argv[++i]); }
    else if (a == "--step") o.step_mode = true;
    else if (a == "--range" && need(2)) { o.from_step = std::atoi(argv[++i]); o.to_step = std::atoi(argv[++i]); }
    else if (a == "--no-skips") o.show_skips = false;
    else if (a == "--no-stale") o.show_stale = false;
    else if (a == "--quiet") o.quiet = true;
    else if (a == "--compare") o.compare = true;
    else if (a == "--counterexample") o.counterexample = true;
    else { std::printf("未知参数 '%s'\n\n", a.c_str()); printUsage(); return 1; }
  }

  if (o.counterexample) {
    findCounterexample(4000, o.width == 40 ? 14 : o.width, o.height == 20 ? 10 : o.height,
                       o.ratio, o.corner_cut);
    return 0;
  }

  // ---- 造地图 ----
  GridMap2D map;
  if (!buildMap(o, &map)) return 1;

  // ---- 起终点：默认左下角 -> 右上角 ----
  int sx = (o.sx >= 0) ? o.sx : 0;
  int sy = (o.sy >= 0) ? o.sy : 0;
  int gx = (o.gx >= 0) ? o.gx : map.width() - 1;
  int gy = (o.gy >= 0) ? o.gy : map.height() - 1;
  if (!nudge(map, &sx, &sy) || !nudge(map, &gx, &gy)) {
    std::printf("地图上找不到空闲格\n");
    return 1;
  }

  if (o.compare) {
    runCompare(map, sx, sy, gx, gy, o.conn, o.corner_cut);
    return 0;
  }

  // ---- 跑搜索 ----
  SearchConfig cfg;
  cfg.algorithm = o.algo;
  cfg.use_8_connected = (o.conn == 8);
  cfg.allow_corner_cutting = o.corner_cut;
  cfg.record_trace = true;
  cfg.record_stale_pops = o.show_stale;

  GraphSearch search;
  search.setMap(&map);
  search.setConfig(cfg);

  std::printf("算法 %s\n", toString(o.algo));
  std::printf("地图 %dx%d  起点(%d,%d) -> 终点(%d,%d)\n", map.width(), map.height(), sx, sy, gx, gy);
  std::printf("图例: S起点 G终点 @当前出队 o在frontier里 x=CLOSED *最终路径 #障碍\n");
  search.search(sx, sy, gx, gy);

  const std::vector<int> final_path = search.pathIds();

  if (!o.quiet) {
    TraceReplayer rp;
    rp.bind(search, map.numCells());
    const int total = rp.numSteps();
    const int to = (o.to_step < 0) ? total : std::min(o.to_step, total);

    std::printf("\n【第 0 步】初始状态: g[起点]=0, 步数[起点]=0, frontier={起点}\n");
    std::printf("%s", renderState(map, rp, final_path).c_str());

    while (rp.stepForward()) {
      const int k = rp.cursor();
      if (k < o.from_step || k > to) continue;  // 只详细打印指定区间
      printStepDetail(*rp.currentStep(), total, o.algo, o.show_skips);
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

  printStats(search, map);
  return 0;
}
