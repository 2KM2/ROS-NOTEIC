// BFS / DFS / Dijkstra 可视化节点 —— 在 rviz 里一步一步调试路径。
//
// 它自己不含任何算法逻辑：算法在 graph_search.cpp，"复原第 k 步的状态"在
// trace_replayer.cpp，这里只干三件事：造地图 / 接 rviz 的点击 / 把状态画成 Marker。
//
// 和 astar_tutorial 的那个节点最大的不同：**可以在线换算法**。
// 同一张图、同一对起终点，按 1/2/3 切 BFS/DFS/Dijkstra，看三种"取出规则"
// 长出完全不同的扩展形状 —— 这是本包的主要观察对象。
//
// 颜色约定（和 README 一致）：
//   灰   nav_msgs/OccupancyGrid   障碍
//   蓝   closed    已经出队处理过，g/步数/父亲定死了
//   绿   open      在 frontier 里排队，等着被取出来
//   红   current   本步取出的节点
//   紫   current   本步取出的是**重复条目**（懒惰去重丢掉它，什么都不做）
//   黄   neighbors 本步检查过的邻居
//   青   tentative_path  起点到 current 的当前父链（Dijkstra 松弛/DFS 重压时会抖）
//   橙   path      最终路径
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "search_tutorial/graph_search.h"
#include "search_tutorial/trace_replayer.h"

namespace search_tutorial {

class SearchRvizNode {
 public:
  SearchRvizNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadParams();
    buildMapConnected();

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("search/map", 1, /*latch=*/true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("search/markers", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("search/path", 1, /*latch=*/true);

    // rviz 自带的两个工具，不需要任何插件：
    //   "2D Pose Estimate" -> /initialpose            设起点
    //   "2D Nav Goal"      -> /move_base_simple/goal  设终点
    start_sub_ = nh_.subscribe("initialpose", 1, &SearchRvizNode::onStartClick, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &SearchRvizNode::onGoalClick, this);
    cmd_sub_ = nh_.subscribe("search/cmd", 10, &SearchRvizNode::onCmd, this);

    srv_step_ = nh_.advertiseService("search/step", &SearchRvizNode::srvStep, this);
    srv_back_ = nh_.advertiseService("search/back", &SearchRvizNode::srvBack, this);
    srv_play_ = nh_.advertiseService("search/play_pause", &SearchRvizNode::srvPlayPause, this);
    srv_reset_ = nh_.advertiseService("search/reset", &SearchRvizNode::srvReset, this);
    srv_finish_ = nh_.advertiseService("search/finish", &SearchRvizNode::srvFinish, this);
    srv_replan_ = nh_.advertiseService("search/replan", &SearchRvizNode::srvReplan, this);
    srv_newmap_ = nh_.advertiseService("search/new_map", &SearchRvizNode::srvNewMap, this);
    srv_bfs_ = nh_.advertiseService("search/use_bfs", &SearchRvizNode::srvBfs, this);
    srv_dfs_ = nh_.advertiseService("search/use_dfs", &SearchRvizNode::srvDfs, this);
    srv_dij_ = nh_.advertiseService("search/use_dijkstra", &SearchRvizNode::srvDijkstra, this);
    srv_compare_ = nh_.advertiseService("search/compare", &SearchRvizNode::srvCompare, this);

    publishMap();
    runSearch();

    // 30 Hz 定时器；播放时按 replay_rate 换算出每 tick 走几步
    timer_ = nh_.createTimer(ros::Duration(1.0 / kTickHz), &SearchRvizNode::onTimer, this);
    printHelp();
  }

 private:
  static constexpr double kTickHz = 30.0;
  static constexpr int kMaxMapAttempts = 12;  // 换种子重试上限，见 buildMapConnected()

  // =============================================================== 参数
  void loadParams() {
    pnh_.param<std::string>("frame_id", frame_id_, std::string("map"));

    pnh_.param("map/width", map_w_, 40);
    pnh_.param("map/height", map_h_, 28);
    pnh_.param("map/resolution", map_res_, 0.5);
    pnh_.param<std::string>("map/type", map_type_, std::string("walls"));
    pnh_.param("map/obstacle_ratio", map_ratio_, 0.18);
    pnh_.param("map/num_walls", map_num_walls_, 4);
    pnh_.param("map/wall_gap", map_wall_gap_, 2);
    pnh_.param("map/seed", map_seed_, 2026);
    pnh_.param("map/inflate_radius", map_inflate_, 0);
    pnh_.param("map/border", map_border_, true);

    std::string algo_name;
    pnh_.param<std::string>("search/algorithm", algo_name, std::string("dijkstra"));
    if (!parseAlgorithm(algo_name, &cfg_.algorithm)) {
      say("[警告] 未知的 algorithm '%s'，可选 bfs/dfs/dijkstra，回退到 dijkstra",
          algo_name.c_str());
      cfg_.algorithm = Algorithm::kDijkstra;
    }
    // 默认 4 邻域：机器人只上下左右走，不走对角线
    pnh_.param("search/use_8_connected", cfg_.use_8_connected, false);
    pnh_.param("search/allow_corner_cutting", cfg_.allow_corner_cutting, false);
    pnh_.param("search/max_iterations", cfg_.max_iterations, 0);
    pnh_.param("search/record_stale_pops", cfg_.record_stale_pops, true);
    cfg_.record_trace = true;  // 单步回放的前提

    pnh_.param("replay/rate", replay_rate_, 40.0);
    pnh_.param("replay/auto_play", playing_, true);
    pnh_.param("replay/show_cost_text", show_cost_text_, false);
    pnh_.param("replay/max_text_labels", max_text_labels_, 500);
    pnh_.param("replay/show_tentative_path", show_tentative_path_, true);
    pnh_.param("replay/show_neighbors", show_neighbors_, true);
    pnh_.param("replay/compare_on_replan", compare_on_replan_, true);

    pnh_.param("planning/start_ix", req_start_ix_, -1);
    pnh_.param("planning/start_iy", req_start_iy_, -1);
    pnh_.param("planning/goal_ix", req_goal_ix_, -1);
    pnh_.param("planning/goal_iy", req_goal_iy_, -1);
  }

  // ---------------------------------------------------------------- 中文输出
  // 为什么不用 ROS_INFO？
  // rosconsole 底层的 log4cxx 在这个镜像的 locale 下会把每一个非 ASCII **字节**
  // 替换成 '?'，于是所有中文提示都变成 "?????"（`rosrun` 和 `roslaunch` 都一样）。
  // 教学节点的输出全是中文，所以这里直接写 stdout；
  // ROS_WARN/ROS_ERROR 只留给纯 ASCII 的消息，那条路还要负责 /rosout 和日志文件。
  static void sayLine(const std::string& text) {
    std::fwrite(text.data(), 1, text.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);  // roslaunch 抓的是管道；不 flush 会一直攒着不吐出来
  }

  __attribute__((format(printf, 1, 2))) static void say(const char* fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sayLine(buf);
  }

  void printHelp() {
    sayLine(
        "\n============ BFS / DFS / Dijkstra 教学节点 ============\n"
        "rviz 里:  2D Pose Estimate 设起点   2D Nav Goal 设终点\n"
        "键盘控制:  rosrun search_tutorial search_keyboard\n"
        "  空格/n/→=下一步   b/←=上一步   p=播放/暂停   r=回到第0步   f=跳到结尾\n"
        "  1=BFS  2=DFS  3=Dijkstra  a=循环切换  c=三算法对比  v=切4/8邻域  t=显示g/步数\n"
        "服务控制:  rosservice call /search/step | back | play_pause | reset | finish\n"
        "           | replan | new_map | use_bfs | use_dfs | use_dijkstra | compare\n"
        "颜色:  蓝=CLOSED  绿=在frontier里  红=本步取出  紫=取出的是重复条目\n"
        "       黄=本步检查的邻居  青=当前父链  橙=最终路径\n"
        "看什么:  BFS/Dijkstra 一圈圈往外推(4邻域下是菱形) / DFS 一条线扎到底\n"
        "默认 4 邻域(只上下左右,不走对角线): 每条边代价都是 1,\n"
        "        所以 BFS 和 Dijkstra 求的是同一个最优值, cost/steps 必然相同。\n"
        "        按 v 切到 8 邻域(斜边 √2), 再按 c, 就能看到 BFS 开始给出更长的路径。\n"
        "======================================================");
  }

  // =============================================================== 地图
  void buildMap() {
    map_.resize(map_w_, map_h_, map_res_,
                -0.5 * map_w_ * map_res_,   // 把地图摆在原点周围，rviz 里好找
                -0.5 * map_h_ * map_res_);

    if (map_type_ == "random") {
      map_.fillRandomObstacles(map_ratio_, static_cast<unsigned>(map_seed_), 2);
    } else if (map_type_ == "maze") {
      map_.fillMaze(static_cast<unsigned>(map_seed_));
    } else if (map_type_ == "walls") {
      // 顺序很重要：先撒随机障碍，再造墙。fillWalls 会把墙上的口子强制置空，
      // 所以放在后面能保证口子一定通；反过来写就可能被随机障碍堵死 -> 地图无解。
      map_.fillRandomObstacles(map_ratio_ * 0.5, static_cast<unsigned>(map_seed_) + 7, 2);
      map_.fillWalls(map_num_walls_, static_cast<unsigned>(map_seed_), map_wall_gap_);
    } else if (map_type_ == "empty") {
      // 空地图：最适合观察三种"取出规则"的扩展形状差异
      //   4 邻域(默认)：BFS = 菱形(曼哈顿等距面)，Dijkstra = 同一个菱形，DFS = 一条乱线
      //   8 邻域：      BFS = 方形(切比雪夫等距面)，Dijkstra = 八角形(近似圆) —— 形状分开了，
      //                 因为这时"步数相等"和"代价相等"不再是同一条等高线
    } else {
      say("[警告] 未知 map/type '%s'，可选 empty/random/walls/maze，按 walls 处理",
          map_type_.c_str());
      map_.fillWalls(map_num_walls_, static_cast<unsigned>(map_seed_), map_wall_gap_);
    }

    if (map_inflate_ > 0) map_.inflate(map_inflate_);
    if (map_border_) map_.addBorder();
    search_.setMap(&map_);
  }

  // 造地图 + 挑起终点，并保证两点确实连通。
  // 随机地图完全可能把起点封在孤立小房间里，那时三个算法都会走"frontier 抽干 -> 无解"
  // 分支。这个分支本身值得看（map/type=random + obstacle_ratio=0.45），但不该是默认体验。
  // 注意重试只换 map/seed，不动任何搜索参数 —— 算法不会为了"结果好看"被偷偷改掉。
  void buildMapConnected() {
    const int base_seed = map_seed_;
    for (int attempt = 0; attempt < kMaxMapAttempts; ++attempt) {
      map_seed_ = base_seed + attempt;
      buildMap();
      pickDefaultEndpoints();
      if (map_.isConnected(start_ix_, start_iy_, goal_ix_, goal_iy_, cfg_.use_8_connected,
                           cfg_.allow_corner_cutting)) {
        if (attempt > 0) {
          say("[警告] map/seed=%d 生成的地图里起终点不连通，已改用 seed=%d（第 %d 次尝试）",
              base_seed, map_seed_, attempt + 1);
        }
        return;
      }
    }
    say("[警告] 连试 %d 个种子都造不出连通的地图（障碍太密？），保留最后一张 —— "
        "接下来你会看到\"无解\"分支：frontier 被抽干。", kMaxMapAttempts);
  }

  // 找一个离期望位置最近的空闲格，避免默认起终点正好压在障碍上
  bool findNearestFree(int wx, int wy, int* ox, int* oy) const {
    if (map_.isFree(wx, wy)) {
      *ox = wx;
      *oy = wy;
      return true;
    }
    const int max_r = std::max(map_.width(), map_.height());
    for (int r = 1; r <= max_r; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) continue;  // 只走这一圈
          const int x = wx + dx;
          const int y = wy + dy;
          if (map_.isFree(x, y)) {
            *ox = x;
            *oy = y;
            return true;
          }
        }
      }
    }
    return false;
  }

  void pickDefaultEndpoints() {
    const int sx = req_start_ix_ >= 0 ? req_start_ix_ : 2;
    const int sy = req_start_iy_ >= 0 ? req_start_iy_ : 2;
    const int gx = req_goal_ix_ >= 0 ? req_goal_ix_ : map_.width() - 3;
    const int gy = req_goal_iy_ >= 0 ? req_goal_iy_ : map_.height() - 3;
    if (!findNearestFree(sx, sy, &start_ix_, &start_iy_)) {
      start_ix_ = sx;
      start_iy_ = sy;
    }
    if (!findNearestFree(gx, gy, &goal_ix_, &goal_iy_)) {
      goal_ix_ = gx;
      goal_iy_ = gy;
    }
  }

  void publishMap() {
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = map_.resolution();
    grid.info.width = map_.width();
    grid.info.height = map_.height();
    grid.info.origin.position.x = map_.originX();
    grid.info.origin.position.y = map_.originY();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(map_.data().begin(), map_.data().end());
    map_pub_.publish(grid);
  }

  // =============================================================== 搜索
  // rewind=true: 不管当前是播放还是暂停，都回到第 0 步。
  // 换算法 / 换邻域时用它 —— 那两种情况下你想看的是**新算法从头怎么走**，
  // 而不是它的最终结果（点击换起终点则相反，直接放给你看更顺手）。
  void runSearch(bool rewind = false) {
    search_.setConfig(cfg_);
    const SearchResult r = search_.search(start_ix_, start_iy_, goal_ix_, goal_iy_);
    const SearchStats& st = search_.stats();

    say("\n[%s] %s | %d邻域 | 扩展 %d 个节点, 入队 %d 次, 弹出重复条目 %d 次, "
        "frontier峰值 %zu (真实OPEN峰值 %zu) | 路径 %.3f 格 = %.3f m, %d 步 | %.3f ms",
        shortName(cfg_.algorithm), toString(r), cfg_.use_8_connected ? 8 : 4, st.expanded,
        st.pushes, st.stale_pops, st.frontier_peak, st.open_peak, st.path_cost, st.path_length_m,
        st.path_steps, st.time_ms);
    sayLine("       " + search_.optimalityNote());

    if (compare_on_replan_) logCompare();

    replayer_.bind(search_, map_.numCells());
    if (playing_ || rewind) {
      replayer_.reset();
    } else {
      replayer_.finish();  // 暂停模式下直接给最终结果，想回看再 reset
    }
    publishPath();
    publishVis();
  }

  // 同一张图、同一对起终点，三个算法各跑一遍，把数字并排打出来。
  // 这是本包最有说服力的一屏：**代价**那一列只有 Dijkstra 是最小的，
  // **步数**那一列只有 BFS 是最小的，DFS 两列都难看。
  void logCompare() {
    SearchConfig cfg = cfg_;
    cfg.record_trace = false;  // 只要数字，不录 trace
    GraphSearch tmp;
    tmp.setMap(&map_);

    sayLine("       ---- 同一张图三个算法对比 (cost=几何代价, steps=步数) ----");
    say("       %-9s %9s %8s %8s %9s %6s %9s  %s", "algo", "expanded", "pushes", "stale", "peak",
        "steps", "cost", "guarantee");
    double best_cost = -1.0;
    int best_steps = -1;
    struct Row {
      const char* name;
      int expanded, pushes, stale, steps;
      std::size_t peak;
      double cost;
      bool ok, min_cost, min_steps;
    };
    std::vector<Row> rows;
    for (const Algorithm a : {Algorithm::kBfs, Algorithm::kDfs, Algorithm::kDijkstra}) {
      cfg.algorithm = a;
      tmp.setConfig(cfg);
      tmp.search(start_ix_, start_iy_, goal_ix_, goal_iy_);
      const SearchStats& s = tmp.stats();
      const bool ok = s.result == SearchResult::kSuccess;
      rows.push_back({shortName(a), s.expanded, s.pushes, s.stale_pops, s.path_steps,
                      s.frontier_peak, s.path_cost, ok, tmp.guaranteesMinCost(),
                      tmp.guaranteesMinSteps()});
      if (ok && a == Algorithm::kDijkstra) best_cost = s.path_cost;
      if (ok && a == Algorithm::kBfs) best_steps = s.path_steps;
    }
    for (const Row& r : rows) {
      if (!r.ok) {
        say("       %-9s  (无解)", r.name);
        continue;
      }
      std::string tail;
      if (r.min_cost) tail += "代价最小 ";
      if (r.min_steps) tail += "步数最少 ";
      if (tail.empty()) tail = "无保证 ";
      if (best_cost > 0.0 && r.cost > best_cost + 1e-6) {
        char b[64];
        std::snprintf(b, sizeof(b), "<-比最短路长%.1f%%", 100.0 * (r.cost / best_cost - 1.0));
        tail += b;
      } else if (best_steps > 0 && r.steps > best_steps) {
        char b[64];
        std::snprintf(b, sizeof(b), "<-比BFS多%d步", r.steps - best_steps);
        tail += b;
      }
      say("       %-9s %9d %8d %8d %9zu %6d %9.4f  %s", r.name, r.expanded, r.pushes, r.stale,
          r.peak, r.steps, r.cost, tail.c_str());
    }
    if (cfg_.use_8_connected) {
      sayLine("       8 邻域：斜边代价 √2，BFS 的 cost 可以严格大于 Dijkstra 的。按 v 切回 4 邻域。");
    } else {
      sayLine("       4 邻域：所有边代价都是 1，BFS 和 Dijkstra 的 cost/steps 必然相同 —— "
              "此时 BFS 就够用。");
      sayLine("       按 v 切到 8 邻域(允许对角线)，再按 c，就能看到 BFS 那行开始次优。");
    }
  }

  void setAlgorithm(Algorithm a) {
    if (cfg_.algorithm == a) {
      say("[算法] 已经是 %s 了", shortName(a));
      return;
    }
    cfg_.algorithm = a;
    say("\n[算法] 切换到 %s —— 地图和起终点都不变，只换 frontier 的取出规则", toString(a));
    runSearch(/*rewind=*/true);  // 暂停中也回到第 0 步，方便直接按 n 单步对比
  }

  // =============================================================== 回放
  void onTimer(const ros::TimerEvent&) {
    if (playing_ && replayer_.valid()) {
      // 每 tick 该走几步。rate 很高时一次走多步，rate 很低时按累加器攒够 1 步再走。
      step_accum_ += replay_rate_ / kTickHz;
      int n = static_cast<int>(step_accum_);
      step_accum_ -= n;
      bool moved = false;
      while (n-- > 0 && replayer_.stepForward()) moved = true;
      if (moved) publishVis();
      if (replayer_.atEnd() && playing_) {
        playing_ = false;
        say("[回放] 播放完毕，共 %d 步。r=从头再放  b=后退一步", replayer_.numSteps());
        publishVis();
      }
    } else {
      // 暂停时也定期重发一次，防止 rviz 是后启动的、错过了 marker
      if ((++idle_ticks_ % static_cast<int>(kTickHz * 2)) == 0) publishVis();
    }
  }

  void onCmd(const std_msgs::String::ConstPtr& msg) { handleCmd(msg->data); }

  void handleCmd(const std::string& cmd) {
    if (cmd == "step") {
      playing_ = false;
      if (replayer_.stepForward()) {
        logCurrentStep();
      } else {
        say("[回放] 已经在最后一步 (%d/%d)", replayer_.cursor(), replayer_.numSteps());
      }
    } else if (cmd == "back") {
      playing_ = false;
      replayer_.seekTo(replayer_.cursor() - 1);
      logCurrentStep();
    } else if (cmd == "play" || cmd == "play_pause" || cmd == "toggle") {
      if (replayer_.atEnd() && !playing_) replayer_.reset();  // 放完了再按 = 重播
      playing_ = !playing_;
      say("[回放] %s (%d/%d, %.1f 步/秒)", playing_ ? "播放" : "暂停", replayer_.cursor(),
          replayer_.numSteps(), replay_rate_);
    } else if (cmd == "reset") {
      playing_ = false;
      replayer_.reset();
      say("[回放] 回到第 0 步");
    } else if (cmd == "finish") {
      playing_ = false;
      replayer_.finish();
      say("[回放] 跳到最后一步 (%d)", replayer_.numSteps());
    } else if (cmd == "faster") {
      replay_rate_ = std::min(replay_rate_ * 1.6, 20000.0);
      say("[回放] 速度 %.1f 步/秒", replay_rate_);
    } else if (cmd == "slower") {
      replay_rate_ = std::max(replay_rate_ / 1.6, 0.5);
      say("[回放] 速度 %.1f 步/秒", replay_rate_);
    } else if (cmd == "replan") {
      runSearch();
      return;
    } else if (cmd == "new_map") {
      map_seed_ += 1;
      buildMapConnected();
      publishMap();
      runSearch();
      return;
    } else if (cmd == "bfs" || cmd == "algo_bfs") {
      setAlgorithm(Algorithm::kBfs);
      return;
    } else if (cmd == "dfs" || cmd == "algo_dfs") {
      setAlgorithm(Algorithm::kDfs);
      return;
    } else if (cmd == "dijkstra" || cmd == "algo_dijkstra") {
      setAlgorithm(Algorithm::kDijkstra);
      return;
    } else if (cmd == "algo_next") {
      const Algorithm next = (cfg_.algorithm == Algorithm::kBfs)   ? Algorithm::kDfs
                             : (cfg_.algorithm == Algorithm::kDfs) ? Algorithm::kDijkstra
                                                                   : Algorithm::kBfs;
      setAlgorithm(next);
      return;
    } else if (cmd == "compare") {
      logCompare();
      return;
    } else if (cmd == "conn") {
      cfg_.use_8_connected = !cfg_.use_8_connected;
      if (cfg_.use_8_connected) {
        say("\n[邻域] 切换到 8 邻域（允许对角线，斜边代价 √2）—— 边代价不再全是 1，"
            "BFS 的 cost 现在可能大于 Dijkstra 的，按 c 看对比表");
      } else {
        say("\n[邻域] 切换回 4 邻域（只上下左右，本包默认）—— 所有边代价都是 1，"
            "BFS 和 Dijkstra 的 cost 必然相同");
      }
      runSearch(/*rewind=*/true);
      return;
    } else if (cmd == "text") {
      show_cost_text_ = !show_cost_text_;
      say("[显示] g / 步数 文字标签: %s", show_cost_text_ ? "开" : "关");
    } else {
      say("[警告] 未知命令 '%s'", cmd.c_str());
      return;
    }
    publishVis();
  }

  void logCurrentStep() {
    const SearchStep* s = replayer_.currentStep();
    if (s == nullptr) {
      say("[第 0 步] 初始状态：只有起点在 frontier 里");
      return;
    }
    std::ostringstream os;
    os << "\n[第 " << s->iteration << "/" << replayer_.numSteps() << " 步] "
       << popVerb() << ": (" << s->cur_x << "," << s->cur_y << ")  g=" << s->cur_g
       << "  步数=" << s->cur_depth;
    if (s->is_stale_pop) {
      os << "\n  这个格子已经 CLOSED 了 —— 重复条目，直接丢掉，本步什么都不做。"
            "\n  （DFS 压栈不查重、Dijkstra 松弛后重入，都会留下这种条目）";
      sayLine(os.str());
      return;
    }
    if (s->is_goal) os << "   <<< 这就是终点，搜索结束";
    os << "\n  frontier条目=" << s->frontier_size_after << "  OPEN节点=" << s->open_size_after
       << "  CLOSED=" << s->closed_size_after << "\n";
    for (const NeighborEvent& ev : s->neighbors) {
      os << "    邻居(" << ev.x << "," << ev.y << ")  " << toString(ev.action);
      if (ev.action == NeighborAction::kPushedNew ||
          ev.action == NeighborAction::kPushedAgain ||
          ev.action == NeighborAction::kUpdatedBetter ||
          ev.action == NeighborAction::kSkippedWorse ||
          ev.action == NeighborAction::kSkippedClosed) {
        // g_old 为 inf 表示"这格从没被发现过"。直接打 "inf" 而不是拿 -1 冒充，
        // 否则日志里会冒出一个看着像 bug 的负代价。
        os << "   g_old=";
        if (ev.g_old == kInf) os << "inf";
        else os << ev.g_old;
        os << " g_new=" << ev.g_new << " 步数=" << ev.depth_new;
      }
      os << "\n";
    }
    sayLine(os.str());
  }

  const char* popVerb() const {
    switch (cfg_.algorithm) {
      case Algorithm::kBfs: return "出队(FIFO队头,最早入队的)";
      case Algorithm::kDfs: return "出栈(LIFO栈顶,最晚入栈的)";
      case Algorithm::kDijkstra: return "出堆(g最小的)";
    }
    return "取出";
  }

  // =============================================================== 可视化
  // 每一类节点画成一个 CUBE_LIST（一个 marker 装成千上万个方块，比一格一个 marker 快得多）。
  visualization_msgs::Marker makeCubeList(const std::string& ns, double cube_h, double r, double g,
                                          double b, double a, double shrink = 0.92) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = map_.resolution() * shrink;
    m.scale.y = map_.resolution() * shrink;
    m.scale.z = cube_h;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  }

  void addCell(visualization_msgs::Marker* m, int id, double z) const {
    int x = 0;
    int y = 0;
    map_.toXY(id, &x, &y);
    double wx = 0.0;
    double wy = 0.0;
    map_.gridToWorld(x, y, &wx, &wy);
    geometry_msgs::Point p;
    p.x = wx;
    p.y = wy;
    p.z = z;
    m->points.push_back(p);
  }

  void publishVis() {
    if (!replayer_.valid()) return;
    visualization_msgs::MarkerArray arr;

    // 先 DELETEALL：文字标签数量每步都在变，不清一遍会留下上一帧的残影
    {
      visualization_msgs::Marker del;
      del.header.frame_id = frame_id_;
      del.action = visualization_msgs::Marker::DELETEALL;
      arr.markers.push_back(del);
    }

    const double res = map_.resolution();
    const std::vector<int> closed = replayer_.closedIds();
    const std::vector<int> open = replayer_.openIds();
    const int cur_id = replayer_.currentNodeId();
    const SearchStep* step = replayer_.currentStep();
    const bool stale = (step != nullptr && step->is_stale_pop);

    // --- CLOSED：蓝，扁扁地铺在地上 ---
    {
      auto m = makeCubeList("closed", 0.05, 0.15, 0.35, 0.95, 0.55);
      for (const int id : closed) {
        if (id == cur_id && !stale) continue;  // 当前节点单独用红色画
        addCell(&m, id, 0.03);
      }
      arr.markers.push_back(m);
    }

    // --- 在 frontier 里的：绿，稍微高一点，能压在 closed 上面 ---
    {
      auto m = makeCubeList("open", 0.09, 0.15, 0.9, 0.25, 0.6);
      for (const int id : open) addCell(&m, id, 0.06);
      arr.markers.push_back(m);
    }

    // --- 本步检查过的邻居：黄框 ---
    if (show_neighbors_ && step != nullptr) {
      auto m = makeCubeList("neighbors", 0.14, 1.0, 0.85, 0.1, 0.45, 0.55);
      for (const NeighborEvent& ev : step->neighbors) {
        if (ev.id < 0) continue;
        addCell(&m, ev.id, 0.12);
      }
      arr.markers.push_back(m);
    }

    // --- 本步取出的节点：红柱子。若取出的是重复条目则用紫色，
    //     提醒"这一步算法什么都没做"（懒惰去重）。
    {
      auto m = stale ? makeCubeList("current", 0.6, 0.75, 0.15, 0.95, 0.85)
                     : makeCubeList("current", 0.6, 1.0, 0.1, 0.1, 0.95);
      if (cur_id >= 0) addCell(&m, cur_id, 0.3);
      arr.markers.push_back(m);
    }

    // --- 当前父链（起点 -> current），青色细线 ---
    // Dijkstra 松弛换爹、DFS 重复压栈时这条链会明显地抖 —— 那就是"上界在被放松"。
    if (show_tentative_path_ && cur_id >= 0) {
      visualization_msgs::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = ros::Time::now();
      m.ns = "tentative_path";
      m.id = 0;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = res * 0.18;
      m.color.r = 0.1;
      m.color.g = 0.95;
      m.color.b = 0.95;
      m.color.a = 0.9;
      for (const int id : replayer_.pathTo(cur_id)) {
        int x = 0;
        int y = 0;
        map_.toXY(id, &x, &y);
        double wx = 0.0;
        double wy = 0.0;
        map_.gridToWorld(x, y, &wx, &wy);
        geometry_msgs::Point p;
        p.x = wx;
        p.y = wy;
        p.z = 0.22;
        m.points.push_back(p);
      }
      if (m.points.size() >= 2) arr.markers.push_back(m);
    }

    // --- 最终路径：橙色粗线。只在放完时出现 ---
    {
      visualization_msgs::Marker line;
      line.header.frame_id = frame_id_;
      line.header.stamp = ros::Time::now();
      line.ns = "path";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = res * 0.35;
      line.color.r = 1.0;
      line.color.g = 0.55;
      line.color.b = 0.0;
      line.color.a = 1.0;
      if (replayer_.atEnd() && search_.stats().result == SearchResult::kSuccess) {
        for (const auto& pt : search_.pathWorld()) {
          geometry_msgs::Point p;
          p.x = pt.first;
          p.y = pt.second;
          p.z = 0.4;
          line.points.push_back(p);
        }
      }
      if (line.points.size() >= 2) arr.markers.push_back(line);
    }

    // --- 起点 / 终点：两个球 ---
    for (int k = 0; k < 2; ++k) {
      visualization_msgs::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = ros::Time::now();
      m.ns = (k == 0) ? "start" : "goal";
      m.id = 0;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      double wx = 0.0;
      double wy = 0.0;
      map_.gridToWorld(k == 0 ? start_ix_ : goal_ix_, k == 0 ? start_iy_ : goal_iy_, &wx, &wy);
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = res * 1.6;
      m.color.r = (k == 0) ? 0.7 : 0.0;
      m.color.g = (k == 0) ? 0.2 : 1.0;
      m.color.b = (k == 0) ? 1.0 : 0.6;
      m.color.a = 0.95;
      arr.markers.push_back(m);
    }

    // --- 每格上的两行数字：g（几何代价） / 步数。格子多时会卡 rviz，所以有数量上限 ---
    //
    // 这两个数字**同时**显示是有意的：本包的核心结论就是它们不是一回事。
    // 在 8 邻域下盯着终点附近看：BFS 的"步数"最小但 g 偏大，Dijkstra 反过来。
    if (show_cost_text_) {
      const int total = static_cast<int>(open.size() + closed.size());
      if (total <= max_text_labels_) {
        int mid = 0;
        auto add_text = [&](int id) {
          int x = 0;
          int y = 0;
          map_.toXY(id, &x, &y);
          double wx = 0.0;
          double wy = 0.0;
          map_.gridToWorld(x, y, &wx, &wy);
          visualization_msgs::Marker t;
          t.header.frame_id = frame_id_;
          t.header.stamp = ros::Time::now();
          t.ns = "cost_text";
          t.id = mid++;
          t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          t.action = visualization_msgs::Marker::ADD;
          t.pose.position.x = wx;
          t.pose.position.y = wy;
          t.pose.position.z = 0.75;
          t.pose.orientation.w = 1.0;
          t.scale.z = res * 0.42;
          t.color.r = t.color.g = t.color.b = 1.0;
          t.color.a = 0.95;
          char buf[96];
          std::snprintf(buf, sizeof(buf), "%.1f\n%d", replayer_.gValues()[id],
                        replayer_.depths()[id]);
          t.text = buf;  // 两行：g / 步数
          arr.markers.push_back(t);
        };
        for (const int id : closed) add_text(id);
        for (const int id : open) add_text(id);
      } else {
        // 手写节流：ROS_WARN_THROTTLE 会走 rosconsole，中文会被打成 "?????"
        const ros::Time now = ros::Time::now();
        if ((now - last_text_warn_).toSec() > 5.0) {
          last_text_warn_ = now;
          say("[警告] 已访问节点 %d 个 > max_text_labels(%d)，暂不画数字。"
              "想看数字请把地图改小，或调大 replay/max_text_labels。",
              total, max_text_labels_);
        }
      }
    }

    // --- 左上角的状态面板 ---
    {
      visualization_msgs::Marker t;
      t.header.frame_id = frame_id_;
      t.header.stamp = ros::Time::now();
      t.ns = "info";
      t.id = 0;
      t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::Marker::ADD;
      t.pose.position.x = map_.originX();
      t.pose.position.y = map_.originY() + map_.height() * res + res * 2.0;
      t.pose.position.z = 1.0;
      t.pose.orientation.w = 1.0;
      t.scale.z = res * 1.5;
      t.color.r = t.color.g = t.color.b = 1.0;
      t.color.a = 1.0;
      t.text = buildInfoText();
      arr.markers.push_back(t);
    }

    marker_pub_.publish(arr);
  }

  std::string buildInfoText() const {
    const SearchStats& st = search_.stats();
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(2);
    os << toString(cfg_.algorithm) << "   " << (cfg_.use_8_connected ? 8 : 4) << "邻域\n";
    os << "保证: " << (search_.guaranteesMinCost() ? "代价最小 " : "")
       << (search_.guaranteesMinSteps() ? "步数最少" : "")
       << (!search_.guaranteesMinCost() && !search_.guaranteesMinSteps() ? "无" : "") << "\n";
    os << "step " << replayer_.cursor() << " / " << replayer_.numSteps() << "   ["
       << (playing_ ? "PLAY" : "PAUSE") << " " << replay_rate_ << "/s]\n";
    const SearchStep* s = replayer_.currentStep();
    if (s != nullptr) {
      os << (s->is_stale_pop ? "取出重复条目(丢弃) (" : "取出 (") << s->cur_x << "," << s->cur_y
         << ")  g=" << s->cur_g << "  步数=" << s->cur_depth << "\n";
      // frontier条目 和 OPEN节点 分开显示：DFS 下两者差得很远，那个差值就是重复条目的量
      os << "frontier条目=" << s->frontier_size_after << "  OPEN节点=" << s->open_size_after
         << "  CLOSED=" << s->closed_size_after << "\n";
    } else {
      os << "初始状态: 只有起点在 frontier 里\n";
    }
    os << toString(st.result);
    if (st.result == SearchResult::kSuccess) {
      os << "  代价=" << st.path_cost << " 格 = " << st.path_length_m << " m"
         << "  步数=" << st.path_steps << "  扩展=" << st.expanded;
    }
    return os.str();
  }

  void publishPath() {
    nav_msgs::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    for (const auto& pt : search_.pathWorld()) {
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pt.first;
      ps.pose.position.y = pt.second;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_.publish(path);
  }

  // =============================================================== rviz 点击
  void onStartClick(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    int x = 0;
    int y = 0;
    if (!map_.worldToGrid(msg->pose.pose.position.x, msg->pose.pose.position.y, &x, &y)) {
      say("[警告] 起点点到地图外了，已钳到边界");
    }
    if (!findNearestFree(x, y, &start_ix_, &start_iy_)) {
      say("[错误] 附近找不到空闲格");
      return;
    }
    say("起点 -> 栅格 (%d, %d)", start_ix_, start_iy_);
    playing_ = true;
    runSearch();
  }

  void onGoalClick(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    int x = 0;
    int y = 0;
    if (!map_.worldToGrid(msg->pose.position.x, msg->pose.position.y, &x, &y)) {
      say("[警告] 终点点到地图外了，已钳到边界");
    }
    if (!findNearestFree(x, y, &goal_ix_, &goal_iy_)) {
      say("[错误] 附近找不到空闲格");
      return;
    }
    say("终点 -> 栅格 (%d, %d)", goal_ix_, goal_iy_);
    playing_ = true;
    runSearch();
  }

  // =============================================================== 服务
  bool trig(const char* cmd, std_srvs::Trigger::Response& res) {
    handleCmd(cmd);
    res.success = true;
    std::ostringstream os;
    os << shortName(cfg_.algorithm) << " step " << replayer_.cursor() << "/"
       << replayer_.numSteps();
    res.message = os.str();
    return true;
  }
  bool srvStep(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("step", r);
  }
  bool srvBack(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("back", r);
  }
  bool srvPlayPause(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("play_pause", r);
  }
  bool srvReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("reset", r);
  }
  bool srvFinish(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("finish", r);
  }
  bool srvReplan(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("replan", r);
  }
  bool srvNewMap(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("new_map", r);
  }
  bool srvBfs(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("bfs", r);
  }
  bool srvDfs(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("dfs", r);
  }
  bool srvDijkstra(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("dijkstra", r);
  }
  bool srvCompare(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    return trig("compare", r);
  }

  // =============================================================== 成员
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber cmd_sub_;
  ros::ServiceServer srv_step_, srv_back_, srv_play_, srv_reset_, srv_finish_, srv_replan_,
      srv_newmap_, srv_bfs_, srv_dfs_, srv_dij_, srv_compare_;
  ros::Timer timer_;

  GridMap2D map_;
  GraphSearch search_;
  SearchConfig cfg_;
  TraceReplayer replayer_;

  std::string frame_id_;
  int map_w_ = 40, map_h_ = 28;
  double map_res_ = 0.5;
  std::string map_type_;
  double map_ratio_ = 0.18;
  int map_num_walls_ = 4, map_wall_gap_ = 2, map_seed_ = 2026, map_inflate_ = 0;
  // req_* = 用户通过参数显式要求的起终点（-1 表示交给程序自动挑）。
  // start_ix_ 等会被 findNearestFree 就地改写，所以原始意图必须单独存一份。
  int req_start_ix_ = -1, req_start_iy_ = -1, req_goal_ix_ = -1, req_goal_iy_ = -1;
  ros::Time last_text_warn_;  // 手写节流用，见文字标签那段
  bool map_border_ = true;

  int start_ix_ = -1, start_iy_ = -1, goal_ix_ = -1, goal_iy_ = -1;

  bool playing_ = true;
  double replay_rate_ = 40.0;
  double step_accum_ = 0.0;
  int idle_ticks_ = 0;
  bool show_cost_text_ = false;
  bool show_tentative_path_ = true;
  bool show_neighbors_ = true;
  bool compare_on_replan_ = true;
  int max_text_labels_ = 500;
};

}  // namespace search_tutorial

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_rviz_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  search_tutorial::SearchRvizNode node(nh, pnh);
  ros::spin();
  return 0;
}
