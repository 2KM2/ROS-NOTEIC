// 40x40 练习包可视化：造地图、调用你写的四个函数、把 steps 回放到 RViz。
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "grid_search_practice/search.h"

namespace grid_search_practice {

class PracticeRvizNode {
 public:
  PracticeRvizNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadParams();
    buildMapConnected();

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("practice/map", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("practice/markers", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("practice/path", 1, true);

    start_sub_ = nh_.subscribe("initialpose", 1, &PracticeRvizNode::onStartClick, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &PracticeRvizNode::onGoalClick, this);
    cmd_sub_ = nh_.subscribe("practice/cmd", 10, &PracticeRvizNode::onCmd, this);

    publishMap();
    runSearch(true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / kTickHz), &PracticeRvizNode::onTimer, this);
    printHelp();
  }

 private:
  static constexpr double kTickHz = 30.0;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GridMap2D map_;
  SearchOptions opt_;
  Algorithm algo_ = Algorithm::kBfs;
  SearchOutput result_;
  int cursor_ = 0;  // 0 = 还没走，1..steps.size() 对应 result_.steps[i-1]
  bool playing_ = false;
  double replay_rate_ = 20.0;
  double step_accum_ = 0.0;
  int idle_ticks_ = 0;

  int map_w_ = 40;
  int map_h_ = 40;
  double map_res_ = 1.0;
  int num_obstacles_ = 45;
  int map_seed_ = 2026;
  int start_ix_ = 0;
  int start_iy_ = 0;
  int goal_ix_ = 39;
  int goal_iy_ = 39;
  std::string frame_id_ = "map";

  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber cmd_sub_;
  ros::Timer timer_;

  static void sayLine(const std::string& text) {
    std::fwrite(text.data(), 1, text.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
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
        "\n============ 40x40 图搜索练习 ============\n"
        "写算法:  src/grid_search_practice/src/your_search.cpp\n"
        "键盘:    rosrun grid_search_practice practice_keyboard\n"
        "  n=下一步  p=播放  1/2/3/4=BFS/DFS/Dijkstra/A*  v=4/8邻域  m=新地图\n"
        "rviz:    2D Pose Estimate 设起点，2D Nav Goal 设终点\n"
        "颜色:    蓝=CLOSED  绿=OPEN  红=当前  橙=最终路径  紫=起点  青绿=终点\n"
        "算法没写时只显示地图和起终点，这是正常的。\n"
        "==========================================");
  }

  void loadParams() {
    pnh_.param<std::string>("frame_id", frame_id_, std::string("map"));
    pnh_.param("map/width", map_w_, 40);
    pnh_.param("map/height", map_h_, 40);
    pnh_.param("map/resolution", map_res_, 1.0);
    pnh_.param("map/num_obstacles", num_obstacles_, 45);
    pnh_.param("map/seed", map_seed_, 2026);
    pnh_.param("planning/start_ix", start_ix_, 0);
    pnh_.param("planning/start_iy", start_iy_, 0);
    pnh_.param("planning/goal_ix", goal_ix_, map_w_ - 1);
    pnh_.param("planning/goal_iy", goal_iy_, map_h_ - 1);

    std::string algo_name;
    pnh_.param<std::string>("search/algorithm", algo_name, std::string("bfs"));
    if (!parseAlgorithm(algo_name, &algo_)) {
      say("[警告] 未知 algorithm '%s'，回退到 bfs", algo_name.c_str());
      algo_ = Algorithm::kBfs;
    }
    pnh_.param("search/use_8_connected", opt_.use_8_connected, false);
    pnh_.param("search/allow_corner_cutting", opt_.allow_corner_cutting, false);
    pnh_.param("replay/rate", replay_rate_, 20.0);
    pnh_.param("replay/auto_play", playing_, false);
  }

  void buildMapConnected() {
    Cell start{start_ix_, start_iy_};
    Cell goal{goal_ix_, goal_iy_};
    if (!makePracticeMap(&map_, map_w_, map_h_, map_res_, num_obstacles_,
                         static_cast<unsigned>(map_seed_), start, goal)) {
      say("[警告] 造图失败，用空地图");
      map_.resize(map_w_, map_h_, map_res_, 0.0, 0.0);
    }
    if (!map_.inside(start_ix_, start_iy_) || map_.isOccupied(start_ix_, start_iy_)) {
      start_ix_ = 0;
      start_iy_ = 0;
    }
    if (!map_.inside(goal_ix_, goal_iy_) || map_.isOccupied(goal_ix_, goal_iy_)) {
      goal_ix_ = map_.width() - 1;
      goal_iy_ = map_.height() - 1;
    }
    say("[地图] %dx%d  障碍 %d 格  seed=%d  起点(%d,%d) 终点(%d,%d)", map_.width(), map_.height(),
        countOccupied(map_), map_seed_, start_ix_, start_iy_, goal_ix_, goal_iy_);
  }

  void publishMap() {
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = map_.resolution();
    grid.info.width = static_cast<unsigned>(map_.width());
    grid.info.height = static_cast<unsigned>(map_.height());
    grid.info.origin.position.x = map_.originX();
    grid.info.origin.position.y = map_.originY();
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(map_.data().begin(), map_.data().end());
    map_pub_.publish(grid);
  }

  int numSteps() const { return static_cast<int>(result_.steps.size()); }
  bool atEnd() const { return cursor_ >= numSteps(); }

  void runSearch(bool rewind) {
    const Cell start{start_ix_, start_iy_};
    const Cell goal{goal_ix_, goal_iy_};
    result_ = run(algo_, map_, start, goal, opt_);
    if (rewind || playing_) cursor_ = 0;
    else cursor_ = numSteps();
    step_accum_ = 0.0;

    say("\n[%s] %s | %d邻域 | 扩展 %d  入队 %d  记录步数 %d | 成功=%s 代价=%.3f 路径步数=%d",
        shortName(algo_), toString(algo_), opt_.use_8_connected ? 8 : 4, result_.expanded, result_.pushes,
        numSteps(), result_.success ? "是" : "否", result_.path_cost, result_.path_steps);
    if (!result_.success && result_.steps.empty()) {
      say("       还没实现：去 src/grid_search_practice/src/your_search.cpp 写 %s()",
          shortName(algo_));
    }
    if (result_.success && !result_.path.empty()) {
      std::ostringstream os;
      os << "       路径: ";
      for (std::size_t i = 0; i < result_.path.size(); ++i) {
        if (i) os << " -> ";
        os << "(" << result_.path[i].x << "," << result_.path[i].y << ")";
      }
      sayLine(os.str());
    }
    publishPath();
    publishVis();
  }

  void logCurrentStep() {
    if (cursor_ <= 0) {
      say("[第 0 步] 初始状态，按 n 开始扩展");
      return;
    }
    const SearchStep& s = result_.steps[static_cast<std::size_t>(cursor_ - 1)];
    say("[第 %d/%d 步] 取出 (%d,%d)  g=%.3f  步数=%d  OPEN=%zu  CLOSED=%zu  邻居=%zu", cursor_,
        numSteps(), s.current.x, s.current.y, s.current_g, s.current_depth, s.open.size(), s.closed.size(),
        s.neighbors.size());
  }

  void onTimer(const ros::TimerEvent&) {
    if (playing_ && cursor_ < numSteps()) {
      step_accum_ += replay_rate_ / kTickHz;
      int n = static_cast<int>(step_accum_);
      step_accum_ -= n;
      bool moved = false;
      while (n-- > 0 && cursor_ < numSteps()) {
        ++cursor_;
        moved = true;
      }
      if (moved) publishVis();
      if (atEnd() && playing_) {
        playing_ = false;
        say("[回放] 播放完毕，共 %d 步", numSteps());
        publishVis();
      }
    } else if ((++idle_ticks_ % static_cast<int>(kTickHz * 2)) == 0) {
      publishVis();
    }
  }

  void onCmd(const std_msgs::String::ConstPtr& msg) { handleCmd(msg->data); }

  void setAlgorithm(Algorithm a) {
    algo_ = a;
    say("\n[算法] 切换到 %s", toString(a));
    runSearch(true);
  }

  void handleCmd(const std::string& cmd) {
    if (cmd == "step") {
      playing_ = false;
      if (cursor_ < numSteps()) {
        ++cursor_;
        logCurrentStep();
      } else {
        say("[回放] 已经在最后一步 (%d/%d)", cursor_, numSteps());
      }
    } else if (cmd == "back") {
      playing_ = false;
      cursor_ = std::max(0, cursor_ - 1);
      logCurrentStep();
    } else if (cmd == "play_pause" || cmd == "play") {
      if (atEnd() && !playing_) cursor_ = 0;
      playing_ = !playing_;
      say("[回放] %s (%d/%d, %.1f 步/秒)", playing_ ? "播放" : "暂停", cursor_, numSteps(), replay_rate_);
    } else if (cmd == "reset") {
      playing_ = false;
      cursor_ = 0;
      say("[回放] 回到第 0 步");
    } else if (cmd == "finish") {
      playing_ = false;
      cursor_ = numSteps();
      say("[回放] 跳到最后一步 (%d)", numSteps());
    } else if (cmd == "faster") {
      replay_rate_ = std::min(replay_rate_ * 1.6, 20000.0);
      say("[回放] 速度 %.1f 步/秒", replay_rate_);
    } else if (cmd == "slower") {
      replay_rate_ = std::max(replay_rate_ / 1.6, 0.5);
      say("[回放] 速度 %.1f 步/秒", replay_rate_);
    } else if (cmd == "replan") {
      runSearch(true);
      return;
    } else if (cmd == "new_map") {
      map_seed_ += 1;
      buildMapConnected();
      publishMap();
      runSearch(true);
      return;
    } else if (cmd == "algo_bfs") {
      setAlgorithm(Algorithm::kBfs);
      return;
    } else if (cmd == "algo_dfs") {
      setAlgorithm(Algorithm::kDfs);
      return;
    } else if (cmd == "algo_dijkstra") {
      setAlgorithm(Algorithm::kDijkstra);
      return;
    } else if (cmd == "algo_astar") {
      setAlgorithm(Algorithm::kAstar);
      return;
    } else if (cmd == "conn") {
      opt_.use_8_connected = !opt_.use_8_connected;
      say("\n[邻域] 切换到 %d 邻域", opt_.use_8_connected ? 8 : 4);
      runSearch(true);
      return;
    } else if (cmd == "compare") {
      logCompare();
      return;
    } else {
      say("[警告] 未知命令 '%s'", cmd.c_str());
      return;
    }
    publishPath();
    publishVis();
  }

  void logCompare() {
    sayLine("       ---- 四算法对比（没写完的会显示 未实现）----");
    say("       %-9s %8s %8s %8s %9s", "algo", "ok", "expanded", "steps", "cost");
    for (const Algorithm a :
         {Algorithm::kBfs, Algorithm::kDfs, Algorithm::kDijkstra, Algorithm::kAstar}) {
      SearchOutput o = run(a, map_, Cell{start_ix_, start_iy_}, Cell{goal_ix_, goal_iy_}, opt_);
      if (!o.success && o.steps.empty()) {
        say("       %-9s  未实现", shortName(a));
      } else {
        say("       %-9s %8s %8d %8d %9.3f", shortName(a), o.success ? "yes" : "no", o.expanded,
            o.path_steps, o.path_cost);
      }
    }
  }

  bool worldToGrid(double wx, double wy, int* x, int* y) const {
    return map_.worldToGrid(wx, wy, x, y);
  }

  void onStartClick(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    int x = 0;
    int y = 0;
    if (!worldToGrid(msg->pose.pose.position.x, msg->pose.pose.position.y, &x, &y) ||
        map_.isOccupied(x, y)) {
      say("[起点] 点到障碍或地图外，忽略");
      return;
    }
    start_ix_ = x;
    start_iy_ = y;
    say("[起点] (%d,%d)", x, y);
    runSearch(true);
  }

  void onGoalClick(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    int x = 0;
    int y = 0;
    if (!worldToGrid(msg->pose.position.x, msg->pose.position.y, &x, &y) || map_.isOccupied(x, y)) {
      say("[终点] 点到障碍或地图外，忽略");
      return;
    }
    goal_ix_ = x;
    goal_iy_ = y;
    say("[终点] (%d,%d)", x, y);
    runSearch(true);
  }

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

  void addCell(visualization_msgs::Marker* m, int x, int y, double z) const {
    double wx = 0.0;
    double wy = 0.0;
    map_.gridToWorld(x, y, &wx, &wy);
    geometry_msgs::Point p;
    p.x = wx;
    p.y = wy;
    p.z = z;
    m->points.push_back(p);
  }

  void publishPath() {
    nav_msgs::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    if (atEnd() && result_.success) {
      for (const Cell& c : result_.path) {
        geometry_msgs::PoseStamped ps;
        ps.header = path.header;
        double wx = 0.0;
        double wy = 0.0;
        map_.gridToWorld(c.x, c.y, &wx, &wy);
        ps.pose.position.x = wx;
        ps.pose.position.y = wy;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
      }
    }
    path_pub_.publish(path);
  }

  void publishVis() {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = frame_id_;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    const SearchStep* step = (cursor_ > 0 && cursor_ <= numSteps())
                                 ? &result_.steps[static_cast<std::size_t>(cursor_ - 1)]
                                 : nullptr;

    {
      auto m = makeCubeList("closed", 0.05, 0.15, 0.35, 0.95, 0.55);
      if (step) {
        for (const Cell& c : step->closed) addCell(&m, c.x, c.y, 0.03);
      }
      arr.markers.push_back(m);
    }
    {
      auto m = makeCubeList("open", 0.09, 0.15, 0.9, 0.25, 0.6);
      if (step) {
        for (const Cell& c : step->open) addCell(&m, c.x, c.y, 0.06);
      }
      arr.markers.push_back(m);
    }
    {
      auto m = makeCubeList("neighbors", 0.14, 1.0, 0.85, 0.1, 0.45, 0.55);
      if (step) {
        for (const Cell& c : step->neighbors) addCell(&m, c.x, c.y, 0.12);
      }
      arr.markers.push_back(m);
    }
    {
      auto m = makeCubeList("current", 0.6, 1.0, 0.1, 0.1, 0.95);
      if (step) addCell(&m, step->current.x, step->current.y, 0.3);
      arr.markers.push_back(m);
    }

    {
      visualization_msgs::Marker line;
      line.header.frame_id = frame_id_;
      line.header.stamp = ros::Time::now();
      line.ns = "path";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = map_.resolution() * 0.35;
      line.color.r = 1.0;
      line.color.g = 0.55;
      line.color.b = 0.0;
      line.color.a = 1.0;
      if (atEnd() && result_.success) {
        for (const Cell& c : result_.path) {
          double wx = 0.0;
          double wy = 0.0;
          map_.gridToWorld(c.x, c.y, &wx, &wy);
          geometry_msgs::Point p;
          p.x = wx;
          p.y = wy;
          p.z = 0.4;
          line.points.push_back(p);
        }
      }
      if (line.points.size() >= 2) arr.markers.push_back(line);
    }

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
      m.scale.x = m.scale.y = m.scale.z = map_.resolution() * 1.4;
      m.color.r = (k == 0) ? 0.7 : 0.0;
      m.color.g = (k == 0) ? 0.2 : 1.0;
      m.color.b = (k == 0) ? 1.0 : 0.6;
      m.color.a = 0.95;
      arr.markers.push_back(m);
    }

    {
      visualization_msgs::Marker t;
      t.header.frame_id = frame_id_;
      t.header.stamp = ros::Time::now();
      t.ns = "info";
      t.id = 0;
      t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::Marker::ADD;
      t.pose.position.x = map_.originX() + map_.width() * map_.resolution() * 0.5;
      t.pose.position.y = map_.originY() + map_.height() * map_.resolution() + map_.resolution() * 2.0;
      t.pose.position.z = 1.0;
      t.pose.orientation.w = 1.0;
      t.scale.z = map_.resolution() * 1.4;
      t.color.r = t.color.g = t.color.b = 1.0;
      t.color.a = 1.0;
      std::ostringstream os;
      os.setf(std::ios::fixed);
      os.precision(2);
      os << shortName(algo_) << "   " << (opt_.use_8_connected ? 8 : 4) << "邻域\n";
      os << "step " << cursor_ << " / " << numSteps() << "  [" << (playing_ ? "PLAY" : "PAUSE") << "]\n";
      if (result_.success) {
        os << "代价=" << result_.path_cost << "  步数=" << result_.path_steps;
      } else if (result_.steps.empty()) {
        os << "算法未实现  写 your_search.cpp";
      } else {
        os << "无路径";
      }
      t.text = os.str();
      arr.markers.push_back(t);
    }

    marker_pub_.publish(arr);
  }
};

}  // namespace grid_search_practice

int main(int argc, char** argv) {
  ros::init(argc, argv, "practice_rviz_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  grid_search_practice::PracticeRvizNode node(nh, pnh);
  ros::spin();
  return 0;
}
