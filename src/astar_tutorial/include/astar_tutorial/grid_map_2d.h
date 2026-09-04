// 二维栅格地图 —— A* 的"地图"部分，和算法本身解耦。
//
// 这里刻意只依赖 STL，不碰 ROS：
//   * 想理解算法时，可以用 g++ 单独编译，不用起 roscore；
//   * ROS 节点只负责把这张地图翻译成 nav_msgs/OccupancyGrid 给 rviz 看。
//
// 三套坐标要分清（A* 里绝大多数 bug 都出在这三者的换算上）：
//   1. 栅格索引 (x, y)  —— 整数，x ∈ [0, width), y ∈ [0, height)
//   2. 一维下标 id      —— id = y * width + x，把二维数组压平成 vector
//   3. 世界坐标 (wx, wy)—— 米，rviz 里看到的坐标
#ifndef ASTAR_TUTORIAL_GRID_MAP_2D_H
#define ASTAR_TUTORIAL_GRID_MAP_2D_H

#include <cstdint>
#include <string>
#include <vector>

namespace astar_tutorial {

class GridMap2D {
 public:
  // 栅格取值：0 = 空闲，100 = 占据（和 OccupancyGrid 的约定一致，方便直接发给 rviz）
  static constexpr uint8_t kFree = 0;
  static constexpr uint8_t kOccupied = 100;

  GridMap2D() = default;
  GridMap2D(int width, int height, double resolution, double origin_x, double origin_y);

  // 重新分配大小并全部置空闲
  void resize(int width, int height, double resolution, double origin_x, double origin_y);
  void clear();

  int width() const { return width_; }
  int height() const { return height_; }
  int numCells() const { return width_ * height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  // ---- 索引换算 ----
  bool inside(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
  }
  int toId(int x, int y) const { return y * width_ + x; }
  void toXY(int id, int* x, int* y) const {
    *x = id % width_;
    *y = id / width_;
  }

  // ---- 占据查询 ----
  // 注意：越界一律当成"占据"。这样 A* 里判断邻居时只需要一次 isOccupied 调用，
  // 但为了让 trace 能区分"撞墙"和"出界"，搜索代码里仍然先单独判 inside()。
  bool isOccupied(int x, int y) const {
    if (!inside(x, y)) return true;
    return data_[toId(x, y)] >= kOccupied;
  }
  bool isFree(int x, int y) const { return !isOccupied(x, y); }

  void setOccupied(int x, int y, bool occupied = true) {
    if (!inside(x, y)) return;
    data_[toId(x, y)] = occupied ? kOccupied : kFree;
  }

  const std::vector<uint8_t>& data() const { return data_; }

  // ---- 世界坐标 <-> 栅格 ----
  // 落在地图外时返回 false（x/y 仍会被写成钳位后的值，方便"点歪了也能用"）
  bool worldToGrid(double wx, double wy, int* x, int* y) const;
  // 返回栅格中心的世界坐标（+0.5 是中心而不是左下角，画 marker 时才不会偏半格）
  void gridToWorld(int x, int y, double* wx, double* wy) const;

  // ---- 造地图（教学 / demo 用）----
  // 随机撒方块障碍。同一个 seed 结果完全一样，方便反复对比不同启发函数的表现。
  // block: 每个障碍是 block×block 的方块；ratio: 目标占据率（近似）
  void fillRandomObstacles(double ratio, unsigned seed, int block = 2);
  // 竖着的几道墙，每道墙上留一个随机开口 —— 最能体现 A* 和 Dijkstra 的差别
  void fillWalls(int num_walls, unsigned seed, int gap_half_width = 2);
  // 递归分割迷宫（墙占一格，通道占一格）
  void fillMaze(unsigned seed);
  // 把障碍膨胀 radius 格（机器人有体积时的常规做法）
  void inflate(int radius);
  // 四周加一圈墙
  void addBorder();

  // 从 ASCII 图案读地图：'#' / '1' / 'x' 视为障碍，其余空闲。
  // 第 0 行是图案的第一行，会被放到 y = height-1（即"上面"），跟看图习惯一致。
  void loadFromAscii(const std::vector<std::string>& rows, double resolution,
                     double origin_x, double origin_y);

  // 打印成 ASCII（overlay 里可以塞 'S'、'G'、'*' 之类的覆盖字符）
  std::string toAscii(const std::vector<char>& overlay = {}) const;

  // 洪水填充判断两格是否连通 —— 只回答"通不通"，不关心多远。
  // 用途：造完随机地图先确认起终点确实连通，别让 demo 一上来就显示"无解"。
  // 和 A* 的关系：这就是把 A* 的 g/h/f、优先队列全部拿掉之后剩下的 BFS 骨架，
  // 对照着读能看清"启发式搜索"到底在朴素搜索之上加了什么。
  // 两个开关必须和 AStarConfig 里的同名项保持一致，否则答案会不一样：
  // 禁止斜穿墙角时，"只能从两个障碍的夹缝里斜着挤过去"的通道是走不通的，
  // 于是同一张地图在 allow_corner_cutting=true/false 下**连通性都不同**（不只是代价不同）。
  bool isConnected(int x0, int y0, int x1, int y1, bool use_8_connected = true,
                   bool allow_corner_cutting = false) const;

 private:
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 1.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::vector<uint8_t> data_;
};

}  // namespace astar_tutorial

#endif  // ASTAR_TUTORIAL_GRID_MAP_2D_H
