#include "astar_tutorial/grid_map_2d.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace astar_tutorial {

GridMap2D::GridMap2D(int width, int height, double resolution, double origin_x, double origin_y) {
  resize(width, height, resolution, origin_x, origin_y);
}

void GridMap2D::resize(int width, int height, double resolution, double origin_x, double origin_y) {
  width_ = std::max(width, 0);
  height_ = std::max(height, 0);
  resolution_ = resolution > 0.0 ? resolution : 1.0;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  data_.assign(static_cast<std::size_t>(width_) * height_, kFree);
}

void GridMap2D::clear() { std::fill(data_.begin(), data_.end(), kFree); }

bool GridMap2D::worldToGrid(double wx, double wy, int* x, int* y) const {
  // floor 而不是 (int) 强转：wx-origin 为负时强转是向 0 取整，会把 -0.3 格算成第 0 格。
  const int ix = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  const int iy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  const bool ok = (ix >= 0 && ix < width_ && iy >= 0 && iy < height_);
  *x = std::min(std::max(ix, 0), std::max(width_ - 1, 0));
  *y = std::min(std::max(iy, 0), std::max(height_ - 1, 0));
  return ok;
}

void GridMap2D::gridToWorld(int x, int y, double* wx, double* wy) const {
  // +0.5 = 取格子中心。少了这个，marker 会整体偏半格，看起来"路径贴着障碍边"。
  *wx = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
  *wy = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
}

void GridMap2D::addBorder() {
  for (int x = 0; x < width_; ++x) {
    setOccupied(x, 0);
    setOccupied(x, height_ - 1);
  }
  for (int y = 0; y < height_; ++y) {
    setOccupied(0, y);
    setOccupied(width_ - 1, y);
  }
}

void GridMap2D::fillRandomObstacles(double ratio, unsigned seed, int block) {
  if (numCells() == 0) return;
  block = std::max(block, 1);
  ratio = std::min(std::max(ratio, 0.0), 0.9);

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dx(0, std::max(width_ - 1, 0));
  std::uniform_int_distribution<int> dy(0, std::max(height_ - 1, 0));

  const int target = static_cast<int>(ratio * numCells());
  int placed = 0;
  // guard：ratio 高时随机撒点会越来越难命中空格，加个尝试次数上限免得死循环
  const int max_tries = 200 * std::max(target, 1);
  for (int tries = 0; placed < target && tries < max_tries; ++tries) {
    const int ox = dx(rng);
    const int oy = dy(rng);
    for (int j = 0; j < block; ++j) {
      for (int i = 0; i < block; ++i) {
        const int x = ox + i;
        const int y = oy + j;
        if (!inside(x, y)) continue;
        if (data_[toId(x, y)] >= kOccupied) continue;
        data_[toId(x, y)] = kOccupied;
        ++placed;
      }
    }
  }
}

void GridMap2D::fillWalls(int num_walls, unsigned seed, int gap_half_width) {
  if (width_ < 3 || height_ < 3 || num_walls <= 0) return;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> gap_pos(1, height_ - 2);

  for (int w = 1; w <= num_walls; ++w) {
    const int x = w * width_ / (num_walls + 1);
    if (x <= 0 || x >= width_) continue;
    const int gap = gap_pos(rng);
    for (int y = 0; y < height_; ++y) {
      if (std::abs(y - gap) <= gap_half_width) {
        // 留个口子，不然无解。注意这里是**强制置空**而不是简单 continue：
        // 如果先撒了随机障碍再造墙，`continue` 会把已经堵在口子上的障碍留下来，
        // 于是唯一的通道被封死、地图变成无解。
        setOccupied(x, y, false);
        continue;
      }
      setOccupied(x, y);
    }
  }
}

void GridMap2D::fillMaze(unsigned seed) {
  // 经典"打洞"式迷宫：先全填实，再从 (1,1) 出发每次跳两格挖通道。
  // 通道格在奇数坐标上，墙格在偶数坐标上。
  if (width_ < 3 || height_ < 3) return;
  std::fill(data_.begin(), data_.end(), kOccupied);

  std::mt19937 rng(seed);
  std::vector<std::pair<int, int>> stack;
  stack.emplace_back(1, 1);
  setOccupied(1, 1, false);

  const int dirs[4][2] = {{2, 0}, {-2, 0}, {0, 2}, {0, -2}};
  while (!stack.empty()) {
    const auto cur = stack.back();
    std::vector<int> candidates;
    for (int d = 0; d < 4; ++d) {
      const int nx = cur.first + dirs[d][0];
      const int ny = cur.second + dirs[d][1];
      if (nx <= 0 || nx >= width_ - 1 || ny <= 0 || ny >= height_ - 1) continue;
      if (isFree(nx, ny)) continue;  // 已经挖通过
      candidates.push_back(d);
    }
    if (candidates.empty()) {
      stack.pop_back();
      continue;
    }
    std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
    const int d = candidates[pick(rng)];
    const int nx = cur.first + dirs[d][0];
    const int ny = cur.second + dirs[d][1];
    setOccupied(cur.first + dirs[d][0] / 2, cur.second + dirs[d][1] / 2, false);  // 打掉中间的墙
    setOccupied(nx, ny, false);
    stack.emplace_back(nx, ny);
  }
}

void GridMap2D::inflate(int radius) {
  if (radius <= 0) return;
  const std::vector<uint8_t> src = data_;  // 必须在副本上判断，否则会"越膨胀越大"
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (src[toId(x, y)] < kOccupied) continue;
      for (int j = -radius; j <= radius; ++j) {
        for (int i = -radius; i <= radius; ++i) {
          if (inside(x + i, y + j)) data_[toId(x + i, y + j)] = kOccupied;
        }
      }
    }
  }
}

void GridMap2D::loadFromAscii(const std::vector<std::string>& rows, double resolution,
                              double origin_x, double origin_y) {
  int w = 0;
  for (const auto& r : rows) w = std::max<int>(w, static_cast<int>(r.size()));
  resize(w, static_cast<int>(rows.size()), resolution, origin_x, origin_y);

  for (int r = 0; r < static_cast<int>(rows.size()); ++r) {
    // 图案第 0 行显示在最上面，而栅格 y 越大越上 -> 上下翻一下
    const int y = height_ - 1 - r;
    for (int x = 0; x < static_cast<int>(rows[r].size()); ++x) {
      const char c = rows[r][x];
      if (c == '#' || c == '1' || c == 'x' || c == 'X' || c == '*') setOccupied(x, y);
    }
  }
}

// 纯洪水填充的连通性判断。故意写得和 astar.cpp 的 search() 结构一致，方便逐段对照：
//   * 这里的 stack 对应那边的 priority_queue —— 区别只是取节点的顺序（后进先出 vs f 最小优先）；
//   * 这里没有 g/h/f，因为它只问"通不通"，不问"多远"；
//   * 这里 visited 一置上就永不回头，对应那边的 CLOSED。
// 换成队列（先进先出）就是 BFS，顺带能得到"步数最少"的路；再把步数换成 g、加上 h，就是 A*。
bool GridMap2D::isConnected(int x0, int y0, int x1, int y1, bool use_8_connected,
                            bool allow_corner_cutting) const {
  if (!isFree(x0, y0) || !isFree(x1, y1)) return false;
  if (x0 == x1 && y0 == y1) return true;

  static const int kDX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const int num_dirs = use_8_connected ? 8 : 4;
  const int goal_id = toId(x1, y1);

  std::vector<char> visited(static_cast<std::size_t>(numCells()), 0);
  std::vector<int> stack;           // 用栈就是 DFS，用队列就是 BFS；只判连通，两者等价
  stack.reserve(static_cast<std::size_t>(numCells()));
  stack.push_back(toId(x0, y0));
  visited[static_cast<std::size_t>(stack.back())] = 1;

  while (!stack.empty()) {
    const int cur = stack.back();
    stack.pop_back();
    int cx = 0, cy = 0;
    toXY(cur, &cx, &cy);
    for (int d = 0; d < num_dirs; ++d) {
      const int nx = cx + kDX[d];
      const int ny = cy + kDY[d];
      if (!isFree(nx, ny)) continue;           // 越界和障碍一起挡掉
      // 和 astar.cpp 里完全相同的斜穿墙角规则 —— 两边必须都空才允许斜走。
      // 少了这一段，这里会说"连通"而 A* 说"无解"，两个模块打起来。
      if (d >= 4 && !allow_corner_cutting) {
        if (!isFree(nx, cy) || !isFree(cx, ny)) continue;
      }
      const int nid = toId(nx, ny);
      if (visited[static_cast<std::size_t>(nid)]) continue;
      if (nid == goal_id) return true;
      visited[static_cast<std::size_t>(nid)] = 1;
      stack.push_back(nid);
    }
  }
  return false;   // 整个连通分量都走完了也没碰到终点
}

std::string GridMap2D::toAscii(const std::vector<char>& overlay) const {
  std::string out;
  out.reserve(static_cast<std::size_t>(width_ + 1) * height_ * 2);
  for (int r = 0; r < height_; ++r) {
    const int y = height_ - 1 - r;  // 从上往下打印
    for (int x = 0; x < width_; ++x) {
      const int id = toId(x, y);
      char c = (data_[id] >= kOccupied) ? '#' : '.';
      if (!overlay.empty() && id < static_cast<int>(overlay.size()) && overlay[id] != '\0') {
        c = overlay[id];
      }
      out.push_back(c);
      out.push_back(' ');  // 加空格，终端里格子才接近正方形
    }
    out.push_back('\n');
  }
  return out;
}

}  // namespace astar_tutorial
