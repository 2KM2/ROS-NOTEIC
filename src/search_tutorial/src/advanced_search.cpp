#include "search_tutorial/advanced_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace search_tutorial {
namespace {

constexpr double kEps = 1e-9;

bool lessKey(double a1, double a2, double b1, double b2) {
  return a1 < b1 - kEps || (std::fabs(a1 - b1) <= kEps && a2 < b2 - kEps);
}

bool sameValue(double a, double b) {
  return (std::isinf(a) && std::isinf(b)) || std::fabs(a - b) <= kEps;
}

int sign(int v) { return (v > 0) - (v < 0); }

}  // namespace

bool DStarLite::QueueGreater::operator()(const QueueItem& a, const QueueItem& b) const {
  if (std::fabs(a.key.first - b.key.first) > kEps) return a.key.first > b.key.first;
  if (std::fabs(a.key.second - b.key.second) > kEps) return a.key.second > b.key.second;
  return a.id > b.id;
}

void DStarLite::setConnectivity(bool use_8_connected, bool allow_corner_cutting) {
  use_8_connected_ = use_8_connected;
  allow_corner_cutting_ = allow_corner_cutting;
}

bool DStarLite::validEndpoint(int id) const {
  if (map_ == nullptr || id < 0 || id >= map_->numCells()) return false;
  int x = 0, y = 0;
  map_->toXY(id, &x, &y);
  return map_->isFree(x, y);
}

double DStarLite::heuristic(int a, int b) const {
  int ax = 0, ay = 0, bx = 0, by = 0;
  map_->toXY(a, &ax, &ay);
  map_->toXY(b, &bx, &by);
  const int dx = std::abs(ax - bx);
  const int dy = std::abs(ay - by);
  if (!use_8_connected_) return static_cast<double>(dx + dy);
  return static_cast<double>(std::max(dx, dy)) + (kSqrt2 - 1.0) * std::min(dx, dy);
}

double DStarLite::edgeCost(int from, int to) const {
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  map_->toXY(from, &x0, &y0);
  map_->toXY(to, &x1, &y1);
  if (!map_->isFree(x0, y0) || !map_->isFree(x1, y1)) return kInf;
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  if (dx + dy == 1) return 1.0;
  if (!use_8_connected_ || dx != 1 || dy != 1) return kInf;
  if (!allow_corner_cutting_ && (!map_->isFree(x0, y1) || !map_->isFree(x1, y0))) return kInf;
  return kSqrt2;
}

std::vector<int> DStarLite::neighbors(int id) const {
  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  int x = 0, y = 0;
  map_->toXY(id, &x, &y);
  std::vector<int> out;
  const int n = use_8_connected_ ? 8 : 4;
  out.reserve(n);
  for (int d = 0; d < n; ++d) {
    const int nx = x + kDx[d];
    const int ny = y + kDy[d];
    if (map_->inside(nx, ny)) out.push_back(map_->toId(nx, ny));
  }
  return out;
}

DStarLite::Key DStarLite::calculateKey(int id) const {
  const double best = std::min(g_[id], rhs_[id]);
  return {best + heuristic(start_id_, id) + km_, best};
}

void DStarLite::push(int id) {
  open_.push({id, calculateKey(id), queue_version_[id]});
  ++stats_.pushes;
}

void DStarLite::updateVertex(int id) {
  if (id != goal_id_) {
    rhs_[id] = kInf;
    for (const int next : neighbors(id)) {
      rhs_[id] = std::min(rhs_[id], edgeCost(id, next) + g_[next]);
    }
  }
  ++queue_version_[id];  // 懒惰失效堆中的旧条目
  if (!sameValue(g_[id], rhs_[id])) push(id);
}

SearchResult DStarLite::initialize(int sx, int sy, int gx, int gy) {
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = AdvancedSearchStats{};
  if (map_ == nullptr || map_->numCells() == 0 || !map_->inside(sx, sy) ||
      !map_->inside(gx, gy) || !map_->isFree(sx, sy) || !map_->isFree(gx, gy)) {
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }
  start_id_ = map_->toId(sx, sy);
  goal_id_ = map_->toId(gx, gy);
  last_start_id_ = start_id_;
  km_ = 0.0;
  g_.assign(map_->numCells(), kInf);
  rhs_.assign(map_->numCells(), kInf);
  queue_version_.assign(map_->numCells(), 0);
  open_ = decltype(open_)();
  rhs_[goal_id_] = 0.0;
  push(goal_id_);
  const SearchResult result = computeShortestPath();
  const auto t1 = std::chrono::steady_clock::now();
  finalizeStats(std::chrono::duration<double, std::milli>(t1 - t0).count());
  return result;
}

SearchResult DStarLite::replan() {
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = AdvancedSearchStats{};
  if (!validEndpoint(start_id_) || !validEndpoint(goal_id_) || g_.size() != static_cast<std::size_t>(map_->numCells())) {
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }
  const SearchResult result = computeShortestPath();
  const auto t1 = std::chrono::steady_clock::now();
  finalizeStats(std::chrono::duration<double, std::milli>(t1 - t0).count());
  return result;
}

SearchResult DStarLite::moveStart(int sx, int sy) {
  if (map_ == nullptr || !map_->inside(sx, sy) || !map_->isFree(sx, sy) || start_id_ < 0) {
    stats_ = AdvancedSearchStats{};
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }
  const int next_start = map_->toId(sx, sy);
  km_ += heuristic(last_start_id_, next_start);
  start_id_ = next_start;
  last_start_id_ = next_start;
  return replan();
}

SearchResult DStarLite::updateCell(int x, int y, bool occupied) {
  if (map_ == nullptr || !map_->inside(x, y) || start_id_ < 0 || map_->toId(x, y) == goal_id_) {
    stats_ = AdvancedSearchStats{};
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }
  map_->setOccupied(x, y, occupied);
  // 一格变化会影响以周围节点为起点的边；禁止穿角时还会影响斜边合法性。
  for (int yy = y - 1; yy <= y + 1; ++yy) {
    for (int xx = x - 1; xx <= x + 1; ++xx) {
      if (map_->inside(xx, yy)) updateVertex(map_->toId(xx, yy));
    }
  }
  return replan();
}

SearchResult DStarLite::computeShortestPath() {
  while (!open_.empty()) {
    const QueueItem top = open_.top();
    const Key start_key = calculateKey(start_id_);
    if (!lessKey(top.key.first, top.key.second, start_key.first, start_key.second) &&
        sameValue(rhs_[start_id_], g_[start_id_])) {
      break;
    }
    open_.pop();
    ++stats_.queue_pops;
    if (top.version != queue_version_[top.id]) continue;
    const Key fresh = calculateKey(top.id);
    if (lessKey(top.key.first, top.key.second, fresh.first, fresh.second)) {
      ++queue_version_[top.id];
      push(top.id);
      continue;
    }
    // 懒惰删除：同一节点可能有多份 key，只处理仍与当前 key 一致的条目。
    if (lessKey(fresh.first, fresh.second, top.key.first, top.key.second)) continue;
    ++stats_.expanded;
    if (g_[top.id] > rhs_[top.id]) {
      g_[top.id] = rhs_[top.id];
      for (const int pred : neighbors(top.id)) updateVertex(pred);
    } else {
      g_[top.id] = kInf;
      updateVertex(top.id);
      for (const int pred : neighbors(top.id)) updateVertex(pred);
    }
  }
  stats_.result = std::isfinite(g_[start_id_]) ? SearchResult::kSuccess
                                               : SearchResult::kFailedNoPath;
  return stats_.result;
}

std::vector<int> DStarLite::pathIds() const {
  std::vector<int> path;
  if (stats_.result != SearchResult::kSuccess || !validEndpoint(start_id_)) return path;
  int cur = start_id_;
  path.push_back(cur);
  std::vector<char> seen(map_->numCells(), 0);
  seen[cur] = 1;
  while (cur != goal_id_) {
    int best_id = -1;
    double best = kInf;
    for (const int next : neighbors(cur)) {
      const double candidate = edgeCost(cur, next) + g_[next];
      if (candidate < best - kEps ||
          (std::fabs(candidate - best) <= kEps && next < best_id)) {
        best = candidate;
        best_id = next;
      }
    }
    if (best_id < 0 || !std::isfinite(best) || seen[best_id]) return {};
    cur = best_id;
    seen[cur] = 1;
    path.push_back(cur);
  }
  return path;
}

std::vector<std::pair<int, int>> DStarLite::pathGrid() const {
  std::vector<std::pair<int, int>> out;
  for (const int id : pathIds()) {
    int x = 0, y = 0;
    map_->toXY(id, &x, &y);
    out.emplace_back(x, y);
  }
  return out;
}

void DStarLite::finalizeStats(double elapsed_ms) {
  stats_.time_ms = elapsed_ms;
  const std::vector<int> path = pathIds();
  if (stats_.result != SearchResult::kSuccess || path.empty()) return;
  stats_.path_steps = static_cast<int>(path.size()) - 1;
  for (std::size_t i = 1; i < path.size(); ++i) stats_.path_cost += edgeCost(path[i - 1], path[i]);
}

bool JumpPointSearch::OpenGreater::operator()(const OpenItem& a, const OpenItem& b) const {
  if (std::fabs(a.f - b.f) > kEps) return a.f > b.f;
  if (std::fabs(a.g - b.g) > kEps) return a.g < b.g;
  return a.id > b.id;
}

bool JumpPointSearch::walkable(int x, int y) const { return map_ != nullptr && map_->isFree(x, y); }

double JumpPointSearch::heuristic(int id) const {
  int x = 0, y = 0, gx = 0, gy = 0;
  map_->toXY(id, &x, &y);
  map_->toXY(goal_id_, &gx, &gy);
  const int dx = std::abs(x - gx);
  const int dy = std::abs(y - gy);
  return std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy);
}

bool JumpPointSearch::hasForcedNeighbor(int x, int y, int dx, int dy) const {
  if (dx != 0 && dy != 0) {
    return (!walkable(x - dx, y) && walkable(x - dx, y + dy)) ||
           (!walkable(x, y - dy) && walkable(x + dx, y - dy));
  }
  if (dx != 0) {
    return (!walkable(x, y + 1) && walkable(x + dx, y + 1)) ||
           (!walkable(x, y - 1) && walkable(x + dx, y - 1));
  }
  return (!walkable(x + 1, y) && walkable(x + 1, y + dy)) ||
         (!walkable(x - 1, y) && walkable(x - 1, y + dy));
}

int JumpPointSearch::jump(int x, int y, int dx, int dy) const {
  const int nx = x + dx;
  const int ny = y + dy;
  if (!walkable(nx, ny)) return -1;
  const int id = map_->toId(nx, ny);
  if (id == goal_id_ || hasForcedNeighbor(nx, ny, dx, dy)) return id;
  if (dx != 0 && dy != 0) {
    if (jump(nx, ny, dx, 0) >= 0 || jump(nx, ny, 0, dy) >= 0) return id;
  }
  return jump(nx, ny, dx, dy);
}

std::vector<std::pair<int, int>> JumpPointSearch::prunedDirections(int id) const {
  int x = 0, y = 0;
  map_->toXY(id, &x, &y);
  if (parent_[id] < 0) {
    return {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  }
  int px = 0, py = 0;
  map_->toXY(parent_[id], &px, &py);
  const int dx = sign(x - px);
  const int dy = sign(y - py);
  std::vector<std::pair<int, int>> dirs;
  if (dx != 0 && dy != 0) {
    dirs = {{dx, dy}, {dx, 0}, {0, dy}};
    if (!walkable(x - dx, y) && walkable(x - dx, y + dy)) dirs.emplace_back(-dx, dy);
    if (!walkable(x, y - dy) && walkable(x + dx, y - dy)) dirs.emplace_back(dx, -dy);
  } else if (dx != 0) {
    dirs.emplace_back(dx, 0);
    if (!walkable(x, y + 1) && walkable(x + dx, y + 1)) dirs.emplace_back(dx, 1);
    if (!walkable(x, y - 1) && walkable(x + dx, y - 1)) dirs.emplace_back(dx, -1);
  } else {
    dirs.emplace_back(0, dy);
    if (!walkable(x + 1, y) && walkable(x + 1, y + dy)) dirs.emplace_back(1, dy);
    if (!walkable(x - 1, y) && walkable(x - 1, y + dy)) dirs.emplace_back(-1, dy);
  }
  return dirs;
}

double JumpPointSearch::segmentCost(int a, int b) const {
  int ax = 0, ay = 0, bx = 0, by = 0;
  map_->toXY(a, &ax, &ay);
  map_->toXY(b, &bx, &by);
  const int dx = std::abs(ax - bx);
  const int dy = std::abs(ay - by);
  return std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy);
}

SearchResult JumpPointSearch::search(int sx, int sy, int gx, int gy) {
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = AdvancedSearchStats{};
  if (map_ == nullptr || map_->numCells() == 0 || !walkable(sx, sy) || !walkable(gx, gy)) {
    stats_.result = SearchResult::kFailedBadInput;
    return stats_.result;
  }
  start_id_ = map_->toId(sx, sy);
  goal_id_ = map_->toId(gx, gy);
  g_.assign(map_->numCells(), kInf);
  parent_.assign(map_->numCells(), -1);
  closed_.assign(map_->numCells(), 0);
  std::priority_queue<OpenItem, std::vector<OpenItem>, OpenGreater> open;
  g_[start_id_] = 0.0;
  open.push({start_id_, heuristic(start_id_), 0.0});
  ++stats_.pushes;

  while (!open.empty()) {
    const OpenItem item = open.top();
    open.pop();
    ++stats_.queue_pops;
    if (closed_[item.id] || item.g > g_[item.id] + kEps) continue;
    closed_[item.id] = 1;
    ++stats_.expanded;
    if (item.id == goal_id_) {
      stats_.result = SearchResult::kSuccess;
      break;
    }
    int x = 0, y = 0;
    map_->toXY(item.id, &x, &y);
    for (const auto& dir : prunedDirections(item.id)) {
      const int jp = jump(x, y, dir.first, dir.second);
      if (jp < 0 || closed_[jp]) continue;
      const double next_g = g_[item.id] + segmentCost(item.id, jp);
      if (next_g + kEps >= g_[jp]) continue;
      g_[jp] = next_g;
      parent_[jp] = item.id;
      open.push({jp, next_g + heuristic(jp), next_g});
      ++stats_.pushes;
    }
  }
  if (stats_.result != SearchResult::kSuccess) stats_.result = SearchResult::kFailedNoPath;
  const auto t1 = std::chrono::steady_clock::now();
  stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  if (stats_.result == SearchResult::kSuccess) {
    stats_.path_cost = g_[goal_id_];
    stats_.path_steps = static_cast<int>(pathIds().size()) - 1;
  }
  return stats_.result;
}

std::vector<int> JumpPointSearch::jumpPointIds() const {
  std::vector<int> path;
  if (stats_.result != SearchResult::kSuccess) return path;
  for (int cur = goal_id_, guard = 0; cur >= 0 && guard <= static_cast<int>(parent_.size());
       cur = parent_[cur], ++guard) {
    path.push_back(cur);
    if (cur == start_id_) break;
  }
  if (path.empty() || path.back() != start_id_) return {};
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<int> JumpPointSearch::pathIds() const {
  const std::vector<int> jumps = jumpPointIds();
  if (jumps.empty()) return {};
  std::vector<int> path{jumps.front()};
  for (std::size_t i = 1; i < jumps.size(); ++i) {
    int x = 0, y = 0, tx = 0, ty = 0;
    map_->toXY(path.back(), &x, &y);
    map_->toXY(jumps[i], &tx, &ty);
    const int dx = sign(tx - x);
    const int dy = sign(ty - y);
    while (x != tx || y != ty) {
      x += dx;
      y += dy;
      path.push_back(map_->toId(x, y));
    }
  }
  return path;
}

std::vector<std::pair<int, int>> JumpPointSearch::pathGrid() const {
  std::vector<std::pair<int, int>> out;
  for (const int id : pathIds()) {
    int x = 0, y = 0;
    map_->toXY(id, &x, &y);
    out.emplace_back(x, y);
  }
  return out;
}

}  // namespace search_tutorial
