#include "astar_tutorial/trace_replayer.h"

#include <algorithm>

namespace astar_tutorial {

void TraceReplayer::bind(const AStar& astar, int num_cells) {
  trace_ = &astar.trace();
  start_id_ = astar.startId();
  goal_id_ = astar.goalId();
  result_ = astar.stats().result;

  // 起点的 h。trace 第一步弹出的一定是起点，从那儿抄过来最省事。
  start_h_ = 0.0;
  if (!trace_->empty() && trace_->front().cur_id == start_id_) {
    start_h_ = trace_->front().cur_h;
  }

  state_.assign(num_cells, NodeState::kUnvisited);
  g_.assign(num_cells, kInf);
  f_.assign(num_cells, kInf);
  parent_.assign(num_cells, -1);
  cursor_ = -1;  // 强制 seekTo(0) 真正执行初始化
  seekTo(0);
}

void TraceReplayer::seekTo(int k) {
  if (!trace_) return;
  k = std::min(std::max(k, 0), numSteps());

  // 只能往前重放（事件不可逆），所以往后退就从头再来一遍。
  if (k < cursor_ || cursor_ < 0) {
    std::fill(state_.begin(), state_.end(), NodeState::kUnvisited);
    std::fill(g_.begin(), g_.end(), kInf);
    std::fill(f_.begin(), f_.end(), kInf);
    std::fill(parent_.begin(), parent_.end(), -1);
    // 初始状态 = 伪代码第 1 行执行完：只有起点在 OPEN
    if (start_id_ >= 0 && start_id_ < static_cast<int>(state_.size())) {
      state_[start_id_] = NodeState::kOpen;
      g_[start_id_] = 0.0;
      f_[start_id_] = start_h_;
      parent_[start_id_] = -1;
    }
    cursor_ = 0;
  }
  while (cursor_ < k) {
    applyStep((*trace_)[cursor_]);
    ++cursor_;
  }
}

bool TraceReplayer::stepForward() {
  if (atEnd()) return false;
  applyStep((*trace_)[cursor_]);
  ++cursor_;
  return true;
}

void TraceReplayer::applyStep(const SearchStep& step) {
  // 一步 = 「出堆的节点转 CLOSED」+「按事件更新每个邻居」，
  // 顺序和 astar.cpp 的主循环严格一致。
  if (step.cur_id >= 0 && step.cur_id < static_cast<int>(state_.size())) {
    state_[step.cur_id] = NodeState::kClosed;
    g_[step.cur_id] = step.cur_g;
    f_[step.cur_id] = step.cur_f;
    parent_[step.cur_id] = step.cur_parent;
  }
  for (const NeighborEvent& ev : step.neighbors) {
    // 只有这两种事件真的改了节点状态，其余（越界/障碍/已closed/更长）都是"看一眼就放下"。
    if (ev.action != NeighborAction::kPushedNew && ev.action != NeighborAction::kUpdatedBetter) {
      continue;
    }
    if (ev.id < 0 || ev.id >= static_cast<int>(state_.size())) continue;
    state_[ev.id] = NodeState::kOpen;
    g_[ev.id] = ev.g_new;
    f_[ev.id] = ev.f_new;
    parent_[ev.id] = step.cur_id;  // 父亲就是本步出堆的那个节点
  }
}

const SearchStep* TraceReplayer::currentStep() const {
  if (!trace_ || cursor_ <= 0) return nullptr;
  return &(*trace_)[cursor_ - 1];
}

int TraceReplayer::currentNodeId() const {
  const SearchStep* s = currentStep();
  return s ? s->cur_id : start_id_;
}

std::vector<int> TraceReplayer::openIds() const {
  std::vector<int> out;
  for (int id = 0; id < static_cast<int>(state_.size()); ++id) {
    if (state_[id] == NodeState::kOpen) out.push_back(id);
  }
  return out;
}

std::vector<int> TraceReplayer::closedIds() const {
  std::vector<int> out;
  for (int id = 0; id < static_cast<int>(state_.size()); ++id) {
    if (state_[id] == NodeState::kClosed) out.push_back(id);
  }
  return out;
}

std::vector<int> TraceReplayer::pathTo(int id) const {
  std::vector<int> path;
  if (id < 0 || id >= static_cast<int>(parent_.size())) return path;
  if (state_[id] == NodeState::kUnvisited) return path;
  int cur = id;
  for (std::size_t guard = 0; cur >= 0 && guard <= parent_.size(); ++guard) {
    path.push_back(cur);
    if (cur == start_id_) break;
    cur = parent_[cur];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace astar_tutorial
