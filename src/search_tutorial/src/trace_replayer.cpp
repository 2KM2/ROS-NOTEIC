#include "search_tutorial/trace_replayer.h"

#include <algorithm>

namespace search_tutorial {

void TraceReplayer::bind(const GraphSearch& search, int num_cells) {
  trace_ = &search.trace();
  start_id_ = search.startId();
  goal_id_ = search.goalId();
  result_ = search.stats().result;

  state_.assign(num_cells, NodeState::kUnvisited);
  g_.assign(num_cells, kInf);
  depth_.assign(num_cells, -1);
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
    std::fill(depth_.begin(), depth_.end(), -1);
    std::fill(parent_.begin(), parent_.end(), -1);
    // 初始状态 = 伪代码第 1 行执行完：只有起点在 frontier 里
    if (start_id_ >= 0 && start_id_ < static_cast<int>(state_.size())) {
      state_[start_id_] = NodeState::kOpen;
      g_[start_id_] = 0.0;
      depth_[start_id_] = 0;
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
  // 弹出来的是个重复条目：算法什么都没做，回放也什么都不做。
  // 特别注意别把 step.cur_g / cur_parent 写进节点 —— 那是条目自带的过期值，
  // 节点早就以更好的值落定过了，覆盖上去就会看到"g 莫名变大"的假象。
  if (step.is_stale_pop) return;

  // 一步 = 「出队的节点落定并转 CLOSED」+「按事件更新每个邻居」，
  // 顺序和 graph_search.cpp 的主循环严格一致。
  if (step.cur_id >= 0 && step.cur_id < static_cast<int>(state_.size())) {
    state_[step.cur_id] = NodeState::kClosed;
    g_[step.cur_id] = step.cur_g;
    depth_[step.cur_id] = step.cur_depth;
    parent_[step.cur_id] = step.cur_parent;
  }
  for (const NeighborEvent& ev : step.neighbors) {
    // 只有这三种事件真的改了节点状态：
    //   kPushedNew    第一次入 frontier（三个算法都有）
    //   kPushedAgain  DFS 重复压栈：状态还是 OPEN，但 g/depth/父亲跟着栈顶那份走
    //   kUpdatedBetter Dijkstra 松弛成功：g 变小、换爹
    // 其余（越界/障碍/已CLOSED/已在队列/更长）都是"看一眼就放下"，不改任何状态。
    if (ev.action != NeighborAction::kPushedNew &&
        ev.action != NeighborAction::kPushedAgain &&
        ev.action != NeighborAction::kUpdatedBetter) {
      continue;
    }
    if (ev.id < 0 || ev.id >= static_cast<int>(state_.size())) continue;
    state_[ev.id] = NodeState::kOpen;
    g_[ev.id] = ev.g_new;
    depth_[ev.id] = ev.depth_new;
    parent_[ev.id] = step.cur_id;  // 父亲就是本步出队的那个节点
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

}  // namespace search_tutorial
