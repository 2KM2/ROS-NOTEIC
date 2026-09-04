// trace 回放器 —— 把 A* 的搜索过程当录像带来放。
//
// 为什么需要它？
//   A* 跑完之后，nodes_ 里只剩**最终**状态。想在 rviz 里看"第 137 步时 OPEN 里
//   有哪些格子、当前节点的 g 是多少"，就得能把中间状态复原出来。
//
// 做法：不存快照（太占内存），而是**重放事件**。
//   AStar 的 trace 已经记下了每一步"弹出了谁、每个邻居被怎么处理"，
//   而这些信息足以把 state/g/f/parent 数组一步步推出来 —— 和原算法逐位一致。
//
//   cursor = 已经应用的步数：
//     cursor = 0        -> 只有起点在 OPEN（搜索刚开始）
//     cursor = k        -> 相当于原算法执行完第 k 次出堆的瞬间
//     cursor = numSteps -> 搜索结束（成功时终点已 closed，父链完整）
//
// 往后退一步 = seekTo(cursor-1)，内部从 0 重放。O(步数)，交互场景完全够用。
#ifndef ASTAR_TUTORIAL_TRACE_REPLAYER_H
#define ASTAR_TUTORIAL_TRACE_REPLAYER_H

#include <vector>

#include "astar_tutorial/astar.h"

namespace astar_tutorial {

class TraceReplayer {
 public:
  // 绑定一次搜索的结果。astar 必须比本对象活得久（只存指针，不拷贝 trace）。
  void bind(const AStar& astar, int num_cells);

  bool valid() const { return trace_ != nullptr && !state_.empty(); }
  int numSteps() const { return trace_ == nullptr ? 0 : static_cast<int>(trace_->size()); }
  int cursor() const { return cursor_; }
  bool atEnd() const { return cursor_ >= numSteps(); }

  // 前进一步；已经到末尾返回 false
  bool stepForward();
  // 跳到第 k 步（k 会被钳位到 [0, numSteps]）
  void seekTo(int k);
  void reset() { seekTo(0); }
  void finish() { seekTo(numSteps()); }

  // ---- 当前状态查询 ----
  const std::vector<NodeState>& states() const { return state_; }
  const std::vector<double>& gValues() const { return g_; }
  const std::vector<double>& fValues() const { return f_; }
  const std::vector<int>& parents() const { return parent_; }

  // cursor>0 时，返回刚刚被应用的那一步（即"当前正在扩展的节点"所在的步）
  const SearchStep* currentStep() const;
  // 本步出堆的节点 id；cursor==0 时返回起点
  int currentNodeId() const;

  std::vector<int> openIds() const;
  std::vector<int> closedIds() const;

  // 从起点到 id 的**当前**父链（搜索过程中的临时最优路径）。
  // 搜索还没结束时，这条链会随着 g 被改小而变化 —— rviz 里看它抖动很直观。
  std::vector<int> pathTo(int id) const;

  int startId() const { return start_id_; }
  int goalId() const { return goal_id_; }
  SearchResult result() const { return result_; }

 private:
  void applyStep(const SearchStep& step);

  const std::vector<SearchStep>* trace_ = nullptr;
  int start_id_ = -1;
  int goal_id_ = -1;
  double start_h_ = 0.0;
  SearchResult result_ = SearchResult::kFailedBadInput;

  int cursor_ = 0;
  std::vector<NodeState> state_;
  std::vector<double> g_;
  std::vector<double> f_;
  std::vector<int> parent_;
};

}  // namespace astar_tutorial

#endif  // ASTAR_TUTORIAL_TRACE_REPLAYER_H
