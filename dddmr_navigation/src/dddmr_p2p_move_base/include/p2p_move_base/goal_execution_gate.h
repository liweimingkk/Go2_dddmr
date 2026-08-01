#ifndef P2P_MOVE_BASE__GOAL_EXECUTION_GATE_H_
#define P2P_MOVE_BASE__GOAL_EXECUTION_GATE_H_

#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace p2p_move_base
{

// Serializes detached action execution threads while allowing a newly
// accepted goal to supersede any older goal still waiting for its turn.
class GoalExecutionGate
{
public:
  using Generation = std::uint64_t;

  Generation acceptLatest()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ++latest_generation_;
  }

  bool waitToStart(const Generation generation)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() {return !execution_active_;});
    if (generation != latest_generation_) {
      return false;
    }
    execution_active_ = true;
    active_generation_ = generation;
    return true;
  }

  void finish(const Generation generation)
  {
    bool finished = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (execution_active_ && active_generation_ == generation) {
        execution_active_ = false;
        finished = true;
      }
    }
    if (finished) {
      condition_.notify_all();
    }
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  Generation latest_generation_{0U};
  Generation active_generation_{0U};
  bool execution_active_{false};
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__GOAL_EXECUTION_GATE_H_
