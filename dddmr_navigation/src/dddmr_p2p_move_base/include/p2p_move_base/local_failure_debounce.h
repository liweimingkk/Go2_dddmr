#ifndef P2P_MOVE_BASE__LOCAL_FAILURE_DEBOUNCE_H_
#define P2P_MOVE_BASE__LOCAL_FAILURE_DEBOUNCE_H_

#include <cstddef>
#include <stdexcept>

namespace p2p_move_base
{

// Filters isolated local-planner failures without ever reusing an old motion
// command.  The caller must publish zero velocity for every failed cycle; this
// helper only decides when a persistent failure should escalate into a
// waiting/replanning state.
class LocalFailureDebounce
{
public:
  void configure(const std::size_t confirmation_cycles)
  {
    if(confirmation_cycles == 0U){
      throw std::invalid_argument(
        "local failure confirmation cycles must be positive");
    }
    confirmation_cycles_ = confirmation_cycles;
    reset();
  }

  bool recordFailure()
  {
    if(failure_cycles_ < confirmation_cycles_){
      ++failure_cycles_;
    }
    return failure_cycles_ >= confirmation_cycles_;
  }

  void recordSafeCycle()
  {
    reset();
  }

  void reset()
  {
    failure_cycles_ = 0U;
  }

  std::size_t failureCycles() const
  {
    return failure_cycles_;
  }

private:
  std::size_t confirmation_cycles_{1U};
  std::size_t failure_cycles_{0U};
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__LOCAL_FAILURE_DEBOUNCE_H_
