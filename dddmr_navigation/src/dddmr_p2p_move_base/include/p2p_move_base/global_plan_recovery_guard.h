#ifndef P2P_MOVE_BASE__GLOBAL_PLAN_RECOVERY_GUARD_H_
#define P2P_MOVE_BASE__GLOBAL_PLAN_RECOVERY_GUARD_H_

namespace p2p_move_base
{

// Rotating the robot cannot repair an invalid or off-map goal. Recovery motion
// is allowed only after this goal has produced at least one valid global plan;
// it can then still help when a previously navigable path becomes obstructed.
class GlobalPlanRecoveryGuard
{
public:
  void reset()
  {
    has_valid_plan_ = false;
  }

  void recordValidPlan()
  {
    has_valid_plan_ = true;
  }

  bool shouldAttemptRecovery() const
  {
    return has_valid_plan_;
  }

private:
  bool has_valid_plan_{false};
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__GLOBAL_PLAN_RECOVERY_GUARD_H_
