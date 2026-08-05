#ifndef P2P_MOVE_BASE__GLOBAL_PLAN_RECOVERY_GUARD_H_
#define P2P_MOVE_BASE__GLOBAL_PLAN_RECOVERY_GUARD_H_

namespace p2p_move_base
{

enum class PlanningTimeoutAction
{
  ABORT_NO_VALID_PLAN,
  HOLD_AND_REPLAN,
  ATTEMPT_RECOVERY,
  ABORT_RETRIES_EXHAUSTED,
};

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

  PlanningTimeoutAction timeoutAction(
    bool hold_position_after_valid_plan,
    bool recovery_motion_enabled,
    int completed_hold_retries,
    int maximum_hold_retries) const
  {
    if (!has_valid_plan_) {
      return PlanningTimeoutAction::ABORT_NO_VALID_PLAN;
    }
    if (hold_position_after_valid_plan) {
      if (
        maximum_hold_retries > 0 &&
        completed_hold_retries < maximum_hold_retries)
      {
        return PlanningTimeoutAction::HOLD_AND_REPLAN;
      }
      return PlanningTimeoutAction::ABORT_RETRIES_EXHAUSTED;
    }
    if (recovery_motion_enabled) {
      return PlanningTimeoutAction::ATTEMPT_RECOVERY;
    }
    return PlanningTimeoutAction::ABORT_RETRIES_EXHAUSTED;
  }

private:
  bool has_valid_plan_{false};
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__GLOBAL_PLAN_RECOVERY_GUARD_H_
