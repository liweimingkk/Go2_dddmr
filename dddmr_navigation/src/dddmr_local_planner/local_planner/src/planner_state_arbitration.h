#ifndef LOCAL_PLANNER__PLANNER_STATE_ARBITRATION_H_
#define LOCAL_PLANNER__PLANNER_STATE_ARBITRATION_H_

#include <dddmr_sys_core/dddmr_enum_states.h>

namespace local_planner
{

inline dddmr_sys_core::PlannerState arbitratePlannerState(
  const double best_trajectory_cost,
  const bool path_blocked_wait,
  const bool path_blocked_replanning)
{
  // A trajectory with a non-negative cost has already passed every configured
  // critic, including collision checking.  A blocked reference path must not
  // discard that safe local detour.
  if(best_trajectory_cost >= 0.0){
    return dddmr_sys_core::TRAJECTORY_FOUND;
  }

  if(path_blocked_wait){
    return dddmr_sys_core::PATH_BLOCKED_WAIT;
  }

  if(path_blocked_replanning){
    return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
  }

  return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
}

}  // namespace local_planner

#endif  // LOCAL_PLANNER__PLANNER_STATE_ARBITRATION_H_
