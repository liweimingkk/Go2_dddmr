#ifndef PLAN_MANAGE__REPLAN_DISTANCE_POLICY_HPP_
#define PLAN_MANAGE__REPLAN_DISTANCE_POLICY_HPP_

#include <cmath>

namespace scan_planner
{

inline bool isValidReplanDistanceConfiguration(
  const double min_replan_distance,
  const double finish_distance)
{
  return
    std::isfinite(min_replan_distance) && min_replan_distance > 0.0 &&
    std::isfinite(finish_distance) && finish_distance > 0.0 &&
    min_replan_distance < finish_distance;
}

inline bool isReplanDistanceBelowMinimum(
  const double distance,
  const double min_replan_distance)
{
  return
    !std::isfinite(distance) ||
    !std::isfinite(min_replan_distance) ||
    min_replan_distance <= 0.0 ||
    distance < min_replan_distance;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE__REPLAN_DISTANCE_POLICY_HPP_
