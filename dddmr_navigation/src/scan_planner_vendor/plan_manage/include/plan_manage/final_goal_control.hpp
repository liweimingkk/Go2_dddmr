#ifndef PLAN_MANAGE__FINAL_GOAL_CONTROL_HPP_
#define PLAN_MANAGE__FINAL_GOAL_CONTROL_HPP_

#include <cmath>
#include <limits>

namespace scan_planner
{

enum class FinalGoalPhase
{
  TRACKING,
  REACQUIRE_POSITION,
  ALIGN_YAW,
  COMPLETE,
  INVALID,
};

struct FinalGoalStatus
{
  FinalGoalPhase phase{FinalGoalPhase::INVALID};
  double yaw_error{0.0};
};

inline double normalizeAngle(const double angle)
{
  if (!std::isfinite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  return std::remainder(angle, kTwoPi);
}

inline FinalGoalStatus evaluateFinalGoal(
  const bool trajectory_finished,
  const double position_error,
  const bool has_final_yaw,
  const double final_yaw,
  const double current_yaw,
  const double position_tolerance,
  const double yaw_tolerance)
{
  constexpr double kPi = 3.14159265358979323846;
  if (
    !std::isfinite(position_error) || position_error < 0.0 ||
    !std::isfinite(position_tolerance) || position_tolerance <= 0.0 ||
    !std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0 ||
    yaw_tolerance > kPi)
  {
    return {FinalGoalPhase::INVALID, 0.0};
  }

  if (!trajectory_finished) {
    return {FinalGoalPhase::TRACKING, 0.0};
  }
  if (position_error > position_tolerance) {
    return {FinalGoalPhase::REACQUIRE_POSITION, 0.0};
  }
  if (!has_final_yaw) {
    return {FinalGoalPhase::COMPLETE, 0.0};
  }
  if (!std::isfinite(final_yaw) || !std::isfinite(current_yaw)) {
    return {FinalGoalPhase::INVALID, 0.0};
  }

  const double yaw_error = normalizeAngle(final_yaw - current_yaw);
  if (!std::isfinite(yaw_error)) {
    return {FinalGoalPhase::INVALID, 0.0};
  }
  if (std::abs(yaw_error) > yaw_tolerance) {
    return {FinalGoalPhase::ALIGN_YAW, yaw_error};
  }
  return {FinalGoalPhase::COMPLETE, yaw_error};
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE__FINAL_GOAL_CONTROL_HPP_
