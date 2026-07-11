#ifndef LOCAL_PLANNER__IN_PLACE_ROTATION_HYSTERESIS_H_
#define LOCAL_PLANNER__IN_PLACE_ROTATION_HYSTERESIS_H_

#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <vector>

namespace local_planner
{

struct InPlaceRotationCandidate
{
  double linear_x{0.0};
  double angular_z{0.0};
  double cost{-1.0};
};

// Keeps a collision-checked in-place turn on one yaw sign across control
// cycles. The caller still owns trajectory generation and collision scoring;
// this policy can only choose among candidates whose cost is non-negative.
class InPlaceRotationHysteresis
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void configure(
    const double minimum_hold_seconds,
    const double switch_cost_margin,
    const double reset_gap_seconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    minimum_hold_seconds_ = minimum_hold_seconds;
    switch_cost_margin_ = switch_cost_margin;
    reset_gap_seconds_ = reset_gap_seconds;
    resetUnlocked();
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resetUnlocked();
  }

  // preferred_index is the stateless winner selected by the existing critics.
  // candidates.size() is used as the no-selection sentinel.
  std::size_t select(
    const std::vector<InPlaceRotationCandidate> & candidates,
    const std::size_t preferred_index,
    const TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if(preferred_index >= candidates.size() || !isAccepted(candidates[preferred_index])){
      resetIfGapExpired(now);
      return candidates.size();
    }

    const auto & preferred = candidates[preferred_index];
    if(!isInPlace(preferred)){
      resetUnlocked();
      return preferred_index;
    }

    const int preferred_sign = sign(preferred.angular_z);
    if(
      locked_sign_ == 0 ||
      now < last_in_place_time_ ||
      elapsedSeconds(last_in_place_time_, now) > reset_gap_seconds_)
    {
      lockDirection(preferred_sign, now);
      return preferred_index;
    }

    if(preferred_sign == locked_sign_){
      last_in_place_time_ = now;
      return preferred_index;
    }

    const std::size_t locked_index = bestAcceptedInPlaceForSign(candidates, locked_sign_);
    if(locked_index >= candidates.size()){
      // The previous direction is no longer collision-accepted. Never retain a
      // rejected trajectory merely to preserve direction continuity.
      lockDirection(preferred_sign, now);
      return preferred_index;
    }

    const bool hold_expired =
      now >= lock_started_time_ &&
      elapsedSeconds(lock_started_time_, now) >= minimum_hold_seconds_;
    const bool opposite_is_materially_better =
      preferred.cost + switch_cost_margin_ < candidates[locked_index].cost;

    if(hold_expired && opposite_is_materially_better){
      lockDirection(preferred_sign, now);
      return preferred_index;
    }

    last_in_place_time_ = now;
    return locked_index;
  }

private:
  static constexpr double kLinearEpsilon = 1e-4;
  static constexpr double kAngularEpsilon = 1e-4;

  static bool isAccepted(const InPlaceRotationCandidate & candidate)
  {
    return std::isfinite(candidate.cost) && candidate.cost >= 0.0;
  }

  static bool isInPlace(const InPlaceRotationCandidate & candidate)
  {
    return
      std::fabs(candidate.linear_x) <= kLinearEpsilon &&
      std::fabs(candidate.angular_z) > kAngularEpsilon;
  }

  static int sign(const double value)
  {
    return value > 0.0 ? 1 : -1;
  }

  static double elapsedSeconds(const TimePoint from, const TimePoint to)
  {
    return std::chrono::duration<double>(to - from).count();
  }

  static std::size_t bestAcceptedInPlaceForSign(
    const std::vector<InPlaceRotationCandidate> & candidates,
    const int requested_sign)
  {
    std::size_t best_index = candidates.size();
    double best_cost = 0.0;
    for(std::size_t index = 0; index < candidates.size(); ++index){
      const auto & candidate = candidates[index];
      if(
        !isAccepted(candidate) || !isInPlace(candidate) ||
        sign(candidate.angular_z) != requested_sign)
      {
        continue;
      }
      if(best_index >= candidates.size() || candidate.cost <= best_cost){
        best_index = index;
        best_cost = candidate.cost;
      }
    }
    return best_index;
  }

  void lockDirection(const int direction_sign, const TimePoint now)
  {
    locked_sign_ = direction_sign;
    lock_started_time_ = now;
    last_in_place_time_ = now;
  }

  void resetIfGapExpired(const TimePoint now)
  {
    if(
      locked_sign_ != 0 &&
      (now < last_in_place_time_ ||
       elapsedSeconds(last_in_place_time_, now) > reset_gap_seconds_))
    {
      resetUnlocked();
    }
  }

  void resetUnlocked()
  {
    locked_sign_ = 0;
    lock_started_time_ = TimePoint{};
    last_in_place_time_ = TimePoint{};
  }

  std::mutex mutex_;
  int locked_sign_{0};
  TimePoint lock_started_time_{};
  TimePoint last_in_place_time_{};
  double minimum_hold_seconds_{1.0};
  double switch_cost_margin_{0.05};
  double reset_gap_seconds_{2.0};
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__IN_PLACE_ROTATION_HYSTERESIS_H_
