#include "p2p_move_base/stair_traversal_supervisor.h"

#include <cmath>
#include <stdexcept>

namespace p2p_move_base
{

void StairTraversalSupervisor::configure(const StairSupervisorConfig & config)
{
  if (config.confirmation_cycles == 0) {
    throw std::invalid_argument("stair confirmation_cycles must be positive");
  }
  if (!std::isfinite(config.min_confidence) || config.min_confidence < 0.0 ||
    config.min_confidence > 1.0)
  {
    throw std::invalid_argument("stair min_confidence must be in [0, 1]");
  }
  if (!std::isfinite(config.min_support_ratio) || config.min_support_ratio < 0.0 ||
    config.min_support_ratio > 1.0)
  {
    throw std::invalid_argument("stair min_support_ratio must be in [0, 1]");
  }
  if (!std::isfinite(config.max_heading_error_rad) || config.max_heading_error_rad < 0.0 ||
    !std::isfinite(config.max_lateral_error_m) || config.max_lateral_error_m < 0.0 ||
    !std::isfinite(config.align_max_forward_mps) || config.align_max_forward_mps < 0.0 ||
    !std::isfinite(config.align_max_yaw_rps) || config.align_max_yaw_rps < 0.0 ||
    !std::isfinite(config.committed_max_yaw_rps) || config.committed_max_yaw_rps < 0.0)
  {
    throw std::invalid_argument("stair supervisor limits must be finite and non-negative");
  }

  config_ = config;
  reset();
}

bool StairTraversalSupervisor::finiteObservation(const StairObservation & observation) const
{
  return std::isfinite(observation.confidence) &&
         std::isfinite(observation.support_ratio) &&
         std::isfinite(observation.heading_error_rad) &&
         std::isfinite(observation.lateral_error_m);
}

bool StairTraversalSupervisor::geometryWithinLimits(
  const StairObservation & observation) const
{
  return observation.confidence >= config_.min_confidence &&
         observation.support_ratio >= config_.min_support_ratio &&
         observation.snapshot_version > 0U &&
         observation.static_ground_generation > 0U &&
         observation.step_count > 0 &&
         std::fabs(observation.heading_error_rad) <= config_.max_heading_error_rad &&
         std::fabs(observation.lateral_error_m) <= config_.max_lateral_error_m;
}

StairTraversalState StairTraversalSupervisor::latchFault(const std::string & reason)
{
  state_ = StairTraversalState::FAULT_LATCH;
  fault_reason_ = reason;
  confirmation_count_ = 0;
  return state_;
}

StairTraversalState StairTraversalSupervisor::latchExternalFault(
  const std::string & reason)
{
  if (!config_.enabled) {
    return state_;
  }
  return latchFault(reason.empty() ? "external_fault" : reason);
}

StairTraversalState StairTraversalSupervisor::update(
  const StairObservation & observation)
{
  if (!config_.enabled) {
    reset();
    return state_;
  }
  if (faultLatched()) {
    return state_;
  }
  if (!finiteObservation(observation)) {
    return latchFault("nonfinite_observation");
  }
  refreshInputHealth(
    observation.fresh, observation.gait_fresh, observation.gait_unchanged);
  if (faultLatched() || !input_ready_) {
    return state_;
  }
  if (observation.drop_detected) {
    return latchFault("drop_detected");
  }
  if (observation.dynamic_obstacle) {
    return latchFault("dynamic_obstacle");
  }

  if (state_ == StairTraversalState::NORMAL && !observation.stair_candidate) {
    return state_;
  }
  if (!observation.stair_candidate || observation.staircase_id < 0) {
    return latchFault("stair_model_lost");
  }

  switch (state_) {
    case StairTraversalState::NORMAL:
      if (!observation.entry_valid || !geometryWithinLimits(observation)) {
        confirmation_count_ = 0;
        committed_staircase_id_ = -1;
        committed_static_ground_generation_ = 0;
        return state_;
      }
      committed_staircase_id_ = observation.staircase_id;
      committed_static_ground_generation_ = observation.static_ground_generation;
      committed_seen_stair_ = false;
      confirmation_count_ = 1;
      state_ = StairTraversalState::PRECHECK;
      break;

    case StairTraversalState::PRECHECK:
      if (observation.staircase_id != committed_staircase_id_ ||
        observation.static_ground_generation != committed_static_ground_generation_)
      {
        return latchFault("stair_model_changed");
      }
      if (!observation.entry_valid || !geometryWithinLimits(observation)) {
        return latchFault("precheck_failed");
      }
      ++confirmation_count_;
      if (confirmation_count_ >= config_.confirmation_cycles) {
        state_ = StairTraversalState::APPROACH;
      }
      break;

    case StairTraversalState::APPROACH:
      if (observation.staircase_id != committed_staircase_id_ ||
        observation.static_ground_generation != committed_static_ground_generation_)
      {
        return latchFault("stair_model_changed");
      }
      if (!observation.entry_valid) {
        return latchFault("entry_lost");
      }
      if (observation.at_entry) {
        state_ = StairTraversalState::ALIGN;
      }
      break;

    case StairTraversalState::ALIGN:
      if (observation.staircase_id != committed_staircase_id_ ||
        observation.static_ground_generation != committed_static_ground_generation_)
      {
        return latchFault("stair_model_changed");
      }
      if (!observation.at_entry) {
        return latchFault("left_entry_during_alignment");
      }
      if (geometryWithinLimits(observation) && observation.allow_forward) {
        state_ = StairTraversalState::COMMITTED;
      }
      break;

    case StairTraversalState::COMMITTED:
      if (observation.staircase_id != committed_staircase_id_ ||
        observation.static_ground_generation != committed_static_ground_generation_)
      {
        return latchFault("stair_model_changed");
      }
      if (!observation.allow_forward || !geometryWithinLimits(observation)) {
        return latchFault("committed_limits_exceeded");
      }
      if (observation.on_stair) {
        committed_seen_stair_ = true;
      }
      if (!observation.on_stair && !observation.at_entry && !observation.landing_valid) {
        return latchFault("stair_progress_lost");
      }
      if (committed_seen_stair_ && observation.landing_valid) {
        state_ = StairTraversalState::LANDING_VERIFY;
      }
      break;

    case StairTraversalState::LANDING_VERIFY:
      if (!observation.landing_valid) {
        return latchFault("landing_lost");
      }
      if (observation.full_body_on_landing) {
        reset();
        input_ready_ = true;
        input_hold_reason_.clear();
      }
      break;

    case StairTraversalState::FAULT_LATCH:
      break;
  }

  return state_;
}

StairTraversalState StairTraversalSupervisor::refreshInputHealth(
  bool terrain_fresh, bool gait_fresh, bool gait_unchanged)
{
  if (!config_.enabled) {
    reset();
    return state_;
  }
  if (faultLatched()) {
    return state_;
  }

  const auto hold_or_latch = [this](const std::string & reason) {
      input_ready_ = false;
      input_hold_reason_ = reason;
      if (state_ != StairTraversalState::NORMAL) {
        return latchFault(reason);
      }
      return state_;
    };

  if (!terrain_fresh) {
    return hold_or_latch("stale_terrain");
  }
  if (!gait_fresh) {
    return hold_or_latch("stale_gait");
  }
  if (!gait_unchanged) {
    input_ready_ = false;
    input_hold_reason_ = "gait_changed";
    return latchFault("gait_changed");
  }

  input_ready_ = true;
  input_hold_reason_.clear();
  return state_;
}

StairCommandDecision StairTraversalSupervisor::filterCommand(
  double x, double y, double yaw) const
{
  StairCommandDecision decision;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    decision.reason = "nonfinite_command";
    return decision;
  }
  if (!config_.enabled) {
    decision.allowed = true;
    decision.x = x;
    decision.y = y;
    decision.yaw = yaw;
    decision.reason = "supervisor_disabled";
    return decision;
  }
  if (!input_ready_) {
    decision.reason = input_hold_reason_.empty() ?
      "terrain_input_not_ready" : input_hold_reason_;
    return decision;
  }
  if (state_ == StairTraversalState::NORMAL || state_ == StairTraversalState::APPROACH) {
    decision.allowed = true;
    decision.x = x;
    decision.y = y;
    decision.yaw = yaw;
    decision.reason = state_ == StairTraversalState::APPROACH ?
      "stair_approach" : "normal";
    return decision;
  }
  if (state_ == StairTraversalState::ALIGN) {
    if (x < -config_.align_max_forward_mps || x > config_.align_max_forward_mps ||
      y != 0.0 || yaw < -config_.align_max_yaw_rps || yaw > config_.align_max_yaw_rps)
    {
      decision.latch_fault = true;
      decision.reason = "unscored_stair_align_command";
      return decision;
    }
    decision.allowed = true;
    decision.x = x;
    decision.y = y;
    decision.yaw = yaw;
    decision.reason = "stair_align";
    return decision;
  }
  if (state_ == StairTraversalState::COMMITTED) {
    if (x <= 0.0 || y != 0.0 ||
      yaw < -config_.committed_max_yaw_rps || yaw > config_.committed_max_yaw_rps)
    {
      decision.latch_fault = true;
      decision.reason = "unscored_stair_committed_command";
      return decision;
    }
    decision.allowed = true;
    decision.x = x;
    decision.y = y;
    decision.yaw = yaw;
    decision.reason = "stair_committed";
    return decision;
  }

  decision.reason = state_ == StairTraversalState::FAULT_LATCH ?
    fault_reason_ : "stair_hold";
  return decision;
}

void StairTraversalSupervisor::reset()
{
  state_ = StairTraversalState::NORMAL;
  confirmation_count_ = 0;
  committed_staircase_id_ = -1;
  committed_static_ground_generation_ = 0;
  committed_seen_stair_ = false;
  input_ready_ = false;
  input_hold_reason_ = "terrain_no_status";
  fault_reason_.clear();
}

bool StairTraversalSupervisor::resetFault(
  bool command_stopped, const StairObservation & observation)
{
  if (!faultLatched() || !command_stopped || !finiteObservation(observation) ||
    !observation.fresh || !observation.gait_fresh || !observation.gait_unchanged ||
    !observation.terrain_accepted || !observation.landing_valid ||
    !observation.full_body_on_landing || observation.drop_detected ||
    observation.dynamic_obstacle || observation.staircase_id < 0 ||
    !geometryWithinLimits(observation))
  {
    return false;
  }
  reset();
  input_ready_ = true;
  input_hold_reason_.clear();
  return true;
}

bool StairTraversalSupervisor::recoveryAllowed() const
{
  return !config_.enabled || state_ == StairTraversalState::NORMAL ||
         state_ == StairTraversalState::APPROACH;
}

bool StairTraversalSupervisor::replanningAllowed() const
{
  return !config_.enabled || state_ == StairTraversalState::NORMAL ||
         state_ == StairTraversalState::PRECHECK ||
         state_ == StairTraversalState::APPROACH;
}

}  // namespace p2p_move_base
