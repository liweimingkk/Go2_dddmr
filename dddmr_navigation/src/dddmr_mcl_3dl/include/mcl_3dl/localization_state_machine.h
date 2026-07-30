/*
 * Copyright (c) 2026, DDDMR Navigation contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 */

#ifndef MCL_3DL_LOCALIZATION_STATE_MACHINE_H
#define MCL_3DL_LOCALIZATION_STATE_MACHINE_H

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace mcl_3dl
{

enum class LocalizationState
{
  UNINITIALIZED,
  LOCALIZING,
  TRACKING,
  LOST,
};

enum class LocalizationAttemptKind
{
  NONE,
  CONFIGURED_INITIAL,
  FIXED_POSE,
  LOCAL_RECOVERY,
  GLOBAL_RECOVERY,
};

inline bool usesRecoveryTimeout(const LocalizationAttemptKind kind)
{
  return kind == LocalizationAttemptKind::LOCAL_RECOVERY;
}

inline bool switchesToCurrentMapAfterWarmup(
    const LocalizationAttemptKind kind)
{
  return kind == LocalizationAttemptKind::FIXED_POSE ||
    kind == LocalizationAttemptKind::LOCAL_RECOVERY;
}

inline const char* localizationStateName(const LocalizationState state)
{
  switch (state)
  {
    case LocalizationState::UNINITIALIZED:
      return "UNINITIALIZED";
    case LocalizationState::LOCALIZING:
      return "LOCALIZING";
    case LocalizationState::TRACKING:
      return "TRACKING";
    case LocalizationState::LOST:
      return "LOST";
  }
  return "UNINITIALIZED";
}

struct LocalizationStateConfig
{
  double tracking_match_ratio{0.15};
  double lost_match_ratio{0.08};
  double tracking_max_xy_std{0.75};
  double tracking_max_z_std{1.0e6};
  double tracking_max_roll_std{1.0e6};
  double tracking_max_pitch_std{1.0e6};
  double tracking_max_yaw_std{0.35};
  double tracking_max_residual{1.0e6};
  double tracking_max_map_odom_tilt{3.2};
  double tracking_max_ground_normal_error{3.2};
  double tracking_max_base_height_error{1.0e6};
  double tracking_max_pose_height_error{1.0e6};
  double lost_max_xy_std{1.50};
  double lost_max_z_std{1.0e6};
  double lost_max_roll_std{1.0e6};
  double lost_max_pitch_std{1.0e6};
  double lost_max_yaw_std{0.80};
  double lost_max_residual{1.0e6};
  double lost_max_map_odom_tilt{3.2};
  double lost_max_ground_normal_error{3.2};
  double lost_max_base_height_error{1.0e6};
  double lost_max_pose_height_error{1.0e6};
  bool require_ground_health{false};
  std::size_t tracking_good_frames{4};
  std::size_t lost_bad_frames{3};
};

struct LocalizationObservation
{
  double match_ratio{0.0};
  double xy_std{0.0};
  double z_std{0.0};
  double roll_std{0.0};
  double pitch_std{0.0};
  double yaw_std{0.0};
  double residual{0.0};
  double map_odom_tilt{0.0};
  double ground_normal_error{0.0};
  double base_height_error{0.0};
  double pose_height_error{0.0};
  bool ground_valid{true};
  bool particle_count_converged{false};
};

class LocalizationConvergenceTimer
{
public:
  struct Snapshot
  {
    uint64_t generation;
    int64_t started_ns;
  };

  uint64_t reset()
  {
    started_ns_.store(0);
    return generation_.fetch_add(1) + 1;
  }

  bool armAfterCompletedMeasurement(const int64_t completed_ns)
  {
    if (completed_ns <= 0)
    {
      return false;
    }
    int64_t expected = 0;
    return started_ns_.compare_exchange_strong(expected, completed_ns);
  }

  int64_t startedNs() const
  {
    return started_ns_.load();
  }

  uint64_t generation() const
  {
    return generation_.load();
  }

  bool isGenerationCurrent(const uint64_t expected) const
  {
    return generation() == expected;
  }

  Snapshot snapshot() const
  {
    while (true)
    {
      const uint64_t before = generation_.load();
      const int64_t started_ns = started_ns_.load();
      const uint64_t after = generation_.load();
      if (before == after)
      {
        return Snapshot{after, started_ns};
      }
    }
  }

  bool isCurrent(const Snapshot& expected) const
  {
    const Snapshot current = snapshot();
    return current.generation == expected.generation &&
      current.started_ns == expected.started_ns;
  }

  static bool expired(
      const Snapshot& snapshot,
      const int64_t now_ns,
      const double timeout_sec)
  {
    return snapshot.started_ns > 0 &&
      static_cast<double>(now_ns - snapshot.started_ns) / 1e9 > timeout_sec;
  }

private:
  std::atomic<uint64_t> generation_{0};
  std::atomic<int64_t> started_ns_{0};
};

class LocalizationStateMachine
{
public:
  explicit LocalizationStateMachine(LocalizationStateConfig config = {})
  : config_(config)
  {
    config_.tracking_good_frames = std::max<std::size_t>(1, config_.tracking_good_frames);
    config_.lost_bad_frames = std::max<std::size_t>(1, config_.lost_bad_frames);
  }

  LocalizationState state() const
  {
    return state_;
  }

  const char* stateName() const
  {
    return localizationStateName(state_);
  }

  std::size_t goodFrames() const
  {
    return good_frames_;
  }

  std::size_t badFrames() const
  {
    return bad_frames_;
  }

  bool reset()
  {
    return transitionTo(LocalizationState::UNINITIALIZED);
  }

  bool startLocalizing()
  {
    return transitionTo(LocalizationState::LOCALIZING);
  }

  bool markLost()
  {
    return transitionTo(LocalizationState::LOST);
  }

  bool observe(const LocalizationObservation& observation)
  {
    const bool observation_finite =
        std::isfinite(observation.match_ratio) &&
        std::isfinite(observation.xy_std) &&
        std::isfinite(observation.z_std) &&
        std::isfinite(observation.roll_std) &&
        std::isfinite(observation.pitch_std) &&
        std::isfinite(observation.yaw_std) &&
        std::isfinite(observation.residual) &&
        std::isfinite(observation.map_odom_tilt) &&
        std::isfinite(observation.ground_normal_error) &&
        std::isfinite(observation.base_height_error) &&
        std::isfinite(observation.pose_height_error);

    if (state_ == LocalizationState::LOCALIZING)
    {
      const bool good =
        observation_finite &&
        observation.particle_count_converged &&
        observation.match_ratio >= config_.tracking_match_ratio &&
        observation.xy_std <= config_.tracking_max_xy_std &&
        observation.z_std <= config_.tracking_max_z_std &&
        observation.roll_std <= config_.tracking_max_roll_std &&
        observation.pitch_std <= config_.tracking_max_pitch_std &&
        observation.yaw_std <= config_.tracking_max_yaw_std &&
        observation.residual <= config_.tracking_max_residual &&
        observation.map_odom_tilt <= config_.tracking_max_map_odom_tilt &&
        observation.ground_normal_error <= config_.tracking_max_ground_normal_error &&
        observation.base_height_error <= config_.tracking_max_base_height_error &&
        observation.pose_height_error <= config_.tracking_max_pose_height_error &&
        (!config_.require_ground_health || observation.ground_valid);

      good_frames_ = good ? good_frames_ + 1 : 0;
      bad_frames_ = 0;
      if (good_frames_ >= config_.tracking_good_frames)
      {
        return transitionTo(LocalizationState::TRACKING);
      }
      return false;
    }

    if (state_ == LocalizationState::TRACKING)
    {
      const bool bad =
        !observation_finite ||
        observation.match_ratio < config_.lost_match_ratio ||
        observation.xy_std > config_.lost_max_xy_std ||
        observation.z_std > config_.lost_max_z_std ||
        observation.roll_std > config_.lost_max_roll_std ||
        observation.pitch_std > config_.lost_max_pitch_std ||
        observation.yaw_std > config_.lost_max_yaw_std ||
        observation.residual > config_.lost_max_residual ||
        observation.map_odom_tilt > config_.lost_max_map_odom_tilt ||
        observation.ground_normal_error > config_.lost_max_ground_normal_error ||
        observation.base_height_error > config_.lost_max_base_height_error ||
        observation.pose_height_error > config_.lost_max_pose_height_error ||
        (config_.require_ground_health && !observation.ground_valid);

      bad_frames_ = bad ? bad_frames_ + 1 : 0;
      good_frames_ = 0;
      if (bad_frames_ >= config_.lost_bad_frames)
      {
        return transitionTo(LocalizationState::LOST);
      }
    }
    return false;
  }

private:
  bool transitionTo(const LocalizationState next)
  {
    const bool changed = state_ != next;
    state_ = next;
    good_frames_ = 0;
    bad_frames_ = 0;
    return changed;
  }

  LocalizationStateConfig config_;
  LocalizationState state_{LocalizationState::UNINITIALIZED};
  std::size_t good_frames_{0};
  std::size_t bad_frames_{0};
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_LOCALIZATION_STATE_MACHINE_H
