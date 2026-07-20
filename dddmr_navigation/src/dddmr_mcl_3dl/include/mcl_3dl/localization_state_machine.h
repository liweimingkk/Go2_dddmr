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
#include <cstddef>
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
  double tracking_max_yaw_std{0.35};
  double lost_max_xy_std{1.50};
  double lost_max_yaw_std{0.80};
  std::size_t tracking_good_frames{4};
  std::size_t lost_bad_frames{3};
  double tracking_min_dominant_mode_mass{0.0};
};

inline double effectiveTrackingMinDominantModeMass(
    const bool adaptive_enabled, const double configured_mass)
{
  return adaptive_enabled ? std::clamp(configured_mass, 0.0, 1.0) : 0.0;
}

struct LocalizationObservation
{
  double match_ratio{0.0};
  double xy_std{0.0};
  double yaw_std{0.0};
  bool posterior_stable{false};
  double dominant_mode_mass{0.0};
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
    if (state_ == LocalizationState::LOCALIZING)
    {
      const bool good =
        observation.posterior_stable &&
        observation.match_ratio >= config_.tracking_match_ratio &&
        observation.xy_std <= config_.tracking_max_xy_std &&
        observation.yaw_std <= config_.tracking_max_yaw_std &&
        observation.dominant_mode_mass >=
          config_.tracking_min_dominant_mode_mass;

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
        observation.match_ratio < config_.lost_match_ratio ||
        observation.xy_std > config_.lost_max_xy_std ||
        observation.yaw_std > config_.lost_max_yaw_std;

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
