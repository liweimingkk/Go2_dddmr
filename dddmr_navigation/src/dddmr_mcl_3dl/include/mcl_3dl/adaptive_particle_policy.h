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

#ifndef MCL_3DL_ADAPTIVE_PARTICLE_POLICY_H
#define MCL_3DL_ADAPTIVE_PARTICLE_POLICY_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace mcl_3dl
{

enum class ParticlePolicyAction
{
  HOLD,
  REDUCE,
  RESEED,
};

inline const char* particlePolicyActionName(const ParticlePolicyAction action)
{
  switch (action)
  {
    case ParticlePolicyAction::HOLD:
      return "hold";
    case ParticlePolicyAction::REDUCE:
      return "reduce";
    case ParticlePolicyAction::RESEED:
      return "reseed";
  }
  return "hold";
}

struct AdaptiveParticlePolicyConfig
{
  bool enabled{false};
  std::size_t min_particles{60};
  std::size_t max_particles{240};
  std::size_t confidence_frames{2};
  std::size_t failure_frames_before_reseed{3};
  double min_weighted_match_ratio{0.40};
  double max_xy_std{0.40};
  double max_yaw_std{0.15};
  double resample_effective_sample_ratio{0.50};
  double min_dominant_mode_mass{0.80};
  double max_pose_delta_xy{0.30};
  double max_pose_delta_yaw{0.15};
  double lost_match_ratio{0.08};
  double lost_max_xy_std{1.50};
  double lost_max_yaw_std{0.80};
  double kld_error{0.10};
  double kld_z{2.326};
  double kld_regression_tolerance_fraction{0.25};
  double max_reduction_fraction{0.25};
  double legacy_keep_ratio{0.75};
};

struct AdaptiveParticleObservation
{
  bool measurement_valid{false};
  bool pose_delta_valid{false};
  std::size_t current_particles{0};
  std::size_t occupied_bins{0};
  double weighted_match_ratio{0.0};
  double xy_std{std::numeric_limits<double>::infinity()};
  double yaw_std{std::numeric_limits<double>::infinity()};
  double effective_sample_ratio{0.0};
  double dominant_mode_mass{0.0};
  double pose_delta_xy{std::numeric_limits<double>::infinity()};
  double pose_delta_yaw{std::numeric_limits<double>::infinity()};
};

struct AdaptiveParticleDecision
{
  ParticlePolicyAction action{ParticlePolicyAction::HOLD};
  std::size_t target_particles{0};
  std::size_t kld_required_particles{0};
  std::size_t confidence_frames{0};
  bool raw_posterior_stable{false};
  bool posterior_stable{false};
  bool safe_to_reduce{false};
  bool resample_required{false};
};

class AdaptiveParticlePolicy
{
public:
  explicit AdaptiveParticlePolicy(AdaptiveParticlePolicyConfig config = {})
  : config_(sanitize(config))
  {
  }

  void reset()
  {
    confidence_frames_ = 0;
    failure_frames_ = 0;
    has_reduced_ = false;
  }

  const AdaptiveParticlePolicyConfig& config() const
  {
    return config_;
  }

  AdaptiveParticleDecision observe(const AdaptiveParticleObservation& observation)
  {
    AdaptiveParticleDecision decision;
    const std::size_t current = std::max<std::size_t>(1, observation.current_particles);
    decision.target_particles = current;
    decision.kld_required_particles = current;

    if (!config_.enabled)
    {
      confidence_frames_ = 0;
      failure_frames_ = 0;
      decision.resample_required = observation.measurement_valid;
      decision.posterior_stable = current <= config_.min_particles;
      if (current > config_.min_particles)
      {
        decision.target_particles = legacyTarget(current);
        decision.action = ParticlePolicyAction::REDUCE;
      }
      return decision;
    }

    decision.kld_required_particles = observation.occupied_bins > 0 ?
        requiredParticles(observation.occupied_bins) : current;
    // Occupied-bin counts fluctuate slightly after resampling. Only treat a
    // larger demand increase as evidence that the reduced population can no
    // longer represent the posterior.
    const double kld_regression_limit = static_cast<double>(current) *
        (1.0 + config_.kld_regression_tolerance_fraction);
    const bool materially_under_provisioned =
        static_cast<double>(decision.kld_required_particles) >
          kld_regression_limit;
    const bool severe_regression = isSevereRegression(observation);

    decision.raw_posterior_stable =
        isPosteriorUnambiguous(observation) &&
        !severe_regression && !materially_under_provisioned;
    if (decision.raw_posterior_stable)
    {
      confidence_frames_++;
    }
    else
    {
      confidence_frames_ = 0;
    }

    decision.confidence_frames = confidence_frames_;
    decision.posterior_stable =
        confidence_frames_ >= config_.confidence_frames;

    if (has_reduced_ &&
        (severe_regression || materially_under_provisioned))
    {
      failure_frames_++;
      decision.posterior_stable = false;
    }
    else
    {
      failure_frames_ = 0;
    }

    if (failure_frames_ >= config_.failure_frames_before_reseed)
    {
      decision.action = ParticlePolicyAction::RESEED;
      return decision;
    }

    decision.safe_to_reduce =
        decision.posterior_stable && isSafeToReduce(observation) &&
        !materially_under_provisioned;
    if (!decision.safe_to_reduce || current <= config_.min_particles)
    {
      decision.resample_required =
          observation.measurement_valid &&
          observation.effective_sample_ratio <=
            config_.resample_effective_sample_ratio;
      return decision;
    }

    const double keep_ratio = 1.0 - config_.max_reduction_fraction;
    const std::size_t per_frame_floor = static_cast<std::size_t>(
        std::ceil(static_cast<double>(current) * keep_ratio));
    decision.target_particles = std::min(
        current,
        std::max({
            config_.min_particles,
            decision.kld_required_particles,
            per_frame_floor}));
    if (decision.target_particles < current)
    {
      decision.action = ParticlePolicyAction::REDUCE;
      has_reduced_ = true;
    }
    decision.resample_required =
        decision.action == ParticlePolicyAction::REDUCE ||
        observation.effective_sample_ratio <=
          config_.resample_effective_sample_ratio;
    return decision;
  }

  std::size_t requiredParticles(const std::size_t occupied_bins) const
  {
    if (occupied_bins <= 1)
    {
      return config_.min_particles;
    }

    const double k_minus_one = static_cast<double>(occupied_bins - 1);
    const double correction =
        1.0 - 2.0 / (9.0 * k_minus_one) +
        config_.kld_z * std::sqrt(2.0 / (9.0 * k_minus_one));
    const double required =
        k_minus_one / (2.0 * config_.kld_error) *
        correction * correction * correction;
    if (!std::isfinite(required))
    {
      return config_.max_particles;
    }
    return std::clamp(
        static_cast<std::size_t>(std::ceil(std::max(0.0, required))),
        config_.min_particles,
        config_.max_particles);
  }

private:
  static AdaptiveParticlePolicyConfig sanitize(AdaptiveParticlePolicyConfig config)
  {
    config.min_particles = std::max<std::size_t>(1, config.min_particles);
    config.max_particles = std::max(config.min_particles, config.max_particles);
    config.confidence_frames = std::max<std::size_t>(1, config.confidence_frames);
    config.failure_frames_before_reseed =
        std::max<std::size_t>(1, config.failure_frames_before_reseed);
    config.min_weighted_match_ratio = std::clamp(
        finiteOr(config.min_weighted_match_ratio, 0.40), 0.0, 1.0);
    config.max_xy_std = std::max(0.0, finiteOr(config.max_xy_std, 0.40));
    config.max_yaw_std = std::max(0.0, finiteOr(config.max_yaw_std, 0.15));
    config.resample_effective_sample_ratio = std::clamp(
        finiteOr(config.resample_effective_sample_ratio, 0.50), 0.0, 1.0);
    config.min_dominant_mode_mass = std::clamp(
        finiteOr(config.min_dominant_mode_mass, 0.80), 0.0, 1.0);
    config.max_pose_delta_xy = std::max(
        0.0, finiteOr(config.max_pose_delta_xy, 0.30));
    config.max_pose_delta_yaw = std::max(
        0.0, finiteOr(config.max_pose_delta_yaw, 0.15));
    config.lost_match_ratio = std::clamp(
        finiteOr(config.lost_match_ratio, 0.08), 0.0, 1.0);
    config.lost_max_xy_std = std::max(
        config.max_xy_std, finiteOr(config.lost_max_xy_std, 1.50));
    config.lost_max_yaw_std = std::max(
        config.max_yaw_std, finiteOr(config.lost_max_yaw_std, 0.80));
    config.kld_error = std::max(1e-6, finiteOr(config.kld_error, 0.10));
    config.kld_z = std::max(0.0, finiteOr(config.kld_z, 2.326));
    config.kld_regression_tolerance_fraction = std::clamp(
        finiteOr(config.kld_regression_tolerance_fraction, 0.25), 0.0, 1.0);
    config.max_reduction_fraction = std::clamp(
        finiteOr(config.max_reduction_fraction, 0.25), 0.0, 0.95);
    config.legacy_keep_ratio = std::clamp(
        finiteOr(config.legacy_keep_ratio, 0.75), 0.05, 0.95);
    return config;
  }

  static double finiteOr(const double value, const double fallback)
  {
    return std::isfinite(value) ? value : fallback;
  }

  bool hasFinitePosterior(const AdaptiveParticleObservation& observation) const
  {
    return
        std::isfinite(observation.weighted_match_ratio) &&
        std::isfinite(observation.xy_std) && observation.xy_std >= 0.0 &&
        std::isfinite(observation.yaw_std) && observation.yaw_std >= 0.0 &&
        std::isfinite(observation.effective_sample_ratio) &&
        std::isfinite(observation.dominant_mode_mass) &&
        std::isfinite(observation.pose_delta_xy) && observation.pose_delta_xy >= 0.0 &&
        std::isfinite(observation.pose_delta_yaw) && observation.pose_delta_yaw >= 0.0;
  }

  bool isPosteriorUnambiguous(const AdaptiveParticleObservation& observation) const
  {
    return
        observation.measurement_valid && observation.pose_delta_valid &&
        hasFinitePosterior(observation) &&
        observation.dominant_mode_mass >= config_.min_dominant_mode_mass &&
        observation.pose_delta_xy <= config_.max_pose_delta_xy &&
        observation.pose_delta_yaw <= config_.max_pose_delta_yaw;
  }

  bool isSafeToReduce(const AdaptiveParticleObservation& observation) const
  {
    return
        observation.weighted_match_ratio >= config_.min_weighted_match_ratio &&
        observation.xy_std <= config_.max_xy_std &&
        observation.yaw_std <= config_.max_yaw_std &&
        observation.effective_sample_ratio >=
          config_.resample_effective_sample_ratio;
  }

  bool isSevereRegression(const AdaptiveParticleObservation& observation) const
  {
    return
        !observation.measurement_valid || !hasFinitePosterior(observation) ||
        observation.weighted_match_ratio < config_.lost_match_ratio ||
        observation.xy_std > config_.lost_max_xy_std ||
        observation.yaw_std > config_.lost_max_yaw_std;
  }

  std::size_t legacyTarget(const std::size_t current) const
  {
    const std::size_t scaled = static_cast<std::size_t>(
        static_cast<double>(current) * config_.legacy_keep_ratio);
    return std::max(
        config_.min_particles,
        std::min(current - 1, scaled));
  }

  AdaptiveParticlePolicyConfig config_;
  std::size_t confidence_frames_{0};
  std::size_t failure_frames_{0};
  bool has_reduced_{false};
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_ADAPTIVE_PARTICLE_POLICY_H
