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

#ifndef MCL_3DL_POSTERIOR_MODE_H
#define MCL_3DL_POSTERIOR_MODE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{

struct PosteriorModeEstimate
{
  bool valid{false};
  State6DOF mean;
  State6DOF anchor;
  double probability_mass{0.0};
  double weighted_match_ratio{0.0};
  double xy_std{std::numeric_limits<double>::infinity()};
  double yaw_std{std::numeric_limits<double>::infinity()};
  std::array<double, 6> variances{
    {std::numeric_limits<double>::infinity(),
     std::numeric_limits<double>::infinity(),
     std::numeric_limits<double>::infinity(),
     std::numeric_limits<double>::infinity(),
     std::numeric_limits<double>::infinity(),
     std::numeric_limits<double>::infinity()}};
  std::vector<unsigned char> members;
};

inline bool finiteState(const State6DOF& state)
{
  return
      std::isfinite(state.pos_.x_) && std::isfinite(state.pos_.y_) &&
      std::isfinite(state.pos_.z_) && std::isfinite(state.rot_.x_) &&
      std::isfinite(state.rot_.y_) && std::isfinite(state.rot_.z_) &&
      std::isfinite(state.rot_.w_);
}

inline bool posteriorBinIndex(
    const double value, const double bin_size, long long& output)
{
  if (!std::isfinite(value) || !std::isfinite(bin_size) || bin_size <= 0.0)
  {
    return false;
  }
  const double index = std::floor(value / bin_size);
  if (!std::isfinite(index) ||
      index < static_cast<double>(std::numeric_limits<long long>::lowest()) ||
      index >= static_cast<double>(std::numeric_limits<long long>::max()))
  {
    return false;
  }
  output = static_cast<long long>(index);
  return true;
}

inline PosteriorModeEstimate estimatePosteriorMode(
    const std::vector<State6DOF>& states,
    const std::vector<double>& probabilities,
    const std::vector<float>& match_ratios,
    const double xy_radius,
    const double yaw_radius)
{
  PosteriorModeEstimate estimate;
  if (states.empty() || states.size() != probabilities.size() ||
      states.size() != match_ratios.size() || !std::isfinite(xy_radius) ||
      !std::isfinite(yaw_radius) || xy_radius < 0.0 || yaw_radius < 0.0)
  {
    return estimate;
  }

  double probability_sum = 0.0;
  std::vector<double> yaws(states.size(), 0.0);
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    if (!finiteState(states[i]) || !std::isfinite(probabilities[i]) ||
        probabilities[i] < 0.0 || !std::isfinite(match_ratios[i]))
    {
      return estimate;
    }
    probability_sum += probabilities[i];
    yaws[i] = states[i].rot_.getRPY().z_;
  }
  if (!std::isfinite(probability_sum) || probability_sum <= 0.0)
  {
    return estimate;
  }

  const double radius_squared = xy_radius * xy_radius;
  std::size_t anchor_index = 0;
  double best_mode_probability = -1.0;
  for (std::size_t candidate = 0; candidate < states.size(); ++candidate)
  {
    if (probabilities[candidate] <= 0.0)
    {
      continue;
    }
    double candidate_probability = 0.0;
    bool local_weight_maximum = true;
    for (std::size_t i = 0; i < states.size(); ++i)
    {
      const double dx = states[i].pos_.x_ - states[candidate].pos_.x_;
      const double dy = states[i].pos_.y_ - states[candidate].pos_.y_;
      const double yaw_delta = std::atan2(
          std::sin(yaws[i] - yaws[candidate]),
          std::cos(yaws[i] - yaws[candidate]));
      if (dx * dx + dy * dy <= radius_squared &&
          std::abs(yaw_delta) <= yaw_radius)
      {
        candidate_probability += probabilities[i];
        if (probabilities[i] > probabilities[candidate])
        {
          local_weight_maximum = false;
        }
      }
    }
    if (!local_weight_maximum)
    {
      continue;
    }
    if (candidate_probability > best_mode_probability ||
        (candidate_probability == best_mode_probability &&
         probabilities[candidate] > probabilities[anchor_index]))
    {
      best_mode_probability = candidate_probability;
      anchor_index = candidate;
    }
  }

  estimate.anchor = states[anchor_index];
  const double anchor_yaw = yaws[anchor_index];
  estimate.members.assign(states.size(), 0);
  double mode_probability = 0.0;
  ParticleWeightedMeanQuat mode_mean;
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    const double dx = states[i].pos_.x_ - estimate.anchor.pos_.x_;
    const double dy = states[i].pos_.y_ - estimate.anchor.pos_.y_;
    const double yaw_delta = std::atan2(
        std::sin(yaws[i] - anchor_yaw), std::cos(yaws[i] - anchor_yaw));
    if (dx * dx + dy * dy > radius_squared ||
        std::abs(yaw_delta) > yaw_radius)
    {
      continue;
    }
    estimate.members[i] = 1;
    mode_probability += probabilities[i];
    estimate.weighted_match_ratio += probabilities[i] * match_ratios[i];
    mode_mean.add(states[i], static_cast<float>(probabilities[i]));
  }
  if (!std::isfinite(mode_probability) || mode_probability <= 0.0)
  {
    return estimate;
  }

  estimate.mean = mode_mean.getMean();
  estimate.mean.rot_.normalize();
  if (!finiteState(estimate.mean))
  {
    return estimate;
  }
  estimate.probability_mass = mode_probability / probability_sum;
  estimate.weighted_match_ratio /= mode_probability;
  estimate.variances.fill(0.0);
  const Vec3 mean_rpy = estimate.mean.rot_.getRPY();
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    if (!estimate.members[i])
    {
      continue;
    }
    const double dx = states[i].pos_.x_ - estimate.mean.pos_.x_;
    const double dy = states[i].pos_.y_ - estimate.mean.pos_.y_;
    const double dz = states[i].pos_.z_ - estimate.mean.pos_.z_;
    const Vec3 rpy = states[i].rot_.getRPY();
    const double droll = std::atan2(
        std::sin(rpy.x_ - mean_rpy.x_), std::cos(rpy.x_ - mean_rpy.x_));
    const double dpitch = std::atan2(
        std::sin(rpy.y_ - mean_rpy.y_), std::cos(rpy.y_ - mean_rpy.y_));
    const double dyaw = std::atan2(
        std::sin(rpy.z_ - mean_rpy.z_), std::cos(rpy.z_ - mean_rpy.z_));
    const std::array<double, 6> errors{{dx, dy, dz, droll, dpitch, dyaw}};
    for (std::size_t dimension = 0; dimension < errors.size(); ++dimension)
    {
      estimate.variances[dimension] +=
          probabilities[i] * errors[dimension] * errors[dimension];
    }
  }
  for (double& variance : estimate.variances)
  {
    variance /= mode_probability;
  }
  estimate.xy_std = std::sqrt(std::max(
      0.0, estimate.variances[0] + estimate.variances[1]));
  estimate.yaw_std = std::sqrt(std::max(0.0, estimate.variances[5]));
  estimate.valid =
      std::isfinite(estimate.probability_mass) &&
      std::isfinite(estimate.weighted_match_ratio) &&
      std::isfinite(estimate.xy_std) && std::isfinite(estimate.yaw_std) &&
      std::all_of(
          estimate.variances.begin(), estimate.variances.end(),
          [](const double variance)
          {
            return std::isfinite(variance) && variance >= 0.0;
          });
  return estimate;
}

}  // namespace mcl_3dl

#endif  // MCL_3DL_POSTERIOR_MODE_H
