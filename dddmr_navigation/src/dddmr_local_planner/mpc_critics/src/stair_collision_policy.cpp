/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "mpc_critics/stair_collision_policy.h"

#include <cmath>
#include <string>

namespace mpc_critics
{
namespace
{

constexpr float kAxisNormTolerance = 1.0e-3F;
constexpr float kAxisDotTolerance = 1.0e-3F;
constexpr float kEnvelopeTolerance = 1.0e-6F;

bool setError(std::string * error, const char * message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool validAxes(const Eigen::Matrix3f & axes)
{
  if (!axes.allFinite()) {
    return false;
  }
  for (Eigen::Index column = 0; column < 3; ++column) {
    if (std::abs(axes.col(column).norm() - 1.0F) > kAxisNormTolerance) {
      return false;
    }
  }
  return std::abs(axes.col(0).dot(axes.col(1))) <= kAxisDotTolerance &&
         std::abs(axes.col(0).dot(axes.col(2))) <= kAxisDotTolerance &&
         std::abs(axes.col(1).dot(axes.col(2))) <= kAxisDotTolerance;
}

}  // namespace

const char * toString(StairCollisionReason reason) noexcept
{
  switch (reason) {
    case StairCollisionReason::PASSTHROUGH_EXPECTED_RISER:
      return "PASSTHROUGH_EXPECTED_RISER";
    case StairCollisionReason::RISER_SEMANTICS_REJECTED:
      return "RISER_SEMANTICS_REJECTED";
    case StairCollisionReason::INVALID_LEG_ENVELOPE: return "INVALID_LEG_ENVELOPE";
    case StairCollisionReason::INVALID_BODY_FRAME: return "INVALID_BODY_FRAME";
    case StairCollisionReason::MISSING_ACTIVE_STAIR_TRANSITION:
      return "MISSING_ACTIVE_STAIR_TRANSITION";
    case StairCollisionReason::NOT_ADJACENT_TRANSITION: return "NOT_ADJACENT_TRANSITION";
    case StairCollisionReason::OUTSIDE_LEG_ENVELOPE: return "OUTSIDE_LEG_ENVELOPE";
  }
  return "INVALID_STAIR_COLLISION_REASON";
}

bool StairCollisionPolicy::validConfig(
  const StairCollisionPolicyConfig & config,
  std::string * error)
{
  if (!perception_3d::StairRiserSemantics::validConfig(
      config.riser_semantics, error))
  {
    return false;
  }
  if (!config.riser_semantics.enabled) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  if (!config.leg_envelope_min.allFinite() || !config.leg_envelope_max.allFinite() ||
    (config.leg_envelope_max.array() <= config.leg_envelope_min.array()).any())
  {
    return setError(error, "leg pass-through envelope is empty or non-finite");
  }
  if (config.leg_envelope_max.z() > kEnvelopeTolerance) {
    return setError(error, "leg pass-through envelope may not extend above body-frame z=0");
  }
  if (!std::isfinite(config.max_support_xy_distance_m) ||
    config.max_support_xy_distance_m <= 0.0F ||
    !std::isfinite(config.min_body_clearance_m) ||
    config.min_body_clearance_m < 0.0F ||
    !std::isfinite(config.max_body_clearance_m) ||
    config.max_body_clearance_m <= config.min_body_clearance_m)
  {
    return setError(error, "active stair support bounds are invalid or unverified");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

StairCollisionResult StairCollisionPolicy::evaluate(
  const StairCollisionPolicyConfig & config,
  const StairCollisionQuery & query)
{
  StairCollisionResult result;
  if (!validConfig(config)) {
    result.reason = StairCollisionReason::INVALID_LEG_ENVELOPE;
    result.riser_reason = perception_3d::StairRiserSemanticReason::INVALID_CONFIGURATION;
    return result;
  }

  const auto riser_result = perception_3d::StairRiserSemantics::classify(
    config.riser_semantics, query.riser_observation);
  result.riser_reason = riser_result.reason;
  result.staircase_id = riser_result.staircase_id;
  result.step_index = riser_result.step_index;
  if (!riser_result.expected_riser) {
    result.reason = StairCollisionReason::RISER_SEMANTICS_REJECTED;
    return result;
  }
  if (!query.trajectory_origin_world.allFinite() || !validAxes(query.body_axes_world)) {
    result.reason = StairCollisionReason::INVALID_BODY_FRAME;
    return result;
  }
  const auto * staircase = query.riser_observation.snapshot ?
    query.riser_observation.snapshot->staircaseById(riser_result.staircase_id) : nullptr;
  if (staircase == nullptr || query.active_support_staircase_id < 0 ||
    query.active_support_location < -1 ||
    query.active_support_location > (staircase ? staircase->step_count : -1))
  {
    result.reason = StairCollisionReason::MISSING_ACTIVE_STAIR_TRANSITION;
    return result;
  }
  if (query.active_support_staircase_id != riser_result.staircase_id ||
    !perception_3d::stairRiserSupportLocationIsAdjacent(
      riser_result.step_index, query.active_support_location,
      staircase->step_count))
  {
    result.reason = StairCollisionReason::NOT_ADJACENT_TRANSITION;
    return result;
  }
  const Eigen::Vector3f local_point = query.body_axes_world.transpose() *
    (query.riser_observation.obstacle_position - query.trajectory_origin_world);
  if ((local_point.array() <
      (config.leg_envelope_min.array() - kEnvelopeTolerance)).any() ||
    (local_point.array() >
      (config.leg_envelope_max.array() + kEnvelopeTolerance)).any())
  {
    result.reason = StairCollisionReason::OUTSIDE_LEG_ENVELOPE;
    return result;
  }

  result.passthrough = true;
  result.reason = StairCollisionReason::PASSTHROUGH_EXPECTED_RISER;
  return result;
}

}  // namespace mpc_critics
