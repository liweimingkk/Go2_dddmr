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

#include "perception_3d/terrain_edge_policy.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception_3d
{
namespace
{

constexpr float kGeometryEpsilon = 1.0e-5F;
constexpr float kFractionEpsilon = 1.0e-4F;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kHalfPi = kPi / 2.0F;

TerrainEdgeResult reject(
  TerrainEdgeResult result,
  TerrainRejectionReason reason)
{
  result.accepted = false;
  result.reason = reason;
  result.traversal_cost = 0.0F;
  return result;
}

bool finiteNonnegative(float value)
{
  return std::isfinite(value) && value >= 0.0F;
}

bool finiteRatio(float value)
{
  return finiteNonnegative(value) && value <= 1.0F;
}

bool exceedsUpperBound(float value, float limit)
{
  // A zero capability is literal.  Do not let the normal floating-point
  // comparison tolerance turn it into a small implicit capability.
  return limit == 0.0F ? value > 0.0F : value > limit + kGeometryEpsilon;
}

bool validConfig(const TerrainEdgePolicyConfig & config)
{
  const float nonnegative_values[] = {
    config.max_up_slope_rad,
    config.max_down_slope_rad,
    config.max_cross_slope_rad,
    config.max_roughness_m,
    config.max_normal_change_rad,
    config.max_step_up_m,
    config.max_step_down_m,
    config.max_support_sample_spacing_m,
    config.max_stair_riser_height_m,
    config.min_stair_tread_depth_m,
    config.max_stair_tread_depth_m,
    config.max_stair_riser_deviation_m,
    config.max_stair_tread_deviation_m,
    config.max_stair_heading_error_rad,
    config.distance_cost_weight,
    config.slope_cost_weight,
    config.cross_slope_cost_weight,
    config.roughness_cost_weight,
    config.risk_cost_weight,
    config.stair_transition_cost};

  for (float value : nonnegative_values) {
    if (!finiteNonnegative(value)) {
      return false;
    }
  }
  return config.fail_closed && finiteRatio(config.min_support_ratio) &&
         finiteRatio(config.max_unknown_ratio) && finiteRatio(config.min_confidence) &&
         config.max_step_index_delta >= 0 &&
         config.max_stair_tread_depth_m >= config.min_stair_tread_depth_m &&
         config.max_up_slope_rad <= kHalfPi &&
         config.max_down_slope_rad <= kHalfPi &&
         config.max_cross_slope_rad <= kHalfPi &&
         config.max_normal_change_rad <= kPi &&
         config.max_stair_heading_error_rad <= kPi;
}

Eigen::Vector3f upwardNormal(const Eigen::Vector3f & input)
{
  Eigen::Vector3f normal = input.normalized();
  if (normal.z() < 0.0F) {
    normal = -normal;
  }
  return normal;
}

float angleBetween(const Eigen::Vector3f & first, const Eigen::Vector3f & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0F, 1.0F));
}

TerrainRejectionReason endpointClassReason(TerrainClass terrain_class)
{
  switch (terrain_class) {
    case TerrainClass::UNKNOWN:
      return TerrainRejectionReason::UNKNOWN;
    case TerrainClass::EDGE:
    case TerrainClass::DROP:
      return TerrainRejectionReason::DROP;
    case TerrainClass::STAIR_RISER:
      return TerrainRejectionReason::NO_SUPPORT;
    case TerrainClass::FLAT:
    case TerrainClass::RAMP:
    case TerrainClass::STAIR_TREAD:
      return TerrainRejectionReason::NONE;
  }
  return TerrainRejectionReason::UNKNOWN;
}

bool hasFlag(std::uint32_t flags, TerrainNodeFlag flag)
{
  return (flags & static_cast<std::uint32_t>(flag)) != 0U;
}

bool isLandingNode(const TerrainNode & node)
{
  return hasFlag(node.flags, TERRAIN_NODE_LANDING) &&
         (node.terrain_class == TerrainClass::FLAT ||
         node.terrain_class == TerrainClass::RAMP);
}

TerrainRejectionReason validateSupportSamples(
  const TerrainEdgeRequest & request,
  const TerrainEdgePolicyConfig & config,
  const TerrainNode & from_node,
  const TerrainNode & to_node,
  float edge_length,
  TerrainEdgeResult * result)
{
  result->minimum_support_ratio = 1.0F;
  result->maximum_unknown_ratio = 0.0F;
  result->maximum_abs_slope_rad = std::max(
    result->maximum_abs_slope_rad, std::abs(result->signed_slope_rad));

  if (request.support_samples.size() < 2U || request.support_samples.size() > 100000U) {
    return TerrainRejectionReason::NO_SUPPORT;
  }
  if (config.max_support_sample_spacing_m <= 0.0F) {
    return TerrainRejectionReason::NO_SUPPORT;
  }

  float previous_fraction = -1.0F;
  Eigen::Vector3f previous_normal = Eigen::Vector3f::Zero();
  bool has_previous_normal = false;
  const Eigen::Vector2f horizontal_direction =
    (request.to_position - request.from_position).head<2>().normalized();
  const Eigen::Vector3f lateral_direction(
    -horizontal_direction.y(), horizontal_direction.x(), 0.0F);
  for (std::size_t index = 0; index < request.support_samples.size(); ++index) {
    const auto & sample = request.support_samples[index];
    if (
      !std::isfinite(sample.edge_fraction) || sample.edge_fraction < 0.0F ||
      sample.edge_fraction > 1.0F || !finiteRatio(sample.support_ratio) ||
      !finiteRatio(sample.unknown_ratio) || !finiteRatio(sample.confidence) ||
      !sample.normal.allFinite() || sample.normal.norm() <= kGeometryEpsilon ||
      !finiteNonnegative(sample.roughness_m))
    {
      return TerrainRejectionReason::INVALID_GEOMETRY;
    }
    if (index == 0U && sample.edge_fraction > kFractionEpsilon) {
      return TerrainRejectionReason::NO_SUPPORT;
    }
    if (index > 0U) {
      if (sample.edge_fraction <= previous_fraction) {
        return TerrainRejectionReason::INVALID_GEOMETRY;
      }
      const float sample_gap = (sample.edge_fraction - previous_fraction) * edge_length;
      if (sample_gap > config.max_support_sample_spacing_m + kGeometryEpsilon) {
        return TerrainRejectionReason::NO_SUPPORT;
      }
    }
    previous_fraction = sample.edge_fraction;

    result->minimum_support_ratio =
      std::min(result->minimum_support_ratio, sample.support_ratio);
    result->maximum_unknown_ratio =
      std::max(result->maximum_unknown_ratio, sample.unknown_ratio);

    const Eigen::Vector3f sample_normal = upwardNormal(sample.normal);
    const float longitudinal_slope = std::atan2(
      -sample_normal.head<2>().dot(horizontal_direction),
      std::max(std::abs(sample_normal.z()), kGeometryEpsilon));
    const float cross_slope = std::atan2(
      std::abs(sample_normal.dot(lateral_direction)),
      std::max(std::abs(sample_normal.z()), kGeometryEpsilon));
    result->maximum_abs_slope_rad = std::max(
      result->maximum_abs_slope_rad, std::abs(longitudinal_slope));
    result->cross_slope_rad = std::max(result->cross_slope_rad, cross_slope);
    result->maximum_roughness_m = std::max(
      result->maximum_roughness_m, sample.roughness_m);
    if (has_previous_normal) {
      result->normal_change_rad = std::max(
        result->normal_change_rad, angleBetween(previous_normal, sample_normal));
    }
    previous_normal = sample_normal;
    has_previous_normal = true;

    if (longitudinal_slope > 0.0F &&
      exceedsUpperBound(longitudinal_slope, config.max_up_slope_rad))
    {
      return TerrainRejectionReason::SLOPE;
    }
    if (longitudinal_slope < 0.0F &&
      exceedsUpperBound(-longitudinal_slope, config.max_down_slope_rad))
    {
      return TerrainRejectionReason::SLOPE;
    }
    if (exceedsUpperBound(cross_slope, config.max_cross_slope_rad)) {
      return TerrainRejectionReason::CROSS_SLOPE;
    }
    if (exceedsUpperBound(sample.roughness_m, config.max_roughness_m)) {
      return TerrainRejectionReason::ROUGHNESS;
    }
    if (exceedsUpperBound(result->normal_change_rad, config.max_normal_change_rad)) {
      return TerrainRejectionReason::NORMAL_CHANGE;
    }

    const TerrainRejectionReason class_reason = endpointClassReason(sample.terrain_class);
    if (class_reason != TerrainRejectionReason::NONE) {
      return class_reason;
    }
    if (sample.confidence < config.min_confidence) {
      return TerrainRejectionReason::LOW_CONFIDENCE;
    }
    if (sample.unknown_ratio > config.max_unknown_ratio + kGeometryEpsilon) {
      return TerrainRejectionReason::UNKNOWN;
    }
    if (sample.support_ratio + kGeometryEpsilon < config.min_support_ratio) {
      return TerrainRejectionReason::NO_SUPPORT;
    }

    if (request.mode == TerrainEdgeMode::CONTINUOUS_SURFACE) {
      if (sample.surface_id != from_node.surface_id || sample.surface_id != to_node.surface_id) {
        return TerrainRejectionReason::LAYER_MISMATCH;
      }
    } else if (request.mode == TerrainEdgeMode::GENERIC_STEP) {
      if (sample.surface_id != from_node.surface_id || sample.surface_id != to_node.surface_id) {
        return TerrainRejectionReason::LAYER_MISMATCH;
      }
    } else if (request.mode == TerrainEdgeMode::STAIR_TRANSITION) {
      const bool sample_is_tread = sample.terrain_class == TerrainClass::STAIR_TREAD;
      const bool sample_is_landing =
        hasFlag(sample.flags, TERRAIN_NODE_LANDING) &&
        (sample.terrain_class == TerrainClass::FLAT ||
        sample.terrain_class == TerrainClass::RAMP);
      if ((!sample_is_tread && !sample_is_landing) ||
        sample.staircase_id != from_node.staircase_id ||
        sample.staircase_id != to_node.staircase_id ||
        (sample_is_tread && sample.step_index != from_node.step_index &&
        sample.step_index != to_node.step_index))
      {
        return TerrainRejectionReason::NO_SUPPORT;
      }
      if (config.require_manual_corridor &&
        !hasFlag(sample.flags, TERRAIN_NODE_MANUAL_CORRIDOR))
      {
        return TerrainRejectionReason::OUTSIDE_CORRIDOR;
      }
      if (config.require_online_confirmation &&
        !hasFlag(sample.flags, TERRAIN_NODE_ONLINE_CONFIRMED))
      {
        return TerrainRejectionReason::LOW_CONFIDENCE;
      }
    }
  }

  if (previous_fraction < 1.0F - kFractionEpsilon) {
    return TerrainRejectionReason::NO_SUPPORT;
  }
  return TerrainRejectionReason::NONE;
}

TerrainRejectionReason evaluateContinuous(
  const TerrainEdgeRequest & request,
  const TerrainEdgePolicyConfig & config,
  const TerrainNode & from_node,
  const TerrainNode & to_node,
  const TerrainEdgeResult & result)
{
  if (from_node.surface_id != to_node.surface_id) {
    return TerrainRejectionReason::LAYER_MISMATCH;
  }
  if (
    from_node.terrain_class == TerrainClass::STAIR_TREAD ||
    to_node.terrain_class == TerrainClass::STAIR_TREAD)
  {
    if (
      from_node.terrain_class != TerrainClass::STAIR_TREAD ||
      to_node.terrain_class != TerrainClass::STAIR_TREAD ||
      from_node.staircase_id != to_node.staircase_id ||
      from_node.step_index != to_node.step_index)
    {
      return TerrainRejectionReason::STAIR_GEOMETRY;
    }
  }

  if (result.signed_slope_rad > 0.0F &&
    exceedsUpperBound(result.signed_slope_rad, config.max_up_slope_rad))
  {
    return TerrainRejectionReason::SLOPE;
  }
  if (result.signed_slope_rad < 0.0F &&
    exceedsUpperBound(-result.signed_slope_rad, config.max_down_slope_rad))
  {
    return TerrainRejectionReason::SLOPE;
  }
  if (exceedsUpperBound(result.cross_slope_rad, config.max_cross_slope_rad)) {
    return TerrainRejectionReason::CROSS_SLOPE;
  }
  if (
    exceedsUpperBound(
      std::max(from_node.roughness_m, to_node.roughness_m),
      config.max_roughness_m))
  {
    return TerrainRejectionReason::ROUGHNESS;
  }
  if (exceedsUpperBound(result.normal_change_rad, config.max_normal_change_rad)) {
    return TerrainRejectionReason::NORMAL_CHANGE;
  }
  (void)request;
  return TerrainRejectionReason::NONE;
}

TerrainRejectionReason evaluateGenericStep(
  const TerrainEdgeRequest & request,
  const TerrainEdgePolicyConfig & config,
  const TerrainNode & from_node,
  const TerrainNode & to_node)
{
  if (from_node.surface_id != to_node.surface_id) {
    return TerrainRejectionReason::LAYER_MISMATCH;
  }
  if (
    from_node.terrain_class == TerrainClass::STAIR_TREAD ||
    to_node.terrain_class == TerrainClass::STAIR_TREAD)
  {
    return TerrainRejectionReason::STAIR_GEOMETRY;
  }
  const float height_change = request.to_position.z() - request.from_position.z();
  if (std::abs(height_change) <= kGeometryEpsilon) {
    return TerrainRejectionReason::INVALID_GEOMETRY;
  }
  if (height_change > 0.0F && exceedsUpperBound(height_change, config.max_step_up_m)) {
    return TerrainRejectionReason::STEP_UP;
  }
  if (height_change < 0.0F && exceedsUpperBound(-height_change, config.max_step_down_m)) {
    return TerrainRejectionReason::STEP_DOWN;
  }
  return TerrainRejectionReason::NONE;
}

TerrainRejectionReason evaluateStairTransition(
  const TerrainSnapshot & snapshot,
  const TerrainEdgeRequest & request,
  const TerrainEdgePolicyConfig & config,
  const TerrainNode & from_node,
  const TerrainNode & to_node)
{
  if (!config.stair_enabled) {
    return TerrainRejectionReason::STAIR_DISABLED;
  }
  if (config.require_manual_corridor && !request.inside_manual_corridor) {
    return TerrainRejectionReason::OUTSIDE_CORRIDOR;
  }
  if (config.require_online_confirmation && !request.online_confirmation) {
    return TerrainRejectionReason::LOW_CONFIDENCE;
  }
  const bool from_is_tread = from_node.terrain_class == TerrainClass::STAIR_TREAD;
  const bool to_is_tread = to_node.terrain_class == TerrainClass::STAIR_TREAD;
  const bool from_is_landing = isLandingNode(from_node);
  const bool to_is_landing = isLandingNode(to_node);
  if (from_node.staircase_id < 0 ||
    from_node.staircase_id != to_node.staircase_id ||
    (!from_is_tread && !from_is_landing) ||
    (!to_is_tread && !to_is_landing) ||
    (from_is_landing && to_is_landing))
  {
    return TerrainRejectionReason::STAIR_GEOMETRY;
  }

  const StaircaseModel * staircase = snapshot.staircaseById(from_node.staircase_id);
  if (staircase == nullptr) {
    return TerrainRejectionReason::STAIR_MODEL_MISSING;
  }
  if (staircase->confidence < config.min_confidence) {
    return TerrainRejectionReason::LOW_CONFIDENCE;
  }
  const Eigen::Vector3f difference = request.to_position - request.from_position;
  const float height_change = difference.z();
  bool ascending = false;
  float expected_riser_height = staircase->riser_height_m;

  if (from_is_tread && to_is_tread) {
    const std::int32_t step_delta = to_node.step_index - from_node.step_index;
    if (step_delta == 0 || std::abs(step_delta) != 1 ||
      std::abs(step_delta) > config.max_step_index_delta)
    {
      return TerrainRejectionReason::SKIP_STEP;
    }
    if (from_node.step_index < 0 || to_node.step_index < 0 ||
      from_node.step_index >= staircase->step_count ||
      to_node.step_index >= staircase->step_count)
    {
      return TerrainRejectionReason::STAIR_GEOMETRY;
    }
    ascending = step_delta > 0;
    if ((ascending && height_change <= kGeometryEpsilon) ||
      (!ascending && height_change >= -kGeometryEpsilon))
    {
      return TerrainRejectionReason::STAIR_GEOMETRY;
    }
  } else {
    const TerrainNode & landing = from_is_landing ? from_node : to_node;
    const TerrainNode & tread = from_is_tread ? from_node : to_node;
    const bool lower_landing = hasFlag(landing.flags, TERRAIN_NODE_LOWER_LANDING);
    const bool upper_landing = hasFlag(landing.flags, TERRAIN_NODE_UPPER_LANDING);
    if (lower_landing == upper_landing) {
      return TerrainRejectionReason::NO_LANDING;
    }
    const std::int32_t expected_step = lower_landing ? 0 : staircase->step_count - 1;
    if (tread.step_index != expected_step) {
      return TerrainRejectionReason::SKIP_STEP;
    }
    ascending = lower_landing ? from_is_landing : to_is_landing;
    if (upper_landing) {
      // The annotated top tread and upper landing share the upper-floor
      // elevation.  Their connection is still a controlled stair transition,
      // but it must not invent an additional riser.
      expected_riser_height = 0.0F;
      if (std::abs(height_change) > config.max_stair_riser_deviation_m + kGeometryEpsilon) {
        return TerrainRejectionReason::STAIR_GEOMETRY;
      }
    } else if ((ascending && height_change <= kGeometryEpsilon) ||
      (!ascending && height_change >= -kGeometryEpsilon))
    {
      return TerrainRejectionReason::STAIR_GEOMETRY;
    }
  }

  if (ascending && (!config.allow_stair_up || !staircase->allow_up)) {
    return TerrainRejectionReason::DIRECTION_DISABLED;
  }
  if (!ascending && (!config.allow_stair_down || !staircase->allow_down)) {
    return TerrainRejectionReason::DIRECTION_DISABLED;
  }

  const float riser_height = std::abs(height_change);
  if (exceedsUpperBound(riser_height, config.max_stair_riser_height_m)) {
    return ascending ? TerrainRejectionReason::STEP_UP : TerrainRejectionReason::STEP_DOWN;
  }
  if (exceedsUpperBound(
      std::abs(riser_height - expected_riser_height),
      config.max_stair_riser_deviation_m))
  {
    return TerrainRejectionReason::STAIR_GEOMETRY;
  }

  Eigen::Vector3f horizontal_direction = difference;
  horizontal_direction.z() = 0.0F;
  const float tread_depth = horizontal_direction.norm();
  if (
    tread_depth < config.min_stair_tread_depth_m - kGeometryEpsilon ||
    exceedsUpperBound(tread_depth, config.max_stair_tread_depth_m) ||
    exceedsUpperBound(
      std::abs(tread_depth - staircase->tread_depth_m),
      config.max_stair_tread_deviation_m))
  {
    return TerrainRejectionReason::STAIR_GEOMETRY;
  }

  horizontal_direction.normalize();
  Eigen::Vector3f expected_direction = staircase->up_axis;
  expected_direction.z() = 0.0F;
  expected_direction.normalize();
  if (!ascending) {
    expected_direction = -expected_direction;
  }
  if (exceedsUpperBound(
      angleBetween(horizontal_direction, expected_direction),
      config.max_stair_heading_error_rad))
  {
    return TerrainRejectionReason::BAD_ALIGNMENT;
  }
  return TerrainRejectionReason::NONE;
}

}  // namespace

TerrainEdgeResult TerrainEdgePolicy::evaluate(
  const TerrainSnapshotConstPtr & snapshot,
  const TerrainEdgeRequest & request,
  const TerrainEdgePolicyConfig & config)
{
  TerrainEdgeResult result;
  if (!config.enabled) {
    return reject(result, TerrainRejectionReason::TERRAIN_DISABLED);
  }
  if (!config.fail_closed) {
    return reject(result, TerrainRejectionReason::FAIL_OPEN_CONFIGURATION);
  }
  if (!validConfig(config)) {
    return reject(result, TerrainRejectionReason::INVALID_CAPABILITY);
  }
  if (!snapshot) {
    return reject(result, TerrainRejectionReason::MISSING_SNAPSHOT);
  }
  if (!snapshot->valid()) {
    return reject(result, TerrainRejectionReason::INVALID_SNAPSHOT);
  }
  if (request.expected_map_hash.empty() || request.expected_map_hash != snapshot->mapHash()) {
    return reject(result, TerrainRejectionReason::MAP_MISMATCH);
  }
  if (
    request.expected_snapshot_version == 0U ||
    request.expected_snapshot_version != snapshot->version())
  {
    return reject(result, TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH);
  }
  if (request.dynamic_obstacle) {
    return reject(result, TerrainRejectionReason::DYNAMIC_OBSTACLE);
  }

  const TerrainNode * from_node = snapshot->nodeAt(request.from_index);
  const TerrainNode * to_node = snapshot->nodeAt(request.to_index);
  if (from_node == nullptr || to_node == nullptr) {
    return reject(result, TerrainRejectionReason::INDEX_OUT_OF_RANGE);
  }
  if (
    request.from_index == request.to_index || !request.from_position.allFinite() ||
    !request.to_position.allFinite() || request.mode == TerrainEdgeMode::UNKNOWN)
  {
    return reject(result, TerrainRejectionReason::INVALID_GEOMETRY);
  }

  const Eigen::Vector3f difference = request.to_position - request.from_position;
  result.edge_length_m = difference.norm();
  const float horizontal_distance = difference.head<2>().norm();
  if (!std::isfinite(result.edge_length_m) || horizontal_distance <= kGeometryEpsilon) {
    return reject(result, TerrainRejectionReason::INVALID_GEOMETRY);
  }
  result.signed_slope_rad = std::atan2(difference.z(), horizontal_distance);

  const TerrainRejectionReason from_class_reason =
    endpointClassReason(from_node->terrain_class);
  if (from_class_reason != TerrainRejectionReason::NONE) {
    return reject(result, from_class_reason);
  }
  const TerrainRejectionReason to_class_reason = endpointClassReason(to_node->terrain_class);
  if (to_class_reason != TerrainRejectionReason::NONE) {
    return reject(result, to_class_reason);
  }
  if (
    from_node->confidence < config.min_confidence ||
    to_node->confidence < config.min_confidence)
  {
    return reject(result, TerrainRejectionReason::LOW_CONFIDENCE);
  }
  if (
    from_node->support_ratio + kGeometryEpsilon < config.min_support_ratio ||
    to_node->support_ratio + kGeometryEpsilon < config.min_support_ratio)
  {
    return reject(result, TerrainRejectionReason::NO_SUPPORT);
  }

  const Eigen::Vector3f from_normal = upwardNormal(from_node->normal);
  const Eigen::Vector3f to_normal = upwardNormal(to_node->normal);
  result.normal_change_rad = angleBetween(from_normal, to_normal);
  result.maximum_roughness_m = std::max(from_node->roughness_m, to_node->roughness_m);
  if (exceedsUpperBound(result.maximum_roughness_m, config.max_roughness_m)) {
    return reject(result, TerrainRejectionReason::ROUGHNESS);
  }
  if (exceedsUpperBound(result.normal_change_rad, config.max_normal_change_rad)) {
    return reject(result, TerrainRejectionReason::NORMAL_CHANGE);
  }
  Eigen::Vector3f average_normal = from_normal + to_normal;
  if (average_normal.norm() <= kGeometryEpsilon) {
    return reject(result, TerrainRejectionReason::INVALID_GEOMETRY);
  }
  average_normal.normalize();
  const Eigen::Vector2f horizontal_direction = difference.head<2>().normalized();
  const Eigen::Vector3f lateral_direction(
    -horizontal_direction.y(), horizontal_direction.x(), 0.0F);
  result.cross_slope_rad = std::atan2(
    std::abs(average_normal.dot(lateral_direction)),
    std::max(std::abs(average_normal.z()), kGeometryEpsilon));

  const TerrainRejectionReason support_reason = validateSupportSamples(
    request, config, *from_node, *to_node, result.edge_length_m, &result);
  if (support_reason != TerrainRejectionReason::NONE) {
    return reject(result, support_reason);
  }

  TerrainRejectionReason mode_reason = TerrainRejectionReason::INVALID_GEOMETRY;
  switch (request.mode) {
    case TerrainEdgeMode::CONTINUOUS_SURFACE:
      mode_reason = evaluateContinuous(request, config, *from_node, *to_node, result);
      break;
    case TerrainEdgeMode::GENERIC_STEP:
      mode_reason = evaluateGenericStep(request, config, *from_node, *to_node);
      break;
    case TerrainEdgeMode::STAIR_TRANSITION:
      mode_reason = evaluateStairTransition(
        *snapshot, request, config, *from_node, *to_node);
      break;
    case TerrainEdgeMode::UNKNOWN:
      break;
  }
  if (mode_reason != TerrainRejectionReason::NONE) {
    return reject(result, mode_reason);
  }

  const float roughness = std::max(
    result.maximum_roughness_m,
    std::max(from_node->roughness_m, to_node->roughness_m));
  const float risk =
    (1.0F - result.minimum_support_ratio) + result.maximum_unknown_ratio +
    (1.0F - std::min(from_node->confidence, to_node->confidence));
  result.traversal_cost =
    config.distance_cost_weight * result.edge_length_m +
    config.slope_cost_weight * result.maximum_abs_slope_rad +
    config.cross_slope_cost_weight * result.cross_slope_rad +
    config.roughness_cost_weight * roughness + config.risk_cost_weight * risk;
  if (request.mode == TerrainEdgeMode::STAIR_TRANSITION) {
    result.traversal_cost += config.stair_transition_cost;
  }
  if (!finiteNonnegative(result.traversal_cost)) {
    return reject(result, TerrainRejectionReason::INVALID_CAPABILITY);
  }

  result.accepted = true;
  result.reason = TerrainRejectionReason::NONE;
  return result;
}

}  // namespace perception_3d
