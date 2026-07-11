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

#include "perception_3d/stair_riser_semantics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace perception_3d
{
namespace
{

constexpr float kGeometryEpsilon = 1.0e-6F;

bool setError(std::string * error, const char * message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool pointOnSegment(
  const Eigen::Vector2f & point,
  const Eigen::Vector2f & first,
  const Eigen::Vector2f & second)
{
  const Eigen::Vector2f segment = second - first;
  const Eigen::Vector2f offset = point - first;
  const float cross = segment.x() * offset.y() - segment.y() * offset.x();
  if (std::abs(cross) > 1.0e-5F) {
    return false;
  }
  const float projection = offset.dot(segment);
  return projection >= -1.0e-5F &&
         projection <= segment.squaredNorm() + 1.0e-5F;
}

bool pointInPolygon(
  const Eigen::Vector2f & point,
  const std::vector<Eigen::Vector2f> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }
  bool inside = false;
  for (std::size_t index = 0U, previous = polygon.size() - 1U;
    index < polygon.size(); previous = index++)
  {
    const auto & first = polygon[previous];
    const auto & second = polygon[index];
    if (pointOnSegment(point, first, second)) {
      return true;
    }
    const bool crosses_y = (first.y() > point.y()) != (second.y() > point.y());
    if (crosses_y) {
      const float crossing_x =
        (second.x() - first.x()) * (point.y() - first.y()) /
        (second.y() - first.y()) + first.x();
      if (point.x() < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

StairRiserSemanticResult reject(StairRiserSemanticReason reason)
{
  StairRiserSemanticResult result;
  result.reason = reason;
  return result;
}

struct GeometryMatch
{
  bool matched{false};
  std::int32_t step_index{-1};
};

GeometryMatch matchRiserGeometry(
  const Eigen::Vector3f & point,
  const StaircaseModel & staircase,
  const StairRiserSemanticsConfig & config)
{
  if (!point.allFinite() || !pointInPolygon(
      point.head<2>(), staircase.corridor_polygon_xy))
  {
    return {};
  }

  Eigen::Vector2f axis = staircase.up_axis.head<2>();
  if (!axis.allFinite() || axis.norm() <= kGeometryEpsilon) {
    return {};
  }
  axis.normalize();
  const Eigen::Vector2f lateral(-axis.y(), axis.x());

  float best_plane_distance = std::numeric_limits<float>::infinity();
  std::int32_t best_step = -1;
  for (std::int32_t step = 0; step < staircase.step_count; ++step) {
    const Eigen::Vector3f expected_center = staircase.first_riser_center +
      Eigen::Vector3f(
      axis.x() * static_cast<float>(step) * staircase.tread_depth_m,
      axis.y() * static_cast<float>(step) * staircase.tread_depth_m,
      static_cast<float>(step) * staircase.riser_height_m);
    const Eigen::Vector2f horizontal_delta = point.head<2>() - expected_center.head<2>();
    const float plane_distance = std::abs(horizontal_delta.dot(axis));
    const float lateral_distance = std::abs(horizontal_delta.dot(lateral));
    const float vertical_distance = std::abs(point.z() - expected_center.z());
    if (plane_distance <= config.riser_plane_tolerance_m &&
      lateral_distance <= 0.5F * staircase.width_m +
      config.riser_lateral_tolerance_m &&
      vertical_distance <= 0.5F * staircase.riser_height_m +
      config.riser_vertical_tolerance_m &&
      plane_distance < best_plane_distance)
    {
      best_plane_distance = plane_distance;
      best_step = step;
    }
  }
  return {best_step >= 0, best_step};
}

std::optional<std::int32_t> supportAnchorLocation(
  const TerrainNode & node,
  const StaircaseModel & staircase)
{
  if (node.terrain_class == TerrainClass::STAIR_TREAD) {
    if (node.step_index < 0 || node.step_index >= staircase.step_count) {
      return std::nullopt;
    }
    return node.step_index;
  }
  if ((node.terrain_class != TerrainClass::FLAT &&
    node.terrain_class != TerrainClass::RAMP) ||
    (node.flags & TERRAIN_NODE_LANDING) == 0U)
  {
    return std::nullopt;
  }
  const bool lower = (node.flags & TERRAIN_NODE_LOWER_LANDING) != 0U;
  const bool upper = (node.flags & TERRAIN_NODE_UPPER_LANDING) != 0U;
  if (lower == upper) {
    return std::nullopt;
  }
  return lower ? -1 : staircase.step_count;
}

bool supportAnchorInsideSurvey(
  const TerrainNode & node,
  const Eigen::Vector2f & position,
  const StaircaseModel & staircase)
{
  if (node.terrain_class == TerrainClass::STAIR_TREAD) {
    return pointInPolygon(position, staircase.corridor_polygon_xy);
  }
  if ((node.flags & TERRAIN_NODE_LOWER_LANDING) != 0U) {
    return pointInPolygon(position, staircase.lower_landing_polygon_xy);
  }
  if ((node.flags & TERRAIN_NODE_UPPER_LANDING) != 0U) {
    return pointInPolygon(position, staircase.upper_landing_polygon_xy);
  }
  return false;
}

}  // namespace

const char * toString(StairRiserSemanticReason reason) noexcept
{
  switch (reason) {
    case StairRiserSemanticReason::EXPECTED_RISER: return "EXPECTED_RISER";
    case StairRiserSemanticReason::FEATURE_DISABLED: return "FEATURE_DISABLED";
    case StairRiserSemanticReason::INVALID_CONFIGURATION: return "INVALID_CONFIGURATION";
    case StairRiserSemanticReason::MISSING_SNAPSHOT: return "MISSING_SNAPSHOT";
    case StairRiserSemanticReason::INVALID_SNAPSHOT: return "INVALID_SNAPSHOT";
    case StairRiserSemanticReason::STALE_SNAPSHOT: return "STALE_SNAPSHOT";
    case StairRiserSemanticReason::MAP_MISMATCH: return "MAP_MISMATCH";
    case StairRiserSemanticReason::SNAPSHOT_VERSION_MISMATCH:
      return "SNAPSHOT_VERSION_MISMATCH";
    case StairRiserSemanticReason::INDEX_OUT_OF_RANGE: return "INDEX_OUT_OF_RANGE";
    case StairRiserSemanticReason::NODE_DISTANCE: return "NODE_DISTANCE";
    case StairRiserSemanticReason::NOT_STAIR_RISER: return "NOT_STAIR_RISER";
    case StairRiserSemanticReason::MISSING_STATIC_MAP_FLAG: return "MISSING_STATIC_MAP_FLAG";
    case StairRiserSemanticReason::OUTSIDE_MANUAL_CORRIDOR:
      return "OUTSIDE_MANUAL_CORRIDOR";
    case StairRiserSemanticReason::MISSING_ONLINE_CONFIRMATION:
      return "MISSING_ONLINE_CONFIRMATION";
    case StairRiserSemanticReason::STAIR_MODEL_MISSING: return "STAIR_MODEL_MISSING";
    case StairRiserSemanticReason::LOW_CONFIDENCE: return "LOW_CONFIDENCE";
    case StairRiserSemanticReason::DYNAMIC_OBSTACLE: return "DYNAMIC_OBSTACLE";
    case StairRiserSemanticReason::RISER_GEOMETRY_MISMATCH:
      return "RISER_GEOMETRY_MISMATCH";
    case StairRiserSemanticReason::STEP_INDEX_MISMATCH: return "STEP_INDEX_MISMATCH";
    case StairRiserSemanticReason::UNSUPPORTED_ANCHOR: return "UNSUPPORTED_ANCHOR";
    case StairRiserSemanticReason::ANCHOR_NOT_ADJACENT: return "ANCHOR_NOT_ADJACENT";
  }
  return "INVALID_STAIR_RISER_SEMANTIC_REASON";
}

bool StairRiserSemantics::validConfig(
  const StairRiserSemanticsConfig & config,
  std::string * error)
{
  if (!config.enabled) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  if (!config.fail_closed) {
    return setError(error, "enabled stair-riser semantics must be fail closed");
  }
  if (config.expected_map_hash.empty()) {
    return setError(error, "expected map hash is empty");
  }
  if (config.max_snapshot_age_nanoseconds <= 0) {
    return setError(error, "maximum snapshot age must be positive");
  }
  if (!std::isfinite(config.minimum_stair_confidence) ||
    config.minimum_stair_confidence <= 0.0F || config.minimum_stair_confidence > 1.0F)
  {
    return setError(error, "minimum stair confidence must be in (0, 1]");
  }
  if (!std::isfinite(config.max_node_match_distance_m) ||
    config.max_node_match_distance_m <= 0.0F ||
    !std::isfinite(config.riser_plane_tolerance_m) ||
    config.riser_plane_tolerance_m <= 0.0F ||
    !std::isfinite(config.riser_lateral_tolerance_m) ||
    config.riser_lateral_tolerance_m < 0.0F ||
    !std::isfinite(config.riser_vertical_tolerance_m) ||
    config.riser_vertical_tolerance_m <= 0.0F)
  {
    return setError(error, "stair-riser matching tolerances are invalid or unverified");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool stairRiserSupportLocationIsAdjacent(
  std::int32_t riser_step_index,
  std::int32_t support_location,
  std::int32_t step_count) noexcept
{
  if (step_count <= 0 || riser_step_index < 0 ||
    riser_step_index >= step_count || support_location < -1 ||
    support_location > step_count)
  {
    return false;
  }
  return support_location == riser_step_index - 1 ||
         support_location == riser_step_index ||
         (riser_step_index == step_count - 1 &&
         support_location == step_count);
}

StairRiserSemanticResult StairRiserSemantics::classify(
  const StairRiserSemanticsConfig & config,
  const StairRiserObservation & observation)
{
  if (!config.enabled) {
    return reject(StairRiserSemanticReason::FEATURE_DISABLED);
  }
  if (!validConfig(config)) {
    return reject(StairRiserSemanticReason::INVALID_CONFIGURATION);
  }
  if (!observation.snapshot) {
    return reject(StairRiserSemanticReason::MISSING_SNAPSHOT);
  }
  if (!observation.snapshot->valid()) {
    return reject(StairRiserSemanticReason::INVALID_SNAPSHOT);
  }
  if (observation.now_nanoseconds < observation.snapshot->stampNanoseconds() ||
    observation.now_nanoseconds - observation.snapshot->stampNanoseconds() >
    config.max_snapshot_age_nanoseconds)
  {
    return reject(StairRiserSemanticReason::STALE_SNAPSHOT);
  }
  if (observation.snapshot->mapHash() != config.expected_map_hash) {
    return reject(StairRiserSemanticReason::MAP_MISMATCH);
  }
  if (observation.terrain_ground_version == 0U ||
    observation.terrain_ground_version != observation.snapshot->version())
  {
    return reject(StairRiserSemanticReason::SNAPSHOT_VERSION_MISMATCH);
  }
  const TerrainNode * node = observation.snapshot->nodeAt(observation.terrain_node_index);
  if (node == nullptr || node->ground_index != observation.terrain_node_index) {
    return reject(StairRiserSemanticReason::INDEX_OUT_OF_RANGE);
  }
  if (!observation.terrain_node_position.allFinite() ||
    !observation.obstacle_position.allFinite() ||
    (observation.obstacle_position - observation.terrain_node_position).norm() >
    config.max_node_match_distance_m)
  {
    return reject(StairRiserSemanticReason::NODE_DISTANCE);
  }
  if ((node->flags & TERRAIN_NODE_STATIC_MAP) == 0U) {
    return reject(StairRiserSemanticReason::MISSING_STATIC_MAP_FLAG);
  }
  if ((node->flags & TERRAIN_NODE_MANUAL_CORRIDOR) == 0U) {
    return reject(StairRiserSemanticReason::OUTSIDE_MANUAL_CORRIDOR);
  }
  if ((node->flags & TERRAIN_NODE_ONLINE_CONFIRMED) == 0U) {
    return reject(StairRiserSemanticReason::MISSING_ONLINE_CONFIRMATION);
  }
  if (observation.dynamic_obstacle_confirmed) {
    return reject(StairRiserSemanticReason::DYNAMIC_OBSTACLE);
  }
  const StaircaseModel * staircase = observation.snapshot->staircaseById(node->staircase_id);
  if (staircase == nullptr) {
    return reject(StairRiserSemanticReason::STAIR_MODEL_MISSING);
  }
  if (staircase->confidence < config.minimum_stair_confidence) {
    return reject(StairRiserSemanticReason::LOW_CONFIDENCE);
  }
  if (!pointInPolygon(
      observation.obstacle_position.head<2>(), staircase->corridor_polygon_xy))
  {
    return reject(StairRiserSemanticReason::OUTSIDE_MANUAL_CORRIDOR);
  }
  const GeometryMatch obstacle_match = matchRiserGeometry(
    observation.obstacle_position, *staircase, config);
  if (!obstacle_match.matched) {
    return reject(StairRiserSemanticReason::RISER_GEOMETRY_MISMATCH);
  }

  if (node->terrain_class == TerrainClass::STAIR_RISER) {
    if (!pointInPolygon(
        observation.terrain_node_position.head<2>(), staircase->corridor_polygon_xy))
    {
      return reject(StairRiserSemanticReason::OUTSIDE_MANUAL_CORRIDOR);
    }
    const GeometryMatch node_match = matchRiserGeometry(
      observation.terrain_node_position, *staircase, config);
    if (!node_match.matched) {
      return reject(StairRiserSemanticReason::RISER_GEOMETRY_MISMATCH);
    }
    if (obstacle_match.step_index != node_match.step_index ||
      obstacle_match.step_index != node->step_index)
    {
      return reject(StairRiserSemanticReason::STEP_INDEX_MISMATCH);
    }
  } else {
    const auto anchor_location = supportAnchorLocation(*node, *staircase);
    if (!anchor_location.has_value() || !supportAnchorInsideSurvey(
        *node, observation.terrain_node_position.head<2>(), *staircase))
    {
      return reject(StairRiserSemanticReason::UNSUPPORTED_ANCHOR);
    }
    if (!stairRiserSupportLocationIsAdjacent(
        obstacle_match.step_index, *anchor_location, staircase->step_count))
    {
      return reject(StairRiserSemanticReason::ANCHOR_NOT_ADJACENT);
    }
  }

  StairRiserSemanticResult result;
  result.expected_riser = true;
  result.reason = StairRiserSemanticReason::EXPECTED_RISER;
  result.staircase_id = staircase->id;
  result.step_index = obstacle_match.step_index;
  return result;
}

bool terrainSurfaceProjectionCompatible(
  const TerrainNode & reference_node,
  const Eigen::Vector3f & reference_position,
  const TerrainNode & candidate_node,
  const Eigen::Vector3f & candidate_position,
  float plane_distance_tolerance_m) noexcept
{
  if (!reference_position.allFinite() || !candidate_position.allFinite() ||
    !reference_node.normal.allFinite() ||
    reference_node.normal.norm() <= kGeometryEpsilon ||
    reference_node.surface_id < 0 ||
    candidate_node.surface_id != reference_node.surface_id ||
    !std::isfinite(plane_distance_tolerance_m) || plane_distance_tolerance_m < 0.0F)
  {
    return false;
  }
  const Eigen::Vector3f normal = reference_node.normal.normalized();
  return std::abs((candidate_position - reference_position).dot(normal)) <=
         plane_distance_tolerance_m;
}

}  // namespace perception_3d
