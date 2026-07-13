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

#include "perception_3d/terrain_model.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace perception_3d
{
namespace
{

constexpr float kNormalEpsilon = 1.0e-6F;
constexpr float kHalfPi = 1.57079632679489661923F;

bool finiteRatio(float value)
{
  return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

bool setError(std::string * error, const std::string & value)
{
  if (error != nullptr) {
    *error = value;
  }
  return false;
}

bool validPolygon(const std::vector<Eigen::Vector2f> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }
  double doubled_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & point = polygon[index];
    const auto & next = polygon[(index + 1U) % polygon.size()];
    if (!point.allFinite()) {
      return false;
    }
    doubled_area += static_cast<double>(point.x()) * static_cast<double>(next.y()) -
      static_cast<double>(next.x()) * static_cast<double>(point.y());
  }
  return std::abs(doubled_area) > 1.0e-8;
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
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1U;
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

}  // namespace

const char * toString(TerrainClass terrain_class) noexcept
{
  switch (terrain_class) {
    case TerrainClass::UNKNOWN: return "UNKNOWN";
    case TerrainClass::FLAT: return "FLAT";
    case TerrainClass::RAMP: return "RAMP";
    case TerrainClass::STAIR_TREAD: return "STAIR_TREAD";
    case TerrainClass::STAIR_RISER: return "STAIR_RISER";
    case TerrainClass::EDGE: return "EDGE";
    case TerrainClass::DROP: return "DROP";
  }
  return "UNKNOWN";
}

const char * toString(TerrainRejectionReason reason) noexcept
{
  switch (reason) {
    case TerrainRejectionReason::NONE: return "NONE";
    case TerrainRejectionReason::TERRAIN_DISABLED: return "TERRAIN_DISABLED";
    case TerrainRejectionReason::FAIL_OPEN_CONFIGURATION: return "FAIL_OPEN_CONFIGURATION";
    case TerrainRejectionReason::MISSING_SNAPSHOT: return "MISSING_SNAPSHOT";
    case TerrainRejectionReason::INVALID_SNAPSHOT: return "INVALID_SNAPSHOT";
    case TerrainRejectionReason::STALE: return "STALE";
    case TerrainRejectionReason::MAP_MISMATCH: return "MAP_MISMATCH";
    case TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH: return "SNAPSHOT_VERSION_MISMATCH";
    case TerrainRejectionReason::INDEX_OUT_OF_RANGE: return "INDEX_OUT_OF_RANGE";
    case TerrainRejectionReason::INVALID_GEOMETRY: return "INVALID_GEOMETRY";
    case TerrainRejectionReason::INVALID_CAPABILITY: return "INVALID_CAPABILITY";
    case TerrainRejectionReason::UNKNOWN: return "UNKNOWN";
    case TerrainRejectionReason::SLOPE: return "SLOPE";
    case TerrainRejectionReason::CROSS_SLOPE: return "CROSS_SLOPE";
    case TerrainRejectionReason::STEP_UP: return "STEP_UP";
    case TerrainRejectionReason::STEP_DOWN: return "STEP_DOWN";
    case TerrainRejectionReason::DROP: return "DROP";
    case TerrainRejectionReason::NO_SUPPORT: return "NO_SUPPORT";
    case TerrainRejectionReason::LAYER_MISMATCH: return "LAYER_MISMATCH";
    case TerrainRejectionReason::SKIP_STEP: return "SKIP_STEP";
    case TerrainRejectionReason::OUTSIDE_CORRIDOR: return "OUTSIDE_CORRIDOR";
    case TerrainRejectionReason::LOW_CONFIDENCE: return "LOW_CONFIDENCE";
    case TerrainRejectionReason::BAD_ALIGNMENT: return "BAD_ALIGNMENT";
    case TerrainRejectionReason::ROUGHNESS: return "ROUGHNESS";
    case TerrainRejectionReason::NORMAL_CHANGE: return "NORMAL_CHANGE";
    case TerrainRejectionReason::STAIR_DISABLED: return "STAIR_DISABLED";
    case TerrainRejectionReason::STAIR_MODEL_MISSING: return "STAIR_MODEL_MISSING";
    case TerrainRejectionReason::STAIR_GEOMETRY: return "STAIR_GEOMETRY";
    case TerrainRejectionReason::DIRECTION_DISABLED: return "DIRECTION_DISABLED";
    case TerrainRejectionReason::DYNAMIC_OBSTACLE: return "DYNAMIC_OBSTACLE";
    case TerrainRejectionReason::NO_LANDING: return "NO_LANDING";
  }
  return "INVALID_REJECTION_REASON";
}

TerrainSnapshot::TerrainSnapshot(
  std::string map_hash,
  std::uint64_t version,
  std::int64_t stamp_nanoseconds,
  std::vector<TerrainNode> nodes,
  std::vector<StaircaseModel> staircases)
: map_hash_(std::move(map_hash)),
  version_(version),
  stamp_nanoseconds_(stamp_nanoseconds),
  nodes_(std::move(nodes)),
  staircases_(std::move(staircases))
{
  valid_ = validateUncached(&validation_error_);
}

const TerrainNode * TerrainSnapshot::nodeAt(std::size_t index) const noexcept
{
  return index < nodes_.size() ? &nodes_[index] : nullptr;
}

const StaircaseModel * TerrainSnapshot::staircaseById(std::int32_t id) const noexcept
{
  for (const auto & staircase : staircases_) {
    if (staircase.id == id) {
      return &staircase;
    }
  }
  return nullptr;
}

bool TerrainSnapshot::valid(std::string * error) const
{
  if (error != nullptr) {
    *error = validation_error_;
  }
  return valid_;
}

bool TerrainSnapshot::validateUncached(std::string * error) const
{
  if (map_hash_.empty()) {
    return setError(error, "map hash is empty");
  }
  if (version_ == 0U) {
    return setError(error, "snapshot version is zero");
  }
  if (stamp_nanoseconds_ < 0) {
    return setError(error, "snapshot timestamp is negative");
  }
  if (nodes_.empty()) {
    return setError(error, "snapshot contains no terrain nodes");
  }

  for (std::size_t index = 0; index < nodes_.size(); ++index) {
    const auto & node = nodes_[index];
    if (node.ground_index != index) {
      std::ostringstream message;
      message << "terrain node " << index << " does not match mapground ordering";
      return setError(error, message.str());
    }
    if (!node.normal.allFinite() || node.normal.norm() <= kNormalEpsilon) {
      return setError(error, "terrain node has a non-finite or zero normal");
    }
    if (!std::isfinite(node.slope_rad) || node.slope_rad < 0.0F || node.slope_rad > kHalfPi) {
      return setError(error, "terrain node slope is outside [0, pi/2]");
    }
    if (!std::isfinite(node.roughness_m) || node.roughness_m < 0.0F) {
      return setError(error, "terrain node roughness is invalid");
    }
    if (!finiteRatio(node.support_ratio) || !finiteRatio(node.confidence)) {
      return setError(error, "terrain node ratio is outside [0, 1]");
    }
    if (node.terrain_class != TerrainClass::UNKNOWN && node.surface_id < 0) {
      return setError(error, "classified terrain node has no surface id");
    }
    if (
      (node.terrain_class == TerrainClass::STAIR_TREAD ||
      node.terrain_class == TerrainClass::STAIR_RISER) &&
      (node.staircase_id < 0 || node.step_index < 0))
    {
      return setError(error, "stair terrain node has no staircase or step id");
    }
  }

  std::unordered_set<std::int32_t> staircase_ids;
  for (const auto & staircase : staircases_) {
    if (staircase.id < 0 || !staircase_ids.insert(staircase.id).second) {
      return setError(error, "staircase ids must be non-negative and unique");
    }
    if (staircase.map_hash != map_hash_) {
      return setError(error, "staircase map hash does not match snapshot");
    }
    Eigen::Vector3f horizontal_axis = staircase.up_axis;
    horizontal_axis.z() = 0.0F;
    if (!staircase.up_axis.allFinite() || horizontal_axis.norm() <= kNormalEpsilon) {
      return setError(error, "staircase up axis has no finite horizontal direction");
    }
    if (!staircase.lower_landing_center.allFinite() ||
      !staircase.upper_landing_center.allFinite() ||
      staircase.upper_landing_center.z() <= staircase.lower_landing_center.z())
    {
      return setError(error, "staircase landing centers are invalid or not ordered by height");
    }
    const Eigen::Vector2f landing_direction =
      staircase.upper_landing_center.head<2>() -
      staircase.lower_landing_center.head<2>();
    if (landing_direction.norm() <= kNormalEpsilon ||
      landing_direction.dot(horizontal_axis.head<2>()) <= 0.0F)
    {
      return setError(error, "staircase landing centers disagree with the up axis");
    }
    if (!validPolygon(staircase.corridor_polygon_xy) ||
      !validPolygon(staircase.lower_landing_polygon_xy) ||
      !validPolygon(staircase.upper_landing_polygon_xy))
    {
      return setError(error, "staircase corridor or landing polygon is invalid");
    }
    if (!pointInPolygon(
        staircase.lower_landing_center.head<2>(),
        staircase.lower_landing_polygon_xy) ||
      !pointInPolygon(
        staircase.upper_landing_center.head<2>(),
        staircase.upper_landing_polygon_xy))
    {
      return setError(error, "staircase landing center is outside its landing polygon");
    }
    if (
      !std::isfinite(staircase.width_m) || staircase.width_m <= 0.0F ||
      !std::isfinite(staircase.riser_height_m) || staircase.riser_height_m <= 0.0F ||
      !std::isfinite(staircase.tread_depth_m) || staircase.tread_depth_m <= 0.0F ||
      staircase.step_count <= 0 || !finiteRatio(staircase.confidence))
    {
      return setError(error, "staircase geometry or confidence is invalid");
    }
    if (!staircase.first_riser_center.allFinite()) {
      return setError(error, "staircase first riser center is missing or non-finite");
    }
    const float expected_first_riser_z =
      staircase.lower_landing_center.z() + 0.5F * staircase.riser_height_m;
    const float first_riser_height_tolerance =
      std::max(0.02F, 0.10F * staircase.riser_height_m);
    if (std::abs(staircase.first_riser_center.z() - expected_first_riser_z) >
      first_riser_height_tolerance)
    {
      return setError(
        error, "staircase first riser center z is not lower landing z + half a riser");
    }
    if (!pointInPolygon(
        staircase.first_riser_center.head<2>(), staircase.corridor_polygon_xy))
    {
      return setError(error, "staircase first riser center is outside the corridor");
    }
    const Eigen::Vector2f horizontal_axis_unit = horizontal_axis.head<2>().normalized();
    const float first_from_lower =
      (staircase.first_riser_center.head<2>() -
      staircase.lower_landing_center.head<2>()).dot(horizontal_axis_unit);
    const float upper_from_first =
      (staircase.upper_landing_center.head<2>() -
      staircase.first_riser_center.head<2>()).dot(horizontal_axis_unit);
    if (first_from_lower <= kNormalEpsilon || upper_from_first <= kNormalEpsilon) {
      return setError(
        error, "staircase first riser center is not between the landings along the up axis");
    }
    for (std::int32_t step = 0; step < staircase.step_count; ++step) {
      const Eigen::Vector2f riser_xy = staircase.first_riser_center.head<2>() +
        horizontal_axis_unit * static_cast<float>(step) * staircase.tread_depth_m;
      if (!pointInPolygon(riser_xy, staircase.corridor_polygon_xy)) {
        return setError(error, "staircase expected riser center is outside the corridor");
      }
      if ((staircase.upper_landing_center.head<2>() - riser_xy).dot(horizontal_axis_unit) <=
        kNormalEpsilon)
      {
        return setError(error, "staircase expected riser extends beyond the upper landing");
      }
    }
    // step_count is the number of risers. The highest annotated tread is
    // therefore at lower_landing_z + step_count * mean_riser_height and is
    // level with the upper landing. Reject an internally inconsistent survey
    // before planners can create a disconnected or fictitious top step.
    const float measured_total_rise =
      staircase.upper_landing_center.z() - staircase.lower_landing_center.z();
    const float modeled_total_rise =
      static_cast<float>(staircase.step_count) * staircase.riser_height_m;
    const float total_rise_tolerance = std::max(0.02F, 0.10F * staircase.riser_height_m);
    if (std::abs(measured_total_rise - modeled_total_rise) > total_rise_tolerance) {
      return setError(error, "staircase landing rise disagrees with step_count * riser_height");
    }
  }

  for (const auto & node : nodes_) {
    if (
      (node.terrain_class == TerrainClass::STAIR_TREAD ||
      node.terrain_class == TerrainClass::STAIR_RISER) &&
      staircaseById(node.staircase_id) == nullptr)
    {
      return setError(error, "stair terrain node references a missing staircase model");
    }
  }

  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace perception_3d
