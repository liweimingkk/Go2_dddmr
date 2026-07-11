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

#include <mpc_critics/terrain_support_policy.h>

#include <cmath>
#include <cstdlib>

namespace mpc_critics
{
namespace
{

bool isStairSupport(const perception_3d::TerrainNode & node) noexcept
{
  return node.terrain_class == perception_3d::TerrainClass::STAIR_TREAD;
}

bool hasFlag(const perception_3d::TerrainNode & node, perception_3d::TerrainNodeFlag flag) noexcept
{
  return (node.flags & static_cast<std::uint32_t>(flag)) != 0U;
}

}  // namespace

bool terrainSupportLimitsValid(const TerrainSupportLimits & limits) noexcept
{
  return
    std::isfinite(limits.max_support_distance_m) && limits.max_support_distance_m > 0.0 &&
    std::isfinite(limits.max_z_gap_m) && limits.max_z_gap_m >= 0.0 &&
    std::isfinite(limits.min_support_ratio) && limits.min_support_ratio >= 0.0 &&
    limits.min_support_ratio <= 1.0 &&
    std::isfinite(limits.min_confidence) && limits.min_confidence >= 0.0 &&
    limits.min_confidence <= 1.0 &&
    limits.max_step_index_delta >= 0;
}

perception_3d::TerrainRejectionReason validateTerrainSupportData(
  const perception_3d::TerrainSnapshotConstPtr & snapshot,
  std::uint64_t ground_version,
  std::size_t ground_point_count,
  bool kdtree_ready)
{
  using perception_3d::TerrainRejectionReason;

  if (!snapshot) {
    return TerrainRejectionReason::MISSING_SNAPSHOT;
  }
  if (ground_version == 0U || snapshot->version() != ground_version) {
    return TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH;
  }
  if (!snapshot->valid()) {
    return TerrainRejectionReason::INVALID_SNAPSHOT;
  }
  if (ground_point_count == 0U || !kdtree_ready) {
    return TerrainRejectionReason::NO_SUPPORT;
  }
  if (snapshot->nodes().size() != ground_point_count) {
    // The only safe interpretation is that the indexed cloud belongs to a
    // different mapground generation, even if both objects are individually
    // valid.
    return TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH;
  }
  return TerrainRejectionReason::NONE;
}

perception_3d::TerrainRejectionReason evaluateTerrainSupport(
  const TerrainSupportObservation & current,
  const TerrainSupportObservation * previous,
  const TerrainSupportLimits & limits) noexcept
{
  using perception_3d::TerrainClass;
  using perception_3d::TerrainRejectionReason;

  if (!terrainSupportLimitsValid(limits)) {
    return TerrainRejectionReason::INVALID_CAPABILITY;
  }
  if (current.node == nullptr || current.node->ground_index != current.ground_index) {
    return TerrainRejectionReason::INDEX_OUT_OF_RANGE;
  }
  if (!std::isfinite(current.horizontal_distance_m) || current.horizontal_distance_m < 0.0 ||
    !std::isfinite(current.z_gap_m) || current.z_gap_m < 0.0)
  {
    return TerrainRejectionReason::INVALID_GEOMETRY;
  }
  if (current.horizontal_distance_m > limits.max_support_distance_m ||
    current.z_gap_m > limits.max_z_gap_m)
  {
    return TerrainRejectionReason::NO_SUPPORT;
  }

  switch (current.node->terrain_class) {
    case TerrainClass::UNKNOWN:
      return TerrainRejectionReason::UNKNOWN;
    case TerrainClass::EDGE:
      return TerrainRejectionReason::NO_SUPPORT;
    case TerrainClass::DROP:
      return TerrainRejectionReason::DROP;
    case TerrainClass::STAIR_RISER:
      return TerrainRejectionReason::NO_SUPPORT;
    case TerrainClass::FLAT:
    case TerrainClass::RAMP:
    case TerrainClass::STAIR_TREAD:
      break;
  }

  if (current.node->confidence < limits.min_confidence) {
    return TerrainRejectionReason::LOW_CONFIDENCE;
  }
  if (current.node->support_ratio < limits.min_support_ratio) {
    return TerrainRejectionReason::NO_SUPPORT;
  }
  if (current.node->surface_id < 0) {
    return TerrainRejectionReason::LAYER_MISMATCH;
  }
  if (limits.require_online_confirmation &&
    !hasFlag(*current.node, perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED))
  {
    return TerrainRejectionReason::LOW_CONFIDENCE;
  }

  const bool current_is_stair = isStairSupport(*current.node);
  if (current_is_stair) {
    if (!limits.stairs_enabled) {
      return TerrainRejectionReason::STAIR_DISABLED;
    }
    if (current.staircase == nullptr || current.node->staircase_id < 0 ||
      current.node->step_index < 0 || current.staircase->id != current.node->staircase_id)
    {
      return TerrainRejectionReason::STAIR_MODEL_MISSING;
    }
    if (current.staircase->confidence < limits.min_confidence) {
      return TerrainRejectionReason::LOW_CONFIDENCE;
    }
    if (limits.require_manual_corridor &&
      !hasFlag(*current.node, perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR))
    {
      return TerrainRejectionReason::OUTSIDE_CORRIDOR;
    }
  }

  if (previous == nullptr) {
    return TerrainRejectionReason::NONE;
  }
  if (previous->node == nullptr || previous->node->ground_index != previous->ground_index) {
    return TerrainRejectionReason::INDEX_OUT_OF_RANGE;
  }

  const bool previous_is_stair = isStairSupport(*previous->node);
  if (current_is_stair && previous_is_stair) {
    if (current.node->staircase_id != previous->node->staircase_id) {
      return TerrainRejectionReason::LAYER_MISMATCH;
    }
    const std::int64_t step_delta =
      static_cast<std::int64_t>(current.node->step_index) -
      static_cast<std::int64_t>(previous->node->step_index);
    if (std::llabs(step_delta) > limits.max_step_index_delta) {
      return TerrainRejectionReason::SKIP_STEP;
    }
    if (step_delta > 0 && !current.staircase->allow_up) {
      return TerrainRejectionReason::DIRECTION_DISABLED;
    }
    if (step_delta < 0 && !current.staircase->allow_down) {
      return TerrainRejectionReason::DIRECTION_DISABLED;
    }
    return TerrainRejectionReason::NONE;
  }

  if (current_is_stair != previous_is_stair) {
    const auto & landing = current_is_stair ? *previous->node : *current.node;
    if (!hasFlag(landing, perception_3d::TERRAIN_NODE_LANDING)) {
      return TerrainRejectionReason::NO_LANDING;
    }
    const auto & stair_observation = current_is_stair ? current : *previous;
    if (stair_observation.staircase == nullptr) {
      return TerrainRejectionReason::STAIR_MODEL_MISSING;
    }
    if (landing.staircase_id != stair_observation.staircase->id) {
      return TerrainRejectionReason::LAYER_MISMATCH;
    }
    if (limits.require_manual_corridor &&
      !hasFlag(landing, perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR))
    {
      return TerrainRejectionReason::OUTSIDE_CORRIDOR;
    }
    if (limits.require_online_confirmation &&
      !hasFlag(landing, perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED))
    {
      return TerrainRejectionReason::LOW_CONFIDENCE;
    }
    const bool lower_landing =
      hasFlag(landing, perception_3d::TERRAIN_NODE_LOWER_LANDING);
    const bool upper_landing =
      hasFlag(landing, perception_3d::TERRAIN_NODE_UPPER_LANDING);
    if (lower_landing == upper_landing) {
      return TerrainRejectionReason::NO_LANDING;
    }
    const std::int32_t stair_step = stair_observation.node->step_index;
    const std::int32_t last_step = stair_observation.staircase->step_count - 1;
    if ((lower_landing && stair_step != 0) ||
      (upper_landing && stair_step != last_step))
    {
      return TerrainRejectionReason::SKIP_STEP;
    }
    const bool ascending = lower_landing ? current_is_stair : !current_is_stair;
    if (ascending && !stair_observation.staircase->allow_up) {
      return TerrainRejectionReason::DIRECTION_DISABLED;
    }
    if (!ascending && !stair_observation.staircase->allow_down) {
      return TerrainRejectionReason::DIRECTION_DISABLED;
    }
    return TerrainRejectionReason::NONE;
  }

  if (current.node->surface_id != previous->node->surface_id) {
    return TerrainRejectionReason::LAYER_MISMATCH;
  }
  return TerrainRejectionReason::NONE;
}

}  // namespace mpc_critics
