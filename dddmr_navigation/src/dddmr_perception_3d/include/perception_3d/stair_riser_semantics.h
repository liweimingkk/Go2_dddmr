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
#ifndef PERCEPTION_3D_STAIR_RISER_SEMANTICS_H_
#define PERCEPTION_3D_STAIR_RISER_SEMANTICS_H_

#include "perception_3d/terrain_model.h"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>

namespace perception_3d
{

// This classifier only identifies a measured, static stair-riser return.  It
// does not decide whether the robot body may overlap that return; the collision
// critic applies a separate configured leg envelope after this result.
enum class StairRiserSemanticReason : std::uint8_t
{
  EXPECTED_RISER = 0,
  FEATURE_DISABLED,
  INVALID_CONFIGURATION,
  MISSING_SNAPSHOT,
  INVALID_SNAPSHOT,
  STALE_SNAPSHOT,
  MAP_MISMATCH,
  SNAPSHOT_VERSION_MISMATCH,
  INDEX_OUT_OF_RANGE,
  NODE_DISTANCE,
  NOT_STAIR_RISER,
  MISSING_STATIC_MAP_FLAG,
  OUTSIDE_MANUAL_CORRIDOR,
  MISSING_ONLINE_CONFIRMATION,
  STAIR_MODEL_MISSING,
  LOW_CONFIDENCE,
  DYNAMIC_OBSTACLE,
  RISER_GEOMETRY_MISMATCH,
  STEP_INDEX_MISMATCH,
  UNSUPPORTED_ANCHOR,
  ANCHOR_NOT_ADJACENT
};

const char * toString(StairRiserSemanticReason reason) noexcept;

struct StairRiserSemanticsConfig
{
  bool enabled{false};
  bool fail_closed{true};
  std::string expected_map_hash;
  std::int64_t max_snapshot_age_nanoseconds{0};
  float minimum_stair_confidence{0.0F};
  float max_node_match_distance_m{0.0F};
  float riser_plane_tolerance_m{0.0F};
  float riser_lateral_tolerance_m{0.0F};
  float riser_vertical_tolerance_m{0.0F};
};

struct StairRiserObservation
{
  TerrainSnapshotConstPtr snapshot;
  std::uint64_t terrain_ground_version{0U};
  std::int64_t now_nanoseconds{0};
  std::size_t terrain_node_index{0U};
  Eigen::Vector3f terrain_node_position{Eigen::Vector3f::Zero()};
  Eigen::Vector3f obstacle_position{Eigen::Vector3f::Zero()};
  // A temporal tracker or semantic source may assert this bit.  Dynamic
  // always wins over a coincident static stair model.
  bool dynamic_obstacle_confirmed{false};
};

struct StairRiserSemanticResult
{
  bool expected_riser{false};
  StairRiserSemanticReason reason{StairRiserSemanticReason::FEATURE_DISABLED};
  std::int32_t staircase_id{-1};
  std::int32_t step_index{-1};
};

class StairRiserSemantics final
{
public:
  static bool validConfig(
    const StairRiserSemanticsConfig & config,
    std::string * error = nullptr);

  static StairRiserSemanticResult classify(
    const StairRiserSemanticsConfig & config,
    const StairRiserObservation & observation);
};

// A riser normally borders support locations {step - 1, step}.  The highest
// tread and upper landing share one elevation, so the final riser also borders
// the distinct upper-landing location step_count.  Keeping this rule pure and
// shared prevents perception and collision checks from drifting apart.
bool stairRiserSupportLocationIsAdjacent(
  std::int32_t riser_step_index,
  std::int32_t support_location,
  std::int32_t step_count) noexcept;

// Cluster marking must not collapse overlapping floors or stair surfaces by
// XY distance alone.  This pure helper requires one terrain surface identity
// and consistency with the reference node's tangent plane.
bool terrainSurfaceProjectionCompatible(
  const TerrainNode & reference_node,
  const Eigen::Vector3f & reference_position,
  const TerrainNode & candidate_node,
  const Eigen::Vector3f & candidate_position,
  float plane_distance_tolerance_m) noexcept;

}  // namespace perception_3d

#endif  // PERCEPTION_3D_STAIR_RISER_SEMANTICS_H_
