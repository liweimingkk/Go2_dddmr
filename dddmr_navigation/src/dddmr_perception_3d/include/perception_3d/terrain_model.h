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
#ifndef PERCEPTION_3D_TERRAIN_MODEL_H_
#define PERCEPTION_3D_TERRAIN_MODEL_H_

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace perception_3d
{

enum class TerrainClass : std::uint8_t
{
  UNKNOWN = 0,
  FLAT,
  RAMP,
  STAIR_TREAD,
  STAIR_RISER,
  EDGE,
  DROP
};

// Shared across terrain producers, planners, critics, and command gates.  Keep
// these values stable so diagnostics do not depend on free-form log strings.
enum class TerrainRejectionReason : std::uint8_t
{
  NONE = 0,
  TERRAIN_DISABLED,
  FAIL_OPEN_CONFIGURATION,
  MISSING_SNAPSHOT,
  INVALID_SNAPSHOT,
  STALE,
  MAP_MISMATCH,
  SNAPSHOT_VERSION_MISMATCH,
  INDEX_OUT_OF_RANGE,
  INVALID_GEOMETRY,
  INVALID_CAPABILITY,
  UNKNOWN,
  SLOPE,
  CROSS_SLOPE,
  STEP_UP,
  STEP_DOWN,
  DROP,
  NO_SUPPORT,
  LAYER_MISMATCH,
  SKIP_STEP,
  OUTSIDE_CORRIDOR,
  LOW_CONFIDENCE,
  BAD_ALIGNMENT,
  ROUGHNESS,
  NORMAL_CHANGE,
  STAIR_DISABLED,
  STAIR_MODEL_MISSING,
  STAIR_GEOMETRY,
  DIRECTION_DISABLED,
  DYNAMIC_OBSTACLE,
  NO_LANDING
};

const char * toString(TerrainClass terrain_class) noexcept;
const char * toString(TerrainRejectionReason reason) noexcept;

enum TerrainNodeFlag : std::uint32_t
{
  TERRAIN_NODE_NONE = 0U,
  TERRAIN_NODE_OBSERVED = 1U << 0,
  TERRAIN_NODE_STATIC_MAP = 1U << 1,
  TERRAIN_NODE_ONLINE_CONFIRMED = 1U << 2,
  TERRAIN_NODE_MANUAL_CORRIDOR = 1U << 3,
  TERRAIN_NODE_LANDING = 1U << 4,
  TERRAIN_NODE_LOWER_LANDING = 1U << 5,
  TERRAIN_NODE_UPPER_LANDING = 1U << 6
};

struct TerrainNode
{
  std::uint32_t ground_index{0U};
  Eigen::Vector3f normal{Eigen::Vector3f::UnitZ()};
  float slope_rad{0.0F};
  float roughness_m{0.0F};
  float support_ratio{0.0F};
  float confidence{0.0F};
  std::int32_t surface_id{-1};
  std::int32_t staircase_id{-1};
  std::int32_t step_index{-1};
  TerrainClass terrain_class{TerrainClass::UNKNOWN};
  std::uint32_t flags{TERRAIN_NODE_NONE};
};

struct StaircaseModel
{
  std::int32_t id{-1};
  std::string map_hash;
  Eigen::Vector3f up_axis{Eigen::Vector3f::UnitX()};
  Eigen::Vector3f lower_landing_center{Eigen::Vector3f::Zero()};
  Eigen::Vector3f upper_landing_center{Eigen::Vector3f::Zero()};
  // Surveyed center of the first vertical riser face.  This anchor is
  // deliberately not inferred from a corridor bounding box: irregular
  // landings and rotated polygons make that inference unsafe.  A staircase
  // model without a finite, geometrically consistent anchor is invalid.
  Eigen::Vector3f first_riser_center{
    Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())};
  std::vector<Eigen::Vector2f> corridor_polygon_xy;
  std::vector<Eigen::Vector2f> lower_landing_polygon_xy;
  std::vector<Eigen::Vector2f> upper_landing_polygon_xy;
  float width_m{0.0F};
  float riser_height_m{0.0F};
  float tread_depth_m{0.0F};
  std::int32_t step_count{0};
  float confidence{0.0F};
  bool allow_up{false};
  bool allow_down{false};
};

// Construct a complete snapshot and then publish it through a
// shared_ptr<const TerrainSnapshot>.  The class intentionally exposes no
// mutation methods, so a planner can retain one version for an entire search.
class TerrainSnapshot final
{
public:
  TerrainSnapshot(
    std::string map_hash,
    std::uint64_t version,
    std::int64_t stamp_nanoseconds,
    std::vector<TerrainNode> nodes,
    std::vector<StaircaseModel> staircases = {});

  const std::string & mapHash() const noexcept {return map_hash_;}
  std::uint64_t version() const noexcept {return version_;}
  std::int64_t stampNanoseconds() const noexcept {return stamp_nanoseconds_;}
  const std::vector<TerrainNode> & nodes() const noexcept {return nodes_;}
  const std::vector<StaircaseModel> & staircases() const noexcept {return staircases_;}

  const TerrainNode * nodeAt(std::size_t index) const noexcept;
  const StaircaseModel * staircaseById(std::int32_t id) const noexcept;

  // A valid snapshot has a non-empty map identity, a non-zero version, finite
  // bounded node metrics, strict mapground index ordering, and valid unique
  // staircase models.  UNKNOWN nodes are structurally valid but remain
  // non-traversable in TerrainEdgePolicy.
  bool valid(std::string * error = nullptr) const;

private:
  bool validateUncached(std::string * error) const;

  std::string map_hash_;
  std::uint64_t version_{0U};
  std::int64_t stamp_nanoseconds_{0};
  std::vector<TerrainNode> nodes_;
  std::vector<StaircaseModel> staircases_;
  bool valid_{false};
  std::string validation_error_;
};

using TerrainSnapshotConstPtr = std::shared_ptr<const TerrainSnapshot>;

}  // namespace perception_3d

#endif  // PERCEPTION_3D_TERRAIN_MODEL_H_
