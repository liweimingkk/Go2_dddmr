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
#ifndef PERCEPTION_3D_TERRAIN_EDGE_POLICY_H_
#define PERCEPTION_3D_TERRAIN_EDGE_POLICY_H_

#include "perception_3d/terrain_model.h"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace perception_3d
{

enum class TerrainEdgeMode : std::uint8_t
{
  UNKNOWN = 0,
  CONTINUOUS_SURFACE,
  GENERIC_STEP,
  STAIR_TRANSITION
};

// Samples describe robot support along the candidate edge, not obstacle or
// riser points.  edge_fraction must cover [0, 1] in strictly increasing order.
struct TerrainSupportSample
{
  float edge_fraction{0.0F};
  float support_ratio{0.0F};
  float unknown_ratio{1.0F};
  float confidence{0.0F};
  Eigen::Vector3f normal{Eigen::Vector3f::UnitZ()};
  float roughness_m{0.0F};
  TerrainClass terrain_class{TerrainClass::UNKNOWN};
  std::int32_t surface_id{-1};
  std::int32_t staircase_id{-1};
  std::int32_t step_index{-1};
  std::uint32_t flags{TERRAIN_NODE_NONE};
};

struct TerrainEdgeRequest
{
  std::uint32_t from_index{0U};
  std::uint32_t to_index{0U};
  Eigen::Vector3f from_position{Eigen::Vector3f::Zero()};
  Eigen::Vector3f to_position{Eigen::Vector3f::Zero()};
  TerrainEdgeMode mode{TerrainEdgeMode::UNKNOWN};
  std::string expected_map_hash;
  std::uint64_t expected_snapshot_version{0U};
  std::vector<TerrainSupportSample> support_samples;
  bool inside_manual_corridor{false};
  bool online_confirmation{false};
  bool dynamic_obstacle{false};
};

// This is an already validated robot/site capability envelope, not a sensor
// segmentation configuration.  Zero upper bounds are literal zero capability;
// they never mean unlimited.
struct TerrainEdgePolicyConfig
{
  bool enabled{false};
  bool fail_closed{true};

  float max_up_slope_rad{0.0F};
  float max_down_slope_rad{0.0F};
  float max_cross_slope_rad{0.0F};
  float max_roughness_m{0.0F};
  float max_normal_change_rad{0.0F};
  float max_step_up_m{0.0F};
  float max_step_down_m{0.0F};
  float min_support_ratio{1.0F};
  float max_unknown_ratio{0.0F};
  float min_confidence{1.0F};
  float max_support_sample_spacing_m{0.0F};

  bool stair_enabled{false};
  bool allow_stair_up{false};
  bool allow_stair_down{false};
  bool require_manual_corridor{true};
  bool require_online_confirmation{true};
  std::int32_t max_step_index_delta{0};
  float max_stair_riser_height_m{0.0F};
  float min_stair_tread_depth_m{0.0F};
  float max_stair_tread_depth_m{0.0F};
  float max_stair_riser_deviation_m{0.0F};
  float max_stair_tread_deviation_m{0.0F};
  float max_stair_heading_error_rad{0.0F};

  float distance_cost_weight{1.0F};
  float slope_cost_weight{1.0F};
  float cross_slope_cost_weight{1.0F};
  float roughness_cost_weight{1.0F};
  float risk_cost_weight{1.0F};
  float stair_transition_cost{0.0F};
};

struct TerrainEdgeResult
{
  bool accepted{false};
  TerrainRejectionReason reason{TerrainRejectionReason::INVALID_GEOMETRY};
  float traversal_cost{0.0F};
  float edge_length_m{0.0F};
  float signed_slope_rad{0.0F};
  float cross_slope_rad{0.0F};
  float normal_change_rad{0.0F};
  float maximum_abs_slope_rad{0.0F};
  float maximum_roughness_m{0.0F};
  float minimum_support_ratio{0.0F};
  float maximum_unknown_ratio{1.0F};
};

// Stateless and deterministic: identical snapshot, request, and capability
// inputs always produce the same result.  It has no ROS or wall-clock access.
class TerrainEdgePolicy final
{
public:
  static TerrainEdgeResult evaluate(
    const TerrainSnapshotConstPtr & snapshot,
    const TerrainEdgeRequest & request,
    const TerrainEdgePolicyConfig & config);
};

}  // namespace perception_3d

#endif  // PERCEPTION_3D_TERRAIN_EDGE_POLICY_H_
