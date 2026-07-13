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
#ifndef PERCEPTION_3D_TERRAIN_MODEL_BUILDER_H_
#define PERCEPTION_3D_TERRAIN_MODEL_BUILDER_H_

#include "perception_3d/terrain_model.h"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace perception_3d
{

// These thresholds describe geometric observation quality and segmentation;
// they are deliberately separate from TerrainEdgePolicyConfig robot capability
// limits.  A RAMP classification is never an assertion that the robot may use it.
struct TerrainModelBuilderConfig
{
  float normal_radius_m{0.30F};
  std::size_t min_normal_neighbors{8U};
  float max_plane_residual_m{0.04F};
  float flat_slope_threshold_rad{0.0872664626F};  // 5 degrees
  float max_model_slope_rad{1.396263402F};       // 80 degrees

  float support_radius_m{0.20F};
  float support_plane_tolerance_m{0.05F};
  std::size_t support_sector_count{8U};
  float min_observed_support_ratio{0.25F};
  float edge_support_ratio{0.75F};

  float surface_connectivity_radius_m{0.35F};
  float max_surface_height_delta_m{0.20F};
  float max_surface_normal_change_rad{0.3490658504F};  // 20 degrees
};

struct TerrainModelBuildInput
{
  std::string map_hash;
  std::uint64_t version{0U};
  std::int64_t stamp_nanoseconds{0};
  std::vector<Eigen::Vector3f> mapground_points;
  std::vector<Eigen::Vector3f> support_points;
};

struct TerrainModelBuildStatistics
{
  std::size_t flat_count{0U};
  std::size_t ramp_count{0U};
  std::size_t edge_count{0U};
  std::size_t unknown_count{0U};
  std::size_t surface_count{0U};
  std::size_t invalid_ground_point_count{0U};
  std::size_t invalid_support_point_count{0U};
};

struct TerrainModelBuildResult
{
  TerrainSnapshotConstPtr snapshot;
  TerrainModelBuildStatistics statistics;
  std::string error;

  bool ok() const noexcept {return snapshot != nullptr && error.empty();}
};

// Pure, deterministic, and free of ROS/wall-clock state.  Input point ordering
// is preserved exactly so TerrainNode::ground_index stays aligned with mapground.
class TerrainModelBuilder final
{
public:
  static TerrainModelBuildResult build(
    const TerrainModelBuildInput & input,
    const TerrainModelBuilderConfig & config = TerrainModelBuilderConfig{});
};

}  // namespace perception_3d

#endif  // PERCEPTION_3D_TERRAIN_MODEL_BUILDER_H_
