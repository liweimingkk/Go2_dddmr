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
#ifndef MPC_CRITICS_TERRAIN_SUPPORT_POLICY_H_
#define MPC_CRITICS_TERRAIN_SUPPORT_POLICY_H_

#include <perception_3d/terrain_model.h>

#include <cstddef>
#include <cstdint>

namespace mpc_critics
{

struct TerrainSupportLimits
{
  double max_support_distance_m{0.30};
  double max_z_gap_m{0.12};
  double min_support_ratio{0.80};
  double min_confidence{0.90};
  std::int32_t max_step_index_delta{1};
  bool stairs_enabled{false};
  bool require_manual_corridor{true};
  bool require_online_confirmation{true};
};

// A KD-tree lookup is converted to this small value type before policy
// evaluation.  Keeping the policy independent from ROS and PCL makes the
// fail-closed rules deterministic and directly unit-testable.
struct TerrainSupportObservation
{
  std::size_t ground_index{0U};
  double horizontal_distance_m{0.0};
  double z_gap_m{0.0};
  const perception_3d::TerrainNode * node{nullptr};
  const perception_3d::StaircaseModel * staircase{nullptr};
};

bool terrainSupportLimitsValid(const TerrainSupportLimits & limits) noexcept;

perception_3d::TerrainRejectionReason validateTerrainSupportData(
  const perception_3d::TerrainSnapshotConstPtr & snapshot,
  std::uint64_t ground_version,
  std::size_t ground_point_count,
  bool kdtree_ready);

perception_3d::TerrainRejectionReason evaluateTerrainSupport(
  const TerrainSupportObservation & current,
  const TerrainSupportObservation * previous,
  const TerrainSupportLimits & limits) noexcept;

}  // namespace mpc_critics

#endif  // MPC_CRITICS_TERRAIN_SUPPORT_POLICY_H_
