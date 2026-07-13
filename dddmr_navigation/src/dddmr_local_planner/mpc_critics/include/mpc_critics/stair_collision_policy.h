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
#ifndef MPC_CRITICS_STAIR_COLLISION_POLICY_H_
#define MPC_CRITICS_STAIR_COLLISION_POLICY_H_

#include <perception_3d/stair_riser_semantics.h>

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace mpc_critics
{

enum class StairCollisionReason : std::uint8_t
{
  PASSTHROUGH_EXPECTED_RISER = 0,
  RISER_SEMANTICS_REJECTED,
  INVALID_LEG_ENVELOPE,
  INVALID_BODY_FRAME,
  MISSING_ACTIVE_STAIR_TRANSITION,
  NOT_ADJACENT_TRANSITION,
  OUTSIDE_LEG_ENVELOPE
};

const char * toString(StairCollisionReason reason) noexcept;

struct StairCollisionPolicyConfig
{
  perception_3d::StairRiserSemanticsConfig riser_semantics;
  // Bounds are expressed in the trajectory pose's cuboid-axis frame.  The
  // upper z bound must not exceed zero, preventing a misconfiguration from
  // turning the body volume into a riser pass-through region.
  Eigen::Vector3f leg_envelope_min{Eigen::Vector3f::Zero()};
  Eigen::Vector3f leg_envelope_max{Eigen::Vector3f::Zero()};
  float max_support_xy_distance_m{0.0F};
  float min_body_clearance_m{0.0F};
  float max_body_clearance_m{0.0F};
};

struct StairCollisionQuery
{
  perception_3d::StairRiserObservation riser_observation;
  Eigen::Vector3f trajectory_origin_world{Eigen::Vector3f::Zero()};
  // Unit, mutually perpendicular cuboid axes stored as columns.
  Eigen::Matrix3f body_axes_world{Eigen::Matrix3f::Identity()};
  // Location uses the same ordering as the terrain trajectory projector:
  // lower landing=-1, tread=step_index, upper landing=step_count.
  std::int32_t active_support_staircase_id{-1};
  std::int32_t active_support_location{-2};
};

struct StairCollisionResult
{
  bool passthrough{false};
  StairCollisionReason reason{StairCollisionReason::RISER_SEMANTICS_REJECTED};
  perception_3d::StairRiserSemanticReason riser_reason{
    perception_3d::StairRiserSemanticReason::FEATURE_DISABLED};
  std::int32_t staircase_id{-1};
  std::int32_t step_index{-1};
};

class StairCollisionPolicy final
{
public:
  static bool validConfig(
    const StairCollisionPolicyConfig & config,
    std::string * error = nullptr);

  static StairCollisionResult evaluate(
    const StairCollisionPolicyConfig & config,
    const StairCollisionQuery & query);
};

}  // namespace mpc_critics

#endif  // MPC_CRITICS_STAIR_COLLISION_POLICY_H_
