#include <gtest/gtest.h>

#include <mpc_critics/stair_collision_policy.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace mpc_critics
{
namespace
{

using perception_3d::StaircaseModel;
using perception_3d::TerrainClass;
using perception_3d::TerrainNode;
using perception_3d::TerrainSnapshot;

constexpr std::int64_t kStamp = 1'000'000'000LL;

StaircaseModel staircase()
{
  StaircaseModel value;
  value.id = 2;
  value.map_hash = "map-a";
  value.up_axis = Eigen::Vector3f::UnitX();
  value.lower_landing_center = Eigen::Vector3f(-0.30F, 0.0F, 0.0F);
  value.upper_landing_center = Eigen::Vector3f(1.20F, 0.0F, 0.72F);
  value.first_riser_center = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  value.corridor_polygon_xy = {
    {-0.10F, -0.60F}, {1.00F, -0.60F}, {1.00F, 0.60F}, {-0.10F, 0.60F}};
  value.lower_landing_polygon_xy = {
    {-0.60F, -0.60F}, {0.0F, -0.60F}, {0.0F, 0.60F}, {-0.60F, 0.60F}};
  value.upper_landing_polygon_xy = {
    {0.90F, -0.60F}, {1.50F, -0.60F}, {1.50F, 0.60F}, {0.90F, 0.60F}};
  value.width_m = 1.20F;
  value.riser_height_m = 0.18F;
  value.tread_depth_m = 0.30F;
  value.step_count = 4;
  value.confidence = 0.98F;
  value.allow_up = true;
  value.allow_down = true;
  return value;
}

TerrainNode riserNode(std::int32_t step = 0)
{
  TerrainNode node;
  node.ground_index = 0U;
  node.normal = -Eigen::Vector3f::UnitX();
  node.slope_rad = 1.57079632679F;
  node.surface_id = 10;
  node.staircase_id = 2;
  node.step_index = step;
  node.terrain_class = TerrainClass::STAIR_RISER;
  node.flags = perception_3d::TERRAIN_NODE_STATIC_MAP |
    perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  return node;
}

perception_3d::TerrainSnapshotConstPtr snapshot(TerrainNode node = riserNode())
{
  return std::make_shared<const TerrainSnapshot>(
    "map-a", 3U, kStamp, std::vector<TerrainNode>{std::move(node)},
    std::vector<StaircaseModel>{staircase()});
}

StairCollisionPolicyConfig config()
{
  StairCollisionPolicyConfig value;
  value.riser_semantics.enabled = true;
  value.riser_semantics.fail_closed = true;
  value.riser_semantics.expected_map_hash = "map-a";
  value.riser_semantics.max_snapshot_age_nanoseconds = 200'000'000LL;
  value.riser_semantics.minimum_stair_confidence = 0.90F;
  value.riser_semantics.max_node_match_distance_m = 0.04F;
  value.riser_semantics.riser_plane_tolerance_m = 0.02F;
  value.riser_semantics.riser_lateral_tolerance_m = 0.01F;
  value.riser_semantics.riser_vertical_tolerance_m = 0.01F;
  value.leg_envelope_min = Eigen::Vector3f(-0.40F, -0.30F, -0.22F);
  value.leg_envelope_max = Eigen::Vector3f(0.45F, 0.30F, -0.02F);
  value.max_support_xy_distance_m = 0.12F;
  value.min_body_clearance_m = 0.10F;
  value.max_body_clearance_m = 0.40F;
  return value;
}

StairCollisionQuery query(std::int32_t step = 0)
{
  StairCollisionQuery value;
  value.riser_observation.snapshot = snapshot(riserNode(step));
  value.riser_observation.terrain_ground_version = 3U;
  value.riser_observation.now_nanoseconds = kStamp + 100'000'000LL;
  value.riser_observation.terrain_node_index = 0U;
  const float riser_x = static_cast<float>(step) * 0.30F;
  const float riser_z = 0.09F + static_cast<float>(step) * 0.18F;
  value.riser_observation.terrain_node_position =
    Eigen::Vector3f(riser_x, 0.0F, riser_z);
  value.riser_observation.obstacle_position =
    Eigen::Vector3f(riser_x + 0.01F, 0.0F, riser_z);
  // Put the trajectory base 0.20 m above the first-riser center.  The return
  // is therefore in the configured lower/leg-only z envelope.
  value.trajectory_origin_world = Eigen::Vector3f(riser_x, 0.0F, riser_z + 0.11F);
  value.body_axes_world = Eigen::Matrix3f::Identity();
  value.active_support_staircase_id = 2;
  value.active_support_location = -1;
  return value;
}

TEST(StairCollisionPolicy, DisabledConfigurationPreservesLegacyCollision)
{
  const auto result = StairCollisionPolicy::evaluate(
    StairCollisionPolicyConfig{}, query());
  EXPECT_FALSE(result.passthrough);
  EXPECT_EQ(result.reason, StairCollisionReason::RISER_SEMANTICS_REJECTED);
  EXPECT_EQ(
    result.riser_reason,
    perception_3d::StairRiserSemanticReason::FEATURE_DISABLED);
}

TEST(StairCollisionPolicy, ExpectedRiserInsideLegEnvelopeMayPass)
{
  const auto result = StairCollisionPolicy::evaluate(config(), query());
  EXPECT_TRUE(result.passthrough);
  EXPECT_EQ(result.reason, StairCollisionReason::PASSTHROUGH_EXPECTED_RISER);
}

TEST(StairCollisionPolicy, BodyEnvelopeAlwaysRemainsLethal)
{
  auto current = query();
  current.trajectory_origin_world.z() = 0.0F;
  const auto result = StairCollisionPolicy::evaluate(config(), current);
  EXPECT_FALSE(result.passthrough);
  EXPECT_EQ(result.reason, StairCollisionReason::OUTSIDE_LEG_ENVELOPE);
}

TEST(StairCollisionPolicy, OnlyCurrentAdjacentTransitionMayPass)
{
  auto current = query();
  current.active_support_location = 2;
  const auto remote = StairCollisionPolicy::evaluate(config(), current);
  EXPECT_FALSE(remote.passthrough);
  EXPECT_EQ(remote.reason, StairCollisionReason::NOT_ADJACENT_TRANSITION);

  current = query();
  current.active_support_staircase_id = -1;
  const auto missing = StairCollisionPolicy::evaluate(config(), current);
  EXPECT_FALSE(missing.passthrough);
  EXPECT_EQ(missing.reason, StairCollisionReason::MISSING_ACTIVE_STAIR_TRANSITION);
}

TEST(StairCollisionPolicy, FinalRiserAcceptsHighestTreadAndUpperLandingSupport)
{
  auto from_highest_tread = query(3);
  from_highest_tread.active_support_location = 3;
  EXPECT_TRUE(StairCollisionPolicy::evaluate(
    config(), from_highest_tread).passthrough);

  auto from_upper_landing = query(3);
  from_upper_landing.active_support_location = 4;
  EXPECT_TRUE(StairCollisionPolicy::evaluate(
    config(), from_upper_landing).passthrough);
}

TEST(StairCollisionPolicy, UpperLandingCannotAuthorizeNonFinalRiser)
{
  auto current = query(2);
  current.active_support_location = 4;
  const auto result = StairCollisionPolicy::evaluate(config(), current);
  EXPECT_FALSE(result.passthrough);
  EXPECT_EQ(result.reason, StairCollisionReason::NOT_ADJACENT_TRANSITION);
}

TEST(StairCollisionPolicy, EnvelopeCannotBeConfiguredIntoBodySpace)
{
  auto limits = config();
  limits.leg_envelope_max.z() = 0.10F;
  std::string error;
  EXPECT_FALSE(StairCollisionPolicy::validConfig(limits, &error));
  EXPECT_NE(error.find("z=0"), std::string::npos);
  EXPECT_FALSE(StairCollisionPolicy::evaluate(limits, query()).passthrough);
}

TEST(StairCollisionPolicy, SideWallHandrailDynamicAndMapMismatchRemainLethal)
{
  auto current = query();
  current.riser_observation.obstacle_position.y() = 0.65F;
  current.riser_observation.terrain_node_position =
    current.riser_observation.obstacle_position;
  EXPECT_FALSE(StairCollisionPolicy::evaluate(config(), current).passthrough);

  current = query();
  current.riser_observation.obstacle_position.z() = 0.50F;
  current.riser_observation.terrain_node_position =
    current.riser_observation.obstacle_position;
  EXPECT_FALSE(StairCollisionPolicy::evaluate(config(), current).passthrough);

  current = query();
  current.riser_observation.dynamic_obstacle_confirmed = true;
  EXPECT_FALSE(StairCollisionPolicy::evaluate(config(), current).passthrough);

  auto limits = config();
  limits.riser_semantics.expected_map_hash = "other-map";
  EXPECT_FALSE(StairCollisionPolicy::evaluate(limits, query()).passthrough);
}

TEST(StairCollisionPolicy, InvalidBodyAxesFailClosed)
{
  auto current = query();
  current.body_axes_world.col(1) = current.body_axes_world.col(0);
  const auto result = StairCollisionPolicy::evaluate(config(), current);
  EXPECT_FALSE(result.passthrough);
  EXPECT_EQ(result.reason, StairCollisionReason::INVALID_BODY_FRAME);
}

}  // namespace
}  // namespace mpc_critics
