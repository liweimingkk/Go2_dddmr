#include <gtest/gtest.h>

#include "perception_3d/stair_riser_semantics.h"

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace perception_3d
{
namespace
{

constexpr std::int64_t kStamp = 1'000'000'000LL;

StaircaseModel staircase()
{
  StaircaseModel value;
  value.id = 4;
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

TerrainNode riserNode(std::uint32_t index = 0U, std::int32_t step = 0)
{
  TerrainNode node;
  node.ground_index = index;
  node.normal = -Eigen::Vector3f::UnitX();
  node.slope_rad = 1.57079632679F;
  node.roughness_m = 0.01F;
  node.support_ratio = 0.0F;
  node.confidence = 0.0F;
  node.surface_id = 100;
  node.staircase_id = 4;
  node.step_index = step;
  node.terrain_class = TerrainClass::STAIR_RISER;
  node.flags = TERRAIN_NODE_STATIC_MAP | TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED;
  return node;
}

TerrainNode treadAnchor(std::int32_t step = 0)
{
  TerrainNode node;
  node.ground_index = 0U;
  node.normal = Eigen::Vector3f::UnitZ();
  node.support_ratio = 0.95F;
  node.confidence = 0.95F;
  node.surface_id = 200 + step;
  node.staircase_id = 4;
  node.step_index = step;
  node.terrain_class = TerrainClass::STAIR_TREAD;
  node.flags = TERRAIN_NODE_STATIC_MAP | TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED;
  return node;
}

TerrainNode lowerLandingAnchor()
{
  TerrainNode node;
  node.ground_index = 0U;
  node.normal = Eigen::Vector3f::UnitZ();
  node.support_ratio = 0.95F;
  node.confidence = 0.95F;
  node.surface_id = 20;
  node.staircase_id = 4;
  node.terrain_class = TerrainClass::FLAT;
  node.flags = TERRAIN_NODE_STATIC_MAP | TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED | TERRAIN_NODE_LANDING |
    TERRAIN_NODE_LOWER_LANDING;
  return node;
}

TerrainNode upperLandingAnchor()
{
  TerrainNode node;
  node.ground_index = 0U;
  node.normal = Eigen::Vector3f::UnitZ();
  node.support_ratio = 0.95F;
  node.confidence = 0.95F;
  node.surface_id = 21;
  node.staircase_id = 4;
  node.terrain_class = TerrainClass::FLAT;
  node.flags = TERRAIN_NODE_STATIC_MAP | TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED | TERRAIN_NODE_LANDING |
    TERRAIN_NODE_UPPER_LANDING;
  return node;
}

TerrainSnapshotConstPtr snapshot(TerrainNode node = riserNode())
{
  return std::make_shared<const TerrainSnapshot>(
    "map-a", 7U, kStamp, std::vector<TerrainNode>{std::move(node)},
    std::vector<StaircaseModel>{staircase()});
}

StairRiserSemanticsConfig config()
{
  StairRiserSemanticsConfig value;
  value.enabled = true;
  value.fail_closed = true;
  value.expected_map_hash = "map-a";
  value.max_snapshot_age_nanoseconds = 200'000'000LL;
  value.minimum_stair_confidence = 0.90F;
  value.max_node_match_distance_m = 0.04F;
  value.riser_plane_tolerance_m = 0.02F;
  value.riser_lateral_tolerance_m = 0.01F;
  value.riser_vertical_tolerance_m = 0.01F;
  return value;
}

StairRiserObservation observation()
{
  StairRiserObservation value;
  value.snapshot = snapshot();
  value.terrain_ground_version = 7U;
  value.now_nanoseconds = kStamp + 100'000'000LL;
  value.terrain_node_index = 0U;
  value.terrain_node_position = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  value.obstacle_position = Eigen::Vector3f(0.01F, 0.0F, 0.09F);
  return value;
}

TEST(StairRiserSemantics, DisabledAndZeroToleranceDefaultsNeverPass)
{
  const auto disabled = StairRiserSemantics::classify(
    StairRiserSemanticsConfig{}, observation());
  EXPECT_FALSE(disabled.expected_riser);
  EXPECT_EQ(disabled.reason, StairRiserSemanticReason::FEATURE_DISABLED);

  auto invalid = config();
  invalid.riser_plane_tolerance_m = 0.0F;
  const auto result = StairRiserSemantics::classify(invalid, observation());
  EXPECT_FALSE(result.expected_riser);
  EXPECT_EQ(result.reason, StairRiserSemanticReason::INVALID_CONFIGURATION);
}

TEST(StairRiserSemantics, PassesOnlySurveyedConfirmedStaticRiser)
{
  const auto result = StairRiserSemantics::classify(config(), observation());
  EXPECT_TRUE(result.expected_riser);
  EXPECT_EQ(result.reason, StairRiserSemanticReason::EXPECTED_RISER);
  EXPECT_EQ(result.staircase_id, 4);
  EXPECT_EQ(result.step_index, 0);
}

TEST(StairRiserSemantics, MapVersionAndAgeMismatchFailClosed)
{
  auto current = observation();
  auto limits = config();
  limits.expected_map_hash = "other-map";
  EXPECT_EQ(
    StairRiserSemantics::classify(limits, current).reason,
    StairRiserSemanticReason::MAP_MISMATCH);

  current.terrain_ground_version = 8U;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::SNAPSHOT_VERSION_MISMATCH);

  current = observation();
  current.now_nanoseconds = kStamp + 300'000'000LL;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::STALE_SNAPSHOT);
}

TEST(StairRiserSemantics, ManualOnlineAndStaticFlagsAreAllRequired)
{
  const std::vector<std::pair<std::uint32_t, StairRiserSemanticReason>> cases{
    {TERRAIN_NODE_STATIC_MAP, StairRiserSemanticReason::MISSING_STATIC_MAP_FLAG},
    {TERRAIN_NODE_MANUAL_CORRIDOR, StairRiserSemanticReason::OUTSIDE_MANUAL_CORRIDOR},
    {TERRAIN_NODE_ONLINE_CONFIRMED,
      StairRiserSemanticReason::MISSING_ONLINE_CONFIRMATION}};
  for (const auto & [flag, expected_reason] : cases) {
    auto node = riserNode();
    node.flags &= ~flag;
    auto current = observation();
    current.snapshot = snapshot(node);
    EXPECT_EQ(StairRiserSemantics::classify(config(), current).reason, expected_reason);
  }
}

TEST(StairRiserSemantics, DynamicOrUnmodeledObstacleRemainsLethal)
{
  auto current = observation();
  current.dynamic_obstacle_confirmed = true;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::DYNAMIC_OBSTACLE);

  current = observation();
  current.obstacle_position.x() = 0.10F;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::NODE_DISTANCE);

  current = observation();
  current.terrain_node_position = Eigen::Vector3f(0.10F, 0.0F, 0.09F);
  current.obstacle_position = current.terrain_node_position;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::RISER_GEOMETRY_MISMATCH);
}

TEST(StairRiserSemantics, SideWallHandrailAndWrongStepRemainLethal)
{
  auto current = observation();
  current.terrain_node_position = Eigen::Vector3f(0.0F, 0.65F, 0.09F);
  current.obstacle_position = current.terrain_node_position;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::OUTSIDE_MANUAL_CORRIDOR);

  current = observation();
  current.terrain_node_position = Eigen::Vector3f(0.0F, 0.0F, 0.50F);
  current.obstacle_position = current.terrain_node_position;
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::RISER_GEOMETRY_MISMATCH);

  current = observation();
  current.snapshot = snapshot(riserNode(0U, 1));
  EXPECT_EQ(
    StairRiserSemantics::classify(config(), current).reason,
    StairRiserSemanticReason::STEP_INDEX_MISMATCH);
}

TEST(StairRiserSemantics, AdjacentSurveyedRiserUsesExplicitAnchor)
{
  auto current = observation();
  current.snapshot = snapshot(riserNode(0U, 1));
  current.terrain_node_position = Eigen::Vector3f(0.30F, 0.0F, 0.27F);
  current.obstacle_position = Eigen::Vector3f(0.31F, 0.0F, 0.27F);
  const auto result = StairRiserSemantics::classify(config(), current);
  EXPECT_TRUE(result.expected_riser);
  EXPECT_EQ(result.step_index, 1);
}

TEST(StairRiserSemantics, GroundWithoutVerticalRiserUsesAdjacentSupportAnchor)
{
  auto limits = config();
  limits.max_node_match_distance_m = 0.12F;

  auto current = observation();
  current.snapshot = snapshot(treadAnchor(0));
  current.terrain_node_position = Eigen::Vector3f(0.02F, 0.0F, 0.18F);
  current.obstacle_position = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  auto result = StairRiserSemantics::classify(limits, current);
  EXPECT_TRUE(result.expected_riser);
  EXPECT_EQ(result.step_index, 0);

  current.snapshot = snapshot(lowerLandingAnchor());
  current.terrain_node_position = Eigen::Vector3f(-0.02F, 0.0F, 0.0F);
  result = StairRiserSemantics::classify(limits, current);
  EXPECT_TRUE(result.expected_riser);
  EXPECT_EQ(result.step_index, 0);
}

TEST(StairRiserSemantics, UpperLandingAuthorizesOnlyFinalRiser)
{
  auto limits = config();
  limits.max_node_match_distance_m = 0.12F;

  auto current = observation();
  current.snapshot = snapshot(upperLandingAnchor());
  current.terrain_node_position = Eigen::Vector3f(0.92F, 0.0F, 0.72F);
  current.obstacle_position = Eigen::Vector3f(0.90F, 0.0F, 0.63F);
  auto result = StairRiserSemantics::classify(limits, current);
  EXPECT_TRUE(result.expected_riser);
  EXPECT_EQ(result.step_index, 3);

  // Relax only the bounded association radius so adjacency, rather than
  // distance, is the reason an upper landing cannot authorize an earlier riser.
  limits.max_node_match_distance_m = 0.50F;
  current.obstacle_position = Eigen::Vector3f(0.60F, 0.0F, 0.45F);
  result = StairRiserSemantics::classify(limits, current);
  EXPECT_FALSE(result.expected_riser);
  EXPECT_EQ(result.reason, StairRiserSemanticReason::ANCHOR_NOT_ADJACENT);
}

TEST(StairRiserSemantics, SupportAnchorMustBeAdjacentNearAndOnline)
{
  auto limits = config();
  limits.max_node_match_distance_m = 0.12F;
  auto current = observation();
  current.obstacle_position = Eigen::Vector3f(0.0F, 0.0F, 0.09F);

  current.snapshot = snapshot(treadAnchor(2));
  current.terrain_node_position = Eigen::Vector3f(0.02F, 0.0F, 0.18F);
  EXPECT_EQ(
    StairRiserSemantics::classify(limits, current).reason,
    StairRiserSemanticReason::ANCHOR_NOT_ADJACENT);

  current.snapshot = snapshot(treadAnchor(0));
  current.terrain_node_position = Eigen::Vector3f(0.20F, 0.0F, 0.18F);
  EXPECT_EQ(
    StairRiserSemantics::classify(limits, current).reason,
    StairRiserSemanticReason::NODE_DISTANCE);

  auto no_online = treadAnchor(0);
  no_online.flags &= ~static_cast<std::uint32_t>(TERRAIN_NODE_ONLINE_CONFIRMED);
  current.snapshot = snapshot(no_online);
  current.terrain_node_position = Eigen::Vector3f(0.02F, 0.0F, 0.18F);
  EXPECT_EQ(
    StairRiserSemantics::classify(limits, current).reason,
    StairRiserSemanticReason::MISSING_ONLINE_CONFIRMATION);
}

TEST(StairRiserSemantics, AnchorFromAnotherStaircaseCannotAuthorizeRiser)
{
  auto other = staircase();
  other.id = 8;
  other.lower_landing_center.x() += 2.0F;
  other.upper_landing_center.x() += 2.0F;
  other.first_riser_center.x() += 2.0F;
  for (auto * polygon : {
      &other.corridor_polygon_xy,
      &other.lower_landing_polygon_xy,
      &other.upper_landing_polygon_xy})
  {
    for (auto & point : *polygon) {
      point.x() += 2.0F;
    }
  }
  auto anchor = treadAnchor(0);
  anchor.staircase_id = other.id;
  const auto two_stair_snapshot = std::make_shared<const TerrainSnapshot>(
    "map-a", 7U, kStamp, std::vector<TerrainNode>{anchor},
    std::vector<StaircaseModel>{staircase(), other});
  ASSERT_TRUE(two_stair_snapshot->valid());

  auto limits = config();
  limits.max_node_match_distance_m = 0.12F;
  auto current = observation();
  current.snapshot = two_stair_snapshot;
  current.terrain_node_position = Eigen::Vector3f(0.02F, 0.0F, 0.18F);
  current.obstacle_position = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  EXPECT_FALSE(StairRiserSemantics::classify(limits, current).expected_riser);
}

TEST(TerrainSurfaceProjection, RequiresSurfaceIdentityAndReferencePlaneAgreement)
{
  TerrainNode reference;
  reference.surface_id = 8;
  reference.normal = Eigen::Vector3f::UnitZ();
  TerrainNode candidate = reference;
  EXPECT_TRUE(terrainSurfaceProjectionCompatible(
    reference, Eigen::Vector3f(0.0F, 0.0F, 0.0F),
    candidate, Eigen::Vector3f(0.2F, 0.0F, 0.01F), 0.02F));

  candidate.surface_id = 9;
  EXPECT_FALSE(terrainSurfaceProjectionCompatible(
    reference, Eigen::Vector3f(0.0F, 0.0F, 0.0F),
    candidate, Eigen::Vector3f(0.2F, 0.0F, 0.01F), 0.02F));

  candidate.surface_id = 8;
  EXPECT_FALSE(terrainSurfaceProjectionCompatible(
    reference, Eigen::Vector3f(0.0F, 0.0F, 0.0F),
    candidate, Eigen::Vector3f(0.2F, 0.0F, 0.03F), 0.02F));
}

}  // namespace
}  // namespace perception_3d
