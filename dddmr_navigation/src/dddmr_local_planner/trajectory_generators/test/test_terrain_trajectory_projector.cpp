/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include <gtest/gtest.h>

#include <trajectory_generators/terrain_trajectory_projector.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace trajectory_generators
{
namespace
{

constexpr std::uint32_t kVerifiedStairFlags =
  perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
  perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;

perception_3d::TerrainNode node(
  std::uint32_t index,
  perception_3d::TerrainClass terrain_class,
  std::int32_t surface_id,
  const Eigen::Vector3f & normal = Eigen::Vector3f::UnitZ())
{
  perception_3d::TerrainNode value;
  value.ground_index = index;
  value.normal = normal;
  value.slope_rad = std::acos(std::clamp(normal.normalized().z(), -1.0F, 1.0F));
  value.roughness_m = 0.01F;
  value.support_ratio = 0.96F;
  value.confidence = 0.97F;
  value.surface_id = surface_id;
  value.terrain_class = terrain_class;
  value.flags = perception_3d::TERRAIN_NODE_OBSERVED |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  return value;
}

perception_3d::StaircaseModel staircase()
{
  perception_3d::StaircaseModel model;
  model.id = 3;
  model.map_hash = "test-map";
  model.up_axis = Eigen::Vector3f::UnitX();
  model.lower_landing_center = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
  model.upper_landing_center = Eigen::Vector3f(0.9F, 0.0F, 0.36F);
  model.first_riser_center = Eigen::Vector3f(0.05F, 0.0F, 0.09F);
  model.corridor_polygon_xy = {
    {-0.05F, -0.30F}, {0.75F, -0.30F}, {0.75F, 0.30F}, {-0.05F, 0.30F}};
  model.lower_landing_polygon_xy = {
    {-0.30F, -0.30F}, {0.10F, -0.30F}, {0.10F, 0.30F}, {-0.30F, 0.30F}};
  model.upper_landing_polygon_xy = {
    {0.65F, -0.30F}, {1.10F, -0.30F}, {1.10F, 0.30F}, {0.65F, 0.30F}};
  model.width_m = 0.60F;
  model.riser_height_m = 0.18F;
  model.tread_depth_m = 0.30F;
  model.step_count = 2;
  model.confidence = 0.98F;
  model.allow_up = true;
  model.allow_down = true;
  return model;
}

TerrainProjectionContextConstPtr context(
  std::vector<Eigen::Vector3d> points,
  std::vector<perception_3d::TerrainNode> nodes,
  std::vector<perception_3d::StaircaseModel> staircases = {},
  std::uint64_t version = 11U,
  std::uint64_t generation = 7U)
{
  auto snapshot = std::make_shared<const perception_3d::TerrainSnapshot>(
    "test-map", version, 1, std::move(nodes), std::move(staircases));
  return std::make_shared<const TerrainProjectionContext>(
    std::move(snapshot), generation, std::move(points), 0.05);
}

TerrainTrajectoryProjectionLimits rampLimits()
{
  TerrainTrajectoryProjectionLimits limits;
  limits.max_support_xy_distance_m = 0.075;
  limits.max_support_vertical_distance_m = 0.25;
  limits.min_confidence = 0.90;
  limits.min_support_ratio = 0.80;
  limits.max_normal_change_rad = 0.50;
  return limits;
}

TerrainTrajectoryProjectionLimits stairLimits()
{
  TerrainTrajectoryProjectionLimits limits;
  limits.max_support_xy_distance_m = 0.16;
  limits.max_support_vertical_distance_m = 0.25;
  limits.min_confidence = 0.90;
  limits.min_support_ratio = 0.80;
  limits.max_normal_change_rad = 0.50;
  limits.stairs_enabled = true;
  limits.stair_transition_waypoint_tolerance_m = 0.04;
  limits.max_stair_transition_span_m = 0.35;
  limits.max_stair_heading_error_rad = 0.10;
  return limits;
}

TerrainProjectionContextConstPtr verifiedStairContext(
  bool step_zero_online = true,
  perception_3d::TerrainClass step_zero_class = perception_3d::TerrainClass::STAIR_TREAD)
{
  std::vector<Eigen::Vector3d> points{
    {0.0, 0.0, 0.0},
    {0.20, 0.0, 0.18},
    {0.50, 0.0, 0.36},
    {0.70, 0.0, 0.36}};
  std::vector<perception_3d::TerrainNode> nodes;
  nodes.push_back(node(0U, perception_3d::TerrainClass::FLAT, 10));
  nodes.back().staircase_id = 3;
  nodes.back().flags |= kVerifiedStairFlags |
    perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_LOWER_LANDING;
  nodes.push_back(node(1U, step_zero_class, 100));
  nodes.back().staircase_id = 3;
  nodes.back().step_index = 0;
  nodes.back().flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR;
  if (step_zero_online) {
    nodes.back().flags |= perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  } else {
    nodes.back().flags &=
      ~static_cast<std::uint32_t>(perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED);
  }
  nodes.push_back(node(2U, perception_3d::TerrainClass::STAIR_TREAD, 101));
  nodes.back().staircase_id = 3;
  nodes.back().step_index = 1;
  nodes.back().flags |= kVerifiedStairFlags;
  nodes.push_back(node(3U, perception_3d::TerrainClass::FLAT, 20));
  nodes.back().staircase_id = 3;
  nodes.back().flags |= kVerifiedStairFlags |
    perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_UPPER_LANDING;
  return context(std::move(points), std::move(nodes), {staircase()});
}

TEST(TerrainTrajectoryProjector, RampUpdatesFutureZNormalOrientationAndConfidence)
{
  const Eigen::Vector3f normal = Eigen::Vector3f(-0.4472136F, 0.0F, 0.8944272F);
  std::vector<Eigen::Vector3d> points{
    {0.0, 0.0, 0.0}, {0.1, 0.0, 0.05}, {0.2, 0.0, 0.10}, {0.3, 0.0, 0.15}};
  std::vector<perception_3d::TerrainNode> nodes;
  for (std::uint32_t index = 0U; index < points.size(); ++index) {
    nodes.push_back(node(index, perception_3d::TerrainClass::RAMP, 5, normal));
  }
  const auto terrain_context = context(std::move(points), std::move(nodes));
  ASSERT_TRUE(terrain_context->valid()) << terrain_context->validationError();

  TerrainTrajectoryProjector projector(rampLimits());
  const auto result = projector.project(
    terrain_context,
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {
      TerrainBodyPose{Eigen::Vector3d(0.1, 0.0, 0.30), 0.0},
      TerrainBodyPose{Eigen::Vector3d(0.2, 0.0, 0.30), 0.0},
      TerrainBodyPose{Eigen::Vector3d(0.3, 0.0, 0.30), 0.0}});

  ASSERT_TRUE(result.ok()) << toString(result.rejection);
  ASSERT_EQ(result.poses.size(), 3U);
  EXPECT_NEAR(result.poses[0].position.z(), 0.35, 1e-6);
  EXPECT_NEAR(result.poses[1].position.z(), 0.40, 1e-6);
  EXPECT_NEAR(result.poses[2].position.z(), 0.45, 1e-6);
  EXPECT_NEAR(result.poses[1].ground_normal.x(), normal.x(), 1e-6);
  EXPECT_NEAR(result.poses[1].ground_normal.z(), normal.z(), 1e-6);
  EXPECT_NEAR(result.poses[1].confidence, 0.97, 1e-6);
  const Eigen::Vector3d body_up = result.poses[1].orientation * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(body_up.isApprox(result.poses[1].ground_normal, 1e-6));
}

TEST(TerrainTrajectoryProjector, MissingLiveRampSupportFailsClosed)
{
  std::vector<Eigen::Vector3d> points{{0.0, 0.0, 0.0}, {0.1, 0.0, 0.02}};
  std::vector<perception_3d::TerrainNode> nodes{
    node(0U, perception_3d::TerrainClass::RAMP, 5),
    node(1U, perception_3d::TerrainClass::RAMP, 5)};
  nodes[1].flags &=
    ~static_cast<std::uint32_t>(perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED);
  const auto terrain_context = context(std::move(points), std::move(nodes));
  TerrainTrajectoryProjector projector(rampLimits());
  TerrainBodyPose reference;
  reference.position = Eigen::Vector3d(0.0, 0.0, 0.24);
  TerrainBodyPose future;
  future.position = Eigen::Vector3d(0.1, 0.0, 0.24);
  const auto result = projector.project(terrain_context, reference, 0.24, {future});
  EXPECT_EQ(
    result.rejection, TerrainTrajectoryRejection::ONLINE_CONFIRMATION_MISSING);
}

TEST(TerrainTrajectoryProjector, MissingSnapshotAndMismatchedGroundFailClosed)
{
  TerrainTrajectoryProjector projector(rampLimits());
  auto result = projector.project(
    nullptr,
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.1, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::MISSING_SNAPSHOT);

  std::vector<perception_3d::TerrainNode> nodes{
    node(0U, perception_3d::TerrainClass::FLAT, 1),
    node(1U, perception_3d::TerrainClass::FLAT, 1)};
  const auto bad_context = context({{0.0, 0.0, 0.0}}, std::move(nodes));
  ASSERT_FALSE(bad_context->valid());
  result = projector.project(
    bad_context,
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.1, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::SNAPSHOT_GROUND_MISMATCH);
}

TEST(TerrainTrajectoryProjector, UnknownEdgeAndDropInvalidateWholeTrajectory)
{
  for (const auto & test_case : {
      std::pair{perception_3d::TerrainClass::UNKNOWN, TerrainTrajectoryRejection::UNKNOWN},
      std::pair{perception_3d::TerrainClass::EDGE, TerrainTrajectoryRejection::EDGE},
      std::pair{perception_3d::TerrainClass::DROP, TerrainTrajectoryRejection::DROP}})
  {
    SCOPED_TRACE(perception_3d::toString(test_case.first));
    auto reference = node(0U, perception_3d::TerrainClass::FLAT, 1);
    auto hazard = node(1U, test_case.first, test_case.first ==
      perception_3d::TerrainClass::UNKNOWN ? -1 : 1);
    const auto terrain_context = context(
      {{0.0, 0.0, 0.0}, {0.10, 0.0, 0.0}}, {reference, hazard});
    ASSERT_TRUE(terrain_context->valid()) << terrain_context->validationError();
    TerrainTrajectoryProjector projector(rampLimits());
    const auto result = projector.project(
      terrain_context,
      TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
      0.30,
      {TerrainBodyPose{Eigen::Vector3d(0.10, 0.0, 0.30), 0.0}});
    EXPECT_EQ(result.rejection, test_case.second);
    EXPECT_TRUE(result.poses.empty());
  }
}

TEST(TerrainTrajectoryProjector, CrossingOrdinarySurfaceInvalidatesWholeTrajectory)
{
  const auto terrain_context = context(
    {{0.0, 0.0, 0.0}, {0.10, 0.0, 0.0}},
    {node(0U, perception_3d::TerrainClass::FLAT, 1),
      node(1U, perception_3d::TerrainClass::FLAT, 2)});
  TerrainTrajectoryProjector projector(rampLimits());
  const auto result = projector.project(
    terrain_context,
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.10, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::SURFACE_CHANGE);
  EXPECT_TRUE(result.poses.empty());
}

TEST(TerrainTrajectoryProjector, VerifiedAdjacentStairWaypointsAndLandingsAreProjected)
{
  const auto terrain_context = verifiedStairContext();
  ASSERT_TRUE(terrain_context->valid()) << terrain_context->validationError();
  TerrainTrajectoryProjector projector(stairLimits());
  const auto result = projector.project(
    terrain_context,
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {
      TerrainBodyPose{Eigen::Vector3d(0.20, 0.0, 0.30), 0.0},
      TerrainBodyPose{Eigen::Vector3d(0.50, 0.0, 0.30), 0.0},
      TerrainBodyPose{Eigen::Vector3d(0.70, 0.0, 0.30), 0.0}});

  ASSERT_TRUE(result.ok()) << toString(result.rejection);
  ASSERT_EQ(result.poses.size(), 3U);
  EXPECT_EQ(result.poses[0].step_index, 0);
  EXPECT_EQ(result.poses[1].step_index, 1);
  EXPECT_TRUE(result.poses[2].terrain_class == perception_3d::TerrainClass::FLAT);
  EXPECT_NEAR(result.poses[0].position.z(), 0.48, 1e-6);
  EXPECT_NEAR(result.poses[1].position.z(), 0.66, 1e-6);
  EXPECT_NEAR(result.poses[2].position.z(), 0.66, 1e-6);
}

TEST(TerrainTrajectoryProjector, StairSkipStepIsRejected)
{
  auto limits = stairLimits();
  limits.max_support_vertical_distance_m = 0.40;
  TerrainTrajectoryProjector projector(limits);
  const auto result = projector.project(
    verifiedStairContext(),
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.50, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::SKIP_STEP);
  EXPECT_TRUE(result.poses.empty());
}

TEST(TerrainTrajectoryProjector, StairTransitionRequiresOnlineVerifiedWaypoint)
{
  auto limits = stairLimits();
  limits.require_online_confirmation = false;
  TerrainTrajectoryProjector projector(limits);
  const auto result = projector.project(
    verifiedStairContext(false),
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.20, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::TRANSITION_WAYPOINT_MISSING);
}

TEST(TerrainTrajectoryProjector, StairRiserCannotBecomeBodySupport)
{
  TerrainTrajectoryProjector projector(stairLimits());
  const auto result = projector.project(
    verifiedStairContext(true, perception_3d::TerrainClass::STAIR_RISER),
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.20, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::RISER);
  EXPECT_TRUE(result.poses.empty());
}

TEST(TerrainTrajectoryProjector, StairEntryCannotBypassTypedLowerLanding)
{
  std::vector<Eigen::Vector3d> points{{0.0, 0.0, 0.0}, {0.20, 0.0, 0.18}};
  std::vector<perception_3d::TerrainNode> nodes;
  nodes.push_back(node(0U, perception_3d::TerrainClass::FLAT, 10));
  nodes.push_back(node(1U, perception_3d::TerrainClass::STAIR_TREAD, 100));
  nodes.back().staircase_id = 3;
  nodes.back().step_index = 0;
  nodes.back().flags |= kVerifiedStairFlags;
  TerrainTrajectoryProjector projector(stairLimits());
  const auto result = projector.project(
    context(std::move(points), std::move(nodes), {staircase()}),
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.0, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.20, 0.0, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::NO_LANDING);
  EXPECT_TRUE(result.poses.empty());
}

TEST(TerrainTrajectoryProjector, StairTransitionMustPassItsVerifiedWaypoint)
{
  auto limits = stairLimits();
  limits.max_support_xy_distance_m = 0.25;
  TerrainTrajectoryProjector projector(limits);
  const auto result = projector.project(
    verifiedStairContext(),
    TerrainBodyPose{Eigen::Vector3d(0.0, 0.20, 0.30), 0.0},
    0.30,
    {TerrainBodyPose{Eigen::Vector3d(0.20, 0.20, 0.30), 0.0}});
  EXPECT_EQ(result.rejection, TerrainTrajectoryRejection::TRANSITION_WAYPOINT_MISSED);
}

TEST(TerrainTrajectoryProjector, ContextLeaseRejectsPointerOrGenerationChange)
{
  const auto leased = context(
    {{0.0, 0.0, 0.0}}, {node(0U, perception_3d::TerrainClass::FLAT, 1)}, {}, 11U, 7U);
  EXPECT_TRUE(terrainProjectionContextIdentityMatches(leased, leased));

  const auto replacement_same_ids = context(
    {{0.0, 0.0, 0.0}}, {node(0U, perception_3d::TerrainClass::FLAT, 1)}, {}, 11U, 7U);
  EXPECT_FALSE(terrainProjectionContextIdentityMatches(leased, replacement_same_ids));

  const auto replacement_generation = context(
    {{0.0, 0.0, 0.0}}, {node(0U, perception_3d::TerrainClass::FLAT, 1)}, {}, 11U, 8U);
  EXPECT_FALSE(terrainProjectionContextIdentityMatches(leased, replacement_generation));
}

}  // namespace
}  // namespace trajectory_generators
