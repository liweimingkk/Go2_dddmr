#include <gtest/gtest.h>

#include <mpc_critics/terrain_support_policy.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <vector>

namespace mpc_critics
{
namespace
{

using perception_3d::StaircaseModel;
using perception_3d::TerrainClass;
using perception_3d::TerrainNode;
using perception_3d::TerrainRejectionReason;
using perception_3d::TerrainSnapshot;
using perception_3d::TerrainSnapshotConstPtr;

TerrainNode makeNode(
  std::uint32_t index,
  TerrainClass terrain_class = TerrainClass::FLAT,
  std::int32_t surface_id = 1)
{
  TerrainNode node;
  node.ground_index = index;
  node.normal = Eigen::Vector3f::UnitZ();
  node.support_ratio = 0.95F;
  node.confidence = 0.95F;
  node.surface_id = surface_id;
  node.terrain_class = terrain_class;
  node.flags = perception_3d::TERRAIN_NODE_OBSERVED |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  return node;
}

StaircaseModel makeStaircase()
{
  StaircaseModel staircase;
  staircase.id = 7;
  staircase.map_hash = "map-a";
  staircase.up_axis = Eigen::Vector3f::UnitX();
  staircase.width_m = 1.2F;
  staircase.riser_height_m = 0.16F;
  staircase.tread_depth_m = 0.28F;
  staircase.step_count = 5;
  staircase.confidence = 0.97F;
  staircase.allow_up = true;
  staircase.allow_down = true;
  return staircase;
}

TerrainSnapshotConstPtr makeSnapshot(
  std::vector<TerrainNode> nodes,
  std::uint64_t version = 4U,
  std::vector<StaircaseModel> staircases = {})
{
  return std::make_shared<const TerrainSnapshot>(
    "map-a", version, 1000, std::move(nodes), std::move(staircases));
}

TerrainSupportObservation observation(
  const TerrainNode & node,
  const StaircaseModel * staircase = nullptr)
{
  TerrainSupportObservation value;
  value.ground_index = node.ground_index;
  value.horizontal_distance_m = 0.03;
  value.z_gap_m = 0.02;
  value.node = &node;
  value.staircase = staircase;
  return value;
}

TEST(TerrainSupportData, MissingOrMismatchedInputsFailClosed)
{
  EXPECT_EQ(
    validateTerrainSupportData(nullptr, 0U, 0U, false),
    TerrainRejectionReason::MISSING_SNAPSHOT);

  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  EXPECT_EQ(
    validateTerrainSupportData(snapshot, 5U, 2U, true),
    TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH);
  EXPECT_EQ(
    validateTerrainSupportData(snapshot, 4U, 1U, true),
    TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH);
  EXPECT_EQ(
    validateTerrainSupportData(snapshot, 4U, 2U, false),
    TerrainRejectionReason::NO_SUPPORT);
  EXPECT_EQ(
    validateTerrainSupportData(snapshot, 4U, 2U, true),
    TerrainRejectionReason::NONE);
}

TEST(TerrainSupportPolicy, UnknownEdgeDropAndRiserAreNeverSupport)
{
  TerrainSupportLimits limits;
  for (const auto & expected : {
      std::pair{TerrainClass::UNKNOWN, TerrainRejectionReason::UNKNOWN},
      std::pair{TerrainClass::EDGE, TerrainRejectionReason::NO_SUPPORT},
      std::pair{TerrainClass::DROP, TerrainRejectionReason::DROP},
      std::pair{TerrainClass::STAIR_RISER, TerrainRejectionReason::NO_SUPPORT}})
  {
    auto node = makeNode(0U, expected.first);
    if (expected.first == TerrainClass::UNKNOWN) {
      node.surface_id = -1;
    }
    auto current = observation(node);
    EXPECT_EQ(evaluateTerrainSupport(current, nullptr, limits), expected.second);
  }
}

TEST(TerrainSupportPolicy, DistanceHeightConfidenceAndSupportAreBounded)
{
  TerrainSupportLimits limits;
  auto node = makeNode(0U);
  auto current = observation(node);

  current.horizontal_distance_m = limits.max_support_distance_m + 0.01;
  EXPECT_EQ(
    evaluateTerrainSupport(current, nullptr, limits), TerrainRejectionReason::NO_SUPPORT);

  current = observation(node);
  current.z_gap_m = limits.max_z_gap_m + 0.01;
  EXPECT_EQ(
    evaluateTerrainSupport(current, nullptr, limits), TerrainRejectionReason::NO_SUPPORT);

  current = observation(node);
  node.confidence = 0.5F;
  EXPECT_EQ(
    evaluateTerrainSupport(current, nullptr, limits), TerrainRejectionReason::LOW_CONFIDENCE);

  node.confidence = 0.95F;
  node.support_ratio = 0.5F;
  EXPECT_EQ(
    evaluateTerrainSupport(current, nullptr, limits), TerrainRejectionReason::NO_SUPPORT);
}

TEST(TerrainSupportPolicy, ContinuousGroundMustRemainOnOneSurface)
{
  TerrainSupportLimits limits;
  auto first = makeNode(0U, TerrainClass::FLAT, 1);
  auto second = makeNode(1U, TerrainClass::RAMP, 1);
  auto previous = observation(first);
  auto current = observation(second);
  EXPECT_EQ(evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::NONE);

  second.surface_id = 2;
  EXPECT_EQ(
    evaluateTerrainSupport(current, &previous, limits),
    TerrainRejectionReason::LAYER_MISMATCH);
}

TEST(TerrainSupportPolicy, ContinuousGroundRequiresFreshOnlineSupport)
{
  TerrainSupportLimits limits;
  auto node = makeNode(0U, TerrainClass::RAMP, 1);
  node.flags &=
    ~static_cast<std::uint32_t>(perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED);
  const auto current = observation(node);
  EXPECT_EQ(
    evaluateTerrainSupport(current, nullptr, limits),
    TerrainRejectionReason::LOW_CONFIDENCE);
}

TEST(TerrainSupportPolicy, StairsRequireCorridorConfirmationAndAdjacentSteps)
{
  TerrainSupportLimits limits;
  limits.stairs_enabled = true;
  const auto staircase = makeStaircase();

  auto first = makeNode(0U, TerrainClass::STAIR_TREAD, 10);
  first.staircase_id = staircase.id;
  first.step_index = 0;
  first.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  auto second = first;
  second.ground_index = 1U;
  second.step_index = 1;

  auto previous = observation(first, &staircase);
  auto current = observation(second, &staircase);
  EXPECT_EQ(evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::NONE);

  second.step_index = 2;
  EXPECT_EQ(
    evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::SKIP_STEP);

  second.step_index = 1;
  second.flags &= ~static_cast<std::uint32_t>(perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED);
  EXPECT_EQ(
    evaluateTerrainSupport(current, &previous, limits),
    TerrainRejectionReason::LOW_CONFIDENCE);
}

TEST(TerrainSupportPolicy, StairEntryAndExitRequireLanding)
{
  TerrainSupportLimits limits;
  limits.stairs_enabled = true;
  const auto staircase = makeStaircase();
  auto landing = makeNode(0U, TerrainClass::FLAT, 1);
  auto tread = makeNode(1U, TerrainClass::STAIR_TREAD, 10);
  tread.staircase_id = staircase.id;
  tread.step_index = 0;
  tread.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;

  auto previous = observation(landing);
  auto current = observation(tread, &staircase);
  EXPECT_EQ(
    evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::NO_LANDING);

  landing.staircase_id = staircase.id;
  landing.flags |= perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_LOWER_LANDING |
    perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  EXPECT_EQ(evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::NONE);

  previous = observation(tread, &staircase);
  current = observation(landing);
  EXPECT_EQ(evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::NONE);

  tread.step_index = 2;
  previous = observation(landing);
  current = observation(tread, &staircase);
  EXPECT_EQ(
    evaluateTerrainSupport(current, &previous, limits), TerrainRejectionReason::SKIP_STEP);
}

}  // namespace
}  // namespace mpc_critics
