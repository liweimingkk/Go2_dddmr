#include "perception_3d/shared_data.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace perception_3d
{
namespace
{

TerrainSnapshotConstPtr snapshot(const std::uint64_t version)
{
  TerrainNode node;
  node.ground_index = 0U;
  node.terrain_class = TerrainClass::FLAT;
  node.support_ratio = 1.0F;
  node.confidence = 1.0F;
  node.flags = TERRAIN_NODE_OBSERVED | TERRAIN_NODE_STATIC_MAP;
  return std::make_shared<const TerrainSnapshot>(
    "test-map", version, 1, std::vector<TerrainNode>{node});
}

TEST(StaticGroundGeneration, EveryAcceptedMessageInvalidatesEvenAtTheSameSize)
{
  SharedData shared;
  const auto first_snapshot = snapshot(1U);
  shared.setTerrainSnapshot(first_snapshot);

  // Both updates represent a one-point ground cloud.  Generation advancement
  // deliberately has no size input: equal cardinality is not map identity.
  EXPECT_EQ(shared.invalidateTerrainForStaticGroundUpdate(), 1U);
  EXPECT_EQ(shared.getStaticGroundGeneration(), 1U);
  EXPECT_EQ(shared.getTerrainSnapshot(), nullptr);

  ASSERT_TRUE(shared.setTerrainSnapshotForStaticGroundGeneration(snapshot(2U), 1U));
  ASSERT_NE(shared.getTerrainSnapshot(), nullptr);

  EXPECT_EQ(shared.invalidateTerrainForStaticGroundUpdate(), 2U);
  EXPECT_EQ(shared.getStaticGroundGeneration(), 2U);
  EXPECT_EQ(shared.getTerrainSnapshot(), nullptr);
}

TEST(StaticGroundGeneration, RejectsAStaleAsynchronousTerrainBuild)
{
  SharedData shared;
  const auto generation = shared.invalidateTerrainForStaticGroundUpdate();
  ASSERT_EQ(generation, 1U);

  // A second callback arrives while generation 1 is being built.
  ASSERT_EQ(shared.invalidateTerrainForStaticGroundUpdate(), 2U);
  EXPECT_FALSE(shared.setTerrainSnapshotForStaticGroundGeneration(
    snapshot(1U), generation));
  EXPECT_EQ(shared.getTerrainSnapshot(), nullptr);

  EXPECT_TRUE(shared.setTerrainSnapshotForStaticGroundGeneration(snapshot(2U), 2U));
  ASSERT_NE(shared.getTerrainSnapshot(), nullptr);
  EXPECT_EQ(shared.getTerrainSnapshot()->version(), 2U);
}

TEST(StaticGroundGeneration, GenerationZeroCannotPublishABoundSnapshot)
{
  SharedData shared;
  EXPECT_FALSE(shared.setTerrainSnapshotForStaticGroundGeneration(snapshot(1U), 0U));
  EXPECT_EQ(shared.getTerrainSnapshot(), nullptr);
}

TEST(StaticGroundGeneration, TwoPhasePairUpdateBlocksMixedGenerationSnapshots)
{
  SharedData shared;
  ASSERT_EQ(shared.invalidateTerrainForStaticGroundUpdate(), 1U);
  ASSERT_TRUE(shared.setTerrainSnapshotForStaticGroundGeneration(snapshot(1U), 1U));

  const auto first_token = shared.beginStaticMapGroundUpdate();
  ASSERT_NE(first_token, 0U);
  EXPECT_TRUE(shared.isStaticMapGroundUpdatePending());
  EXPECT_EQ(shared.getStaticGroundGeneration(), 1U);
  EXPECT_EQ(shared.getTerrainSnapshot(), nullptr);
  EXPECT_FALSE(shared.setTerrainSnapshotForStaticGroundGeneration(snapshot(2U), 1U));
  shared.setTerrainSnapshot(snapshot(2U));
  EXPECT_EQ(shared.getTerrainSnapshot(), nullptr);

  // The other half of the pair starts a newer epoch. Whether map or ground
  // arrives first, only the token from the latest callback may commit both.
  const auto latest_token = shared.beginStaticMapGroundUpdate();
  ASSERT_GT(latest_token, first_token);
  EXPECT_EQ(shared.commitStaticMapGroundUpdate(first_token), 0U);
  EXPECT_TRUE(shared.isStaticMapGroundUpdatePending());

  EXPECT_EQ(shared.commitStaticMapGroundUpdate(latest_token), 2U);
  EXPECT_FALSE(shared.isStaticMapGroundUpdatePending());
  EXPECT_EQ(shared.getStaticGroundGeneration(), 2U);
  EXPECT_TRUE(shared.setTerrainSnapshotForStaticGroundGeneration(snapshot(2U), 2U));
}

}  // namespace
}  // namespace perception_3d
