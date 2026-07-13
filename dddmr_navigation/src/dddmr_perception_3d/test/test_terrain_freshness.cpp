#include "perception_3d/terrain_freshness.h"

#include <gtest/gtest.h>

namespace perception_3d
{
namespace
{

constexpr std::int64_t kSecond = 1'000'000'000LL;

TEST(TerrainFreshness, UsesTheTimestampOfTheCloudCapturedForTheBuild)
{
  EXPECT_TRUE(capturedTerrainInputIsFresh(true, 1 * kSecond, 900'000'000LL, 0.20));
  EXPECT_TRUE(capturedTerrainInputIsFresh(true, 1 * kSecond, 800'000'000LL, 0.20));
  EXPECT_FALSE(capturedTerrainInputIsFresh(true, 1 * kSecond, 799'999'999LL, 0.20));

  // A fresh callback elsewhere cannot refresh this older copied timestamp.
  EXPECT_FALSE(capturedTerrainInputIsFresh(true, 2 * kSecond, 1 * kSecond, 0.20));
}

TEST(TerrainFreshness, RejectsMissingInvalidAndFutureDatedLiveInputs)
{
  EXPECT_FALSE(capturedTerrainInputIsFresh(false, kSecond, 900'000'000LL, 0.20));
  EXPECT_FALSE(capturedTerrainInputIsFresh(true, kSecond, 0, 0.20));
  EXPECT_FALSE(capturedTerrainInputIsFresh(true, kSecond, kSecond + 1, 0.20));
  EXPECT_FALSE(capturedTerrainInputIsFresh(true, kSecond, 900'000'000LL, 0.0));
}

TEST(TerrainFreshness, StaticOnlySnapshotsDoNotFabricatePeriodicFreshness)
{
  EXPECT_TRUE(terrainSnapshotIsFresh(false, 60 * kSecond, kSecond, 0.20));
  EXPECT_FALSE(terrainSnapshotIsFresh(true, 60 * kSecond, kSecond, 0.20));
  EXPECT_TRUE(terrainSnapshotIsFresh(true, kSecond, 800'000'000LL, 0.20));
  EXPECT_FALSE(terrainSnapshotIsFresh(false, kSecond, kSecond + 1, 0.20));
}

}  // namespace
}  // namespace perception_3d
