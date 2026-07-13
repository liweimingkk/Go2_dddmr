#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "terrain_roi_config.h"

namespace dddmr_pg_map_server
{

namespace
{

struct TestPoint
{
  double x;
  double y;
  double z;
  int id;
};

TerrainROIConfig makeValidTerrainROIConfig()
{
  TerrainROIConfig config;
  config.enabled = true;
  config.voxel_size = 0.05;
  config.minimum = {{-1.0, -1.0, -1.0}};
  config.maximum = {{1.0, 1.0, 1.0}};
  return config;
}

}  // namespace

TEST(TerrainROIConfig, ValidatesVoxelSizes)
{
  EXPECT_TRUE(validateVoxelSize("voxel", 0.2).valid);
  EXPECT_FALSE(validateVoxelSize("voxel", 0.0).valid);
  EXPECT_FALSE(validateVoxelSize("voxel", -0.1).valid);
  EXPECT_FALSE(
    validateVoxelSize(
      "voxel", std::numeric_limits<double>::infinity()).valid);
}

TEST(MapIdentity, AcceptsOnlyAndNormalizesCompleteSha256)
{
  const std::string uppercase(64U, 'A');
  EXPECT_TRUE(isValidMapSha256(uppercase));
  EXPECT_EQ(normalizeMapSha256(uppercase), std::string(64U, 'a'));
  EXPECT_FALSE(isValidMapSha256(std::string(63U, 'a')));
  EXPECT_FALSE(isValidMapSha256(std::string(64U, 'z')));
  EXPECT_TRUE(normalizeMapSha256("not-a-hash").empty());
}

TEST(MapIdentity, TerrainRequiresAnExactMeasuredArtifactMatch)
{
  const std::string measured(64U, 'a');
  EXPECT_TRUE(validateLoadedMapIdentity(true, measured, measured).valid);
  EXPECT_TRUE(validateLoadedMapIdentity(true, std::string(64U, 'A'), measured).valid);
  EXPECT_FALSE(validateLoadedMapIdentity(true, "", measured).valid);
  EXPECT_FALSE(validateLoadedMapIdentity(true, std::string(64U, 'b'), measured).valid);
  EXPECT_FALSE(validateLoadedMapIdentity(true, measured, "unavailable").valid);
}

TEST(MapIdentity, TerrainDisabledPreservesLegacyFlatCompatibility)
{
  EXPECT_TRUE(validateLoadedMapIdentity(false, "", "").valid);
  EXPECT_TRUE(validateLoadedMapIdentity(
    false, std::string(64U, 'a'), std::string(64U, 'b')).valid);
}

TEST(TerrainROIConfig, DisabledROIIsBackwardCompatible)
{
  TerrainROIConfig config;
  config.enabled = false;
  config.voxel_size = 0.0;

  EXPECT_TRUE(validateTerrainROIConfig(config, 0.2).valid);
}

TEST(TerrainROIConfig, AcceptsFiniteHighResolutionBounds)
{
  TerrainROIConfig config;
  config.enabled = true;
  config.voxel_size = 0.05;
  config.minimum = {{-2.0, 1.0, -0.5}};
  config.maximum = {{3.0, 4.0, 2.0}};

  EXPECT_TRUE(validateTerrainROIConfig(config, 0.2).valid);
  EXPECT_TRUE(pointIsInsideTerrainROI(config, -2.0, 4.0, 0.0));
  EXPECT_FALSE(pointIsInsideTerrainROI(config, -2.01, 4.0, 0.0));
  EXPECT_FALSE(pointIsInsideTerrainROI(
    config, std::numeric_limits<double>::quiet_NaN(), 2.0, 0.0));
}

TEST(TerrainROIConfig, RejectsInvalidOrLowResolutionROI)
{
  TerrainROIConfig config;
  config.enabled = true;
  config.voxel_size = 0.3;
  config.minimum = {{-1.0, -1.0, -1.0}};
  config.maximum = {{1.0, 1.0, 1.0}};

  EXPECT_FALSE(validateTerrainROIConfig(config, 0.2).valid);

  config.voxel_size = 0.05;
  EXPECT_FALSE(validateTerrainROIConfig(config, 0.0).valid);

  config.maximum[1] = config.minimum[1];
  EXPECT_FALSE(validateTerrainROIConfig(config, 0.2).valid);

  config.maximum[1] = 1.0;
  config.maximum[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateTerrainROIConfig(config, 0.2).valid);
}

TEST(NavigationGroundMerge, RetainsOutsideAndReplacesInsideROI)
{
  const auto config = makeValidTerrainROIConfig();
  const std::vector<TestPoint> coarse_ground{
    {-2.0, 0.0, 0.0, 1},
    {0.0, 0.0, 0.0, 2},
    {1.0, 0.0, 0.0, 3},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 4}};
  const std::vector<TestPoint> terrain_ground{
    {0.0, 0.0, 0.1, 5},
    {0.5, 0.0, 0.2, 6},
    {0.5, 0.0, 0.2, 7},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 8}};

  const auto result = mergeNavigationGroundPoints(
    coarse_ground, terrain_ground, config, 0.2);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_EQ(result.points[0].id, 1);
  EXPECT_EQ(result.points[1].id, 5);
  EXPECT_EQ(result.points[2].id, 6);
  EXPECT_EQ(result.coarse_points_retained, 1U);
  EXPECT_EQ(result.terrain_points_added, 2U);
  EXPECT_EQ(result.duplicate_points_removed, 1U);
}

TEST(NavigationGroundMerge, EmptyOrInvalidTerrainFailsClosed)
{
  auto config = makeValidTerrainROIConfig();
  const std::vector<TestPoint> coarse_ground{{-2.0, 0.0, 0.0, 1}};
  const std::vector<TestPoint> empty_terrain;

  auto result = mergeNavigationGroundPoints(
    coarse_ground, empty_terrain, config, 0.2);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());

  config.maximum[0] = config.minimum[0];
  const std::vector<TestPoint> terrain_ground{{0.0, 0.0, 0.0, 2}};
  result = mergeNavigationGroundPoints(
    coarse_ground, terrain_ground, config, 0.2);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

TEST(NavigationGroundMerge, TerrainOutsideROIFailsClosed)
{
  const auto config = makeValidTerrainROIConfig();
  const std::vector<TestPoint> coarse_ground{{-2.0, 0.0, 0.0, 1}};
  const std::vector<TestPoint> terrain_ground{{2.0, 0.0, 0.0, 2}};

  const auto result = mergeNavigationGroundPoints(
    coarse_ground, terrain_ground, config, 0.2);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

}  // namespace dddmr_pg_map_server
