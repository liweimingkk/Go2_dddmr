#include <global_planner/endpoint_projection_policy.h>

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace global_planner
{

TEST(EndpointProjectionPolicy, TreatsContinuousSlopeAsOneSurface)
{
  const std::vector<double> slope_z{
    -2.68, -2.61, -2.55, -2.47, -2.38, -2.29, -2.20, -2.11, -2.03};

  const auto bands = findSupportedVerticalSurfaceBands(slope_z, 0.26, 3U);

  ASSERT_EQ(bands.size(), 1U);
  EXPECT_DOUBLE_EQ(bands.front().min_z, -2.68);
  EXPECT_DOUBLE_EQ(bands.front().max_z, -2.03);
  EXPECT_EQ(bands.front().point_count, slope_z.size());
}

TEST(EndpointProjectionPolicy, ReportsStackedGroundAsAmbiguous)
{
  const std::vector<double> stacked_z{
    -2.10, -2.08, -2.04, -0.12, -0.10, -0.07};

  const auto bands = findSupportedVerticalSurfaceBands(stacked_z, 0.26, 3U);

  ASSERT_EQ(bands.size(), 2U);
  EXPECT_LT(bands.front().max_z, bands.back().min_z);
}

TEST(EndpointProjectionPolicy, IgnoresSparseVerticalOutliers)
{
  const std::vector<double> noisy_slope_z{
    -2.68, -2.61, -2.55, -2.47, -2.38, 4.50,
    std::numeric_limits<double>::quiet_NaN()};

  const auto bands = findSupportedVerticalSurfaceBands(noisy_slope_z, 0.26, 3U);

  ASSERT_EQ(bands.size(), 1U);
  EXPECT_TRUE(isInsideVerticalSurfaceBand(-2.55, bands.front()));
  EXPECT_FALSE(isInsideVerticalSurfaceBand(4.50, bands.front()));
}

TEST(EndpointProjectionPolicy, RejectsUnsupportedSparseGround)
{
  const std::vector<double> sparse_z{-2.10, -2.08};

  EXPECT_TRUE(findSupportedVerticalSurfaceBands(sparse_z, 0.26, 3U).empty());
}

}  // namespace global_planner
