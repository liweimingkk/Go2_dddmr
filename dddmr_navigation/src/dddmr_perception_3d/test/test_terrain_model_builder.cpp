#include <gtest/gtest.h>

#include "perception_3d/terrain_edge_policy.h"
#include "perception_3d/terrain_model_builder.h"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace perception_3d
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;

std::vector<Eigen::Vector3f> grid(
  std::size_t size,
  float spacing,
  const std::function<float(float, float)> & height)
{
  std::vector<Eigen::Vector3f> points;
  points.reserve(size * size);
  for (std::size_t row = 0; row < size; ++row) {
    for (std::size_t column = 0; column < size; ++column) {
      const float x = static_cast<float>(column) * spacing;
      const float y = static_cast<float>(row) * spacing;
      points.emplace_back(x, y, height(x, y));
    }
  }
  return points;
}

TerrainModelBuilderConfig testConfig()
{
  TerrainModelBuilderConfig config;
  config.normal_radius_m = 0.25F;
  config.min_normal_neighbors = 8U;
  config.max_plane_residual_m = 0.02F;
  config.flat_slope_threshold_rad = 5.0F * kPi / 180.0F;
  config.max_model_slope_rad = 80.0F * kPi / 180.0F;
  config.support_radius_m = 0.16F;
  config.support_plane_tolerance_m = 0.08F;
  config.support_sector_count = 8U;
  config.min_observed_support_ratio = 0.25F;
  config.edge_support_ratio = 0.75F;
  config.surface_connectivity_radius_m = 0.25F;
  config.max_surface_height_delta_m = 0.20F;
  config.max_surface_normal_change_rad = 20.0F * kPi / 180.0F;
  return config;
}

TerrainModelBuildInput inputFor(std::vector<Eigen::Vector3f> points)
{
  TerrainModelBuildInput input;
  input.map_hash = "synthetic-map-sha256";
  input.version = 9U;
  input.stamp_nanoseconds = 1000;
  input.mapground_points = points;
  input.support_points = std::move(points);
  return input;
}

TEST(TerrainModelBuilder, ClassifiesFlatInteriorAndBoundaryEdge)
{
  auto input = inputFor(grid(9U, 0.10F, [](float, float) {return 0.0F;}));
  const auto result = TerrainModelBuilder::build(input, testConfig());
  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_TRUE(result.snapshot->valid());
  ASSERT_EQ(result.snapshot->nodes().size(), input.mapground_points.size());

  const TerrainNode & center = result.snapshot->nodes()[4U * 9U + 4U];
  EXPECT_EQ(center.terrain_class, TerrainClass::FLAT);
  EXPECT_NEAR(center.slope_rad, 0.0F, 1.0e-5F);
  EXPECT_NEAR(center.normal.z(), 1.0F, 1.0e-5F);
  EXPECT_FLOAT_EQ(center.support_ratio, 1.0F);
  EXPECT_GE(center.surface_id, 0);

  const TerrainNode & corner = result.snapshot->nodes().front();
  EXPECT_EQ(corner.terrain_class, TerrainClass::EDGE);
  EXPECT_LT(corner.support_ratio, testConfig().edge_support_ratio);
  EXPECT_GE(corner.surface_id, 0);
  EXPECT_GT(result.statistics.flat_count, 0U);
  EXPECT_GT(result.statistics.edge_count, 0U);
  EXPECT_EQ(result.statistics.surface_count, 1U);
}

TEST(TerrainModelBuilder, EstimatesRampNormalWithoutGrantingCapability)
{
  const float slope = 12.0F * kPi / 180.0F;
  auto input = inputFor(grid(
      9U, 0.10F,
      [slope](float x, float) {return std::tan(slope) * x;}));
  const auto result = TerrainModelBuilder::build(input, testConfig());
  ASSERT_TRUE(result.ok()) << result.error;
  const TerrainNode & center = result.snapshot->nodes()[4U * 9U + 4U];
  EXPECT_EQ(center.terrain_class, TerrainClass::RAMP);
  EXPECT_NEAR(center.slope_rad, slope, 1.0e-3F);
  EXPECT_GT(center.normal.z(), 0.0F);
  EXPECT_EQ(result.statistics.ramp_count + result.statistics.edge_count,
    input.mapground_points.size());

  // TerrainSnapshot contains geometry only; robot capability remains a
  // separate, default-disabled TerrainEdgePolicyConfig.
  EXPECT_FALSE(TerrainEdgePolicyConfig{}.enabled);
}

TEST(TerrainModelBuilder, MarksRoughAndUnobservedTerrainUnknown)
{
  auto rough_points = grid(
    9U, 0.10F,
    [](float x, float y) {
      const int checker =
        static_cast<int>(std::lround(x * 10.0F)) +
        static_cast<int>(std::lround(y * 10.0F));
      return checker % 2 == 0 ? 0.05F : -0.05F;
    });
  auto rough_input = inputFor(rough_points);
  auto config = testConfig();
  config.max_plane_residual_m = 0.01F;
  auto result = TerrainModelBuilder::build(rough_input, config);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(
    result.snapshot->nodes()[4U * 9U + 4U].terrain_class,
    TerrainClass::UNKNOWN);

  auto no_support_input = inputFor(grid(9U, 0.10F, [](float, float) {return 0.0F;}));
  no_support_input.support_points.clear();
  result = TerrainModelBuilder::build(no_support_input, testConfig());
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(
    result.snapshot->nodes()[4U * 9U + 4U].terrain_class,
    TerrainClass::UNKNOWN);
  EXPECT_FLOAT_EQ(result.snapshot->nodes()[4U * 9U + 4U].support_ratio, 0.0F);
}

TEST(TerrainModelBuilder, PreservesInvalidGroundIndexAsUnknown)
{
  auto input = inputFor(grid(9U, 0.10F, [](float, float) {return 0.0F;}));
  const std::size_t invalid_index = 4U * 9U + 4U;
  input.mapground_points[invalid_index].x() = std::numeric_limits<float>::quiet_NaN();
  input.support_points.front().z() = std::numeric_limits<float>::infinity();
  const auto result = TerrainModelBuilder::build(input, testConfig());
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.statistics.invalid_ground_point_count, 1U);
  EXPECT_EQ(result.statistics.invalid_support_point_count, 1U);
  EXPECT_EQ(result.snapshot->nodes()[invalid_index].ground_index, invalid_index);
  EXPECT_EQ(
    result.snapshot->nodes()[invalid_index].terrain_class,
    TerrainClass::UNKNOWN);
}

TEST(TerrainModelBuilder, SeparatesOverlappingElevationLayers)
{
  auto lower = grid(7U, 0.10F, [](float, float) {return 0.0F;});
  auto upper = grid(7U, 0.10F, [](float, float) {return 0.50F;});
  std::vector<Eigen::Vector3f> combined = lower;
  combined.insert(combined.end(), upper.begin(), upper.end());
  auto input = inputFor(combined);
  const auto result = TerrainModelBuilder::build(input, testConfig());
  ASSERT_TRUE(result.ok()) << result.error;
  const std::size_t lower_center = 3U * 7U + 3U;
  const std::size_t upper_center = lower.size() + lower_center;
  EXPECT_EQ(
    result.snapshot->nodes()[lower_center].terrain_class,
    TerrainClass::FLAT);
  EXPECT_EQ(
    result.snapshot->nodes()[upper_center].terrain_class,
    TerrainClass::FLAT);
  EXPECT_NE(
    result.snapshot->nodes()[lower_center].surface_id,
    result.snapshot->nodes()[upper_center].surface_id);
  EXPECT_EQ(result.statistics.surface_count, 2U);
}

TEST(TerrainModelBuilder, IsDeterministicAndScalesBeyondToyClouds)
{
  auto input = inputFor(grid(
      61U, 0.05F,
      [](float x, float) {return std::tan(8.0F * kPi / 180.0F) * x;}));
  auto config = testConfig();
  config.normal_radius_m = 0.16F;
  config.support_radius_m = 0.09F;
  config.surface_connectivity_radius_m = 0.12F;
  const auto first = TerrainModelBuilder::build(input, config);
  const auto second = TerrainModelBuilder::build(input, config);
  ASSERT_TRUE(first.ok()) << first.error;
  ASSERT_TRUE(second.ok()) << second.error;
  ASSERT_EQ(first.snapshot->nodes().size(), 61U * 61U);
  ASSERT_EQ(first.snapshot->nodes().size(), second.snapshot->nodes().size());
  EXPECT_EQ(first.statistics.ramp_count, second.statistics.ramp_count);
  EXPECT_EQ(first.statistics.edge_count, second.statistics.edge_count);
  EXPECT_EQ(first.statistics.surface_count, second.statistics.surface_count);
  for (std::size_t index = 0; index < first.snapshot->nodes().size(); ++index) {
    const auto & first_node = first.snapshot->nodes()[index];
    const auto & second_node = second.snapshot->nodes()[index];
    EXPECT_EQ(first_node.terrain_class, second_node.terrain_class);
    EXPECT_EQ(first_node.surface_id, second_node.surface_id);
    EXPECT_FLOAT_EQ(first_node.slope_rad, second_node.slope_rad);
    EXPECT_FLOAT_EQ(first_node.roughness_m, second_node.roughness_m);
    EXPECT_FLOAT_EQ(first_node.support_ratio, second_node.support_ratio);
    EXPECT_TRUE(first_node.normal.isApprox(second_node.normal, 0.0F));
  }
}

TEST(TerrainModelBuilder, RejectsInvalidIdentityAndConfiguration)
{
  auto input = inputFor(grid(5U, 0.10F, [](float, float) {return 0.0F;}));
  input.version = 0U;
  auto result = TerrainModelBuilder::build(input, testConfig());
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.snapshot, nullptr);

  input.version = 1U;
  auto config = testConfig();
  config.normal_radius_m = 0.0F;
  result = TerrainModelBuilder::build(input, config);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.snapshot, nullptr);
}

}  // namespace
}  // namespace perception_3d
