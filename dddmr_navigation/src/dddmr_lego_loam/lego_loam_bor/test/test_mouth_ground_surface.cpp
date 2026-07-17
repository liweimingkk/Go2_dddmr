#include "mouth_ground_surface.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace
{

using lego_loam_bor::MouthGroundSurfaceConfig;

void addPlane(
  pcl::PointCloud<pcl::PointXYZI> & cloud, double x_min, double x_max,
  double y_min, double y_max, double spacing, double z, double slope = 0.0)
{
  for (double x = x_min; x <= x_max + spacing * 0.5; x += spacing) {
    for (double y = y_min; y <= y_max + spacing * 0.5; y += spacing) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z + std::tan(slope) * x);
      cloud.push_back(point);
    }
  }
}

void addRiser(
  pcl::PointCloud<pcl::PointXYZI> & cloud, double x, double y_min,
  double y_max, double z_min, double z_max, double spacing)
{
  for (double y = y_min; y <= y_max + spacing * 0.5; y += spacing) {
    for (double z = z_min; z <= z_max + spacing * 0.5; z += spacing) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      cloud.push_back(point);
    }
  }
}

double acceptedFraction(
  const std::vector<std::uint8_t> & mask, std::size_t begin, std::size_t end)
{
  std::size_t accepted = 0U;
  for (std::size_t index = begin; index < end; ++index) {
    accepted += mask[index] != 0U ? 1U : 0U;
  }
  return static_cast<double>(accepted) / static_cast<double>(end - begin);
}

TEST(MouthGroundSurface, AcceptsVariableHeightTreadsAndRejectsRiser)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.30);
  const std::size_t lower_end = cloud.size();
  addPlane(cloud, 0.78, 1.28, -0.40, 0.40, 0.04, -0.12);
  const std::size_t middle_end = cloud.size();
  addPlane(cloud, 1.36, 1.86, -0.40, 0.40, 0.04, 0.06);
  const std::size_t upper_end = cloud.size();
  addRiser(cloud, 0.74, -0.40, 0.40, -0.30, -0.12, 0.04);
  addRiser(cloud, 1.32, -0.40, 0.40, -0.12, 0.06, 0.04);
  const std::size_t riser_end = cloud.size();

  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(acceptedFraction(mask, 0U, lower_end), 0.80);
  EXPECT_GT(acceptedFraction(mask, lower_end, middle_end), 0.80);
  EXPECT_GT(acceptedFraction(mask, middle_end, upper_end), 0.80);
  EXPECT_LT(acceptedFraction(mask, upper_end, riser_end), 0.20);
}

TEST(MouthGroundSurface, AcceptsUnequalRisesWithinCapabilityEnvelope)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.34);
  const std::size_t lower_end = cloud.size();
  addPlane(cloud, 0.78, 1.28, -0.40, 0.40, 0.04, -0.20);
  const std::size_t middle_end = cloud.size();
  addPlane(cloud, 1.36, 1.86, -0.40, 0.40, 0.04, 0.02);

  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(acceptedFraction(mask, 0U, lower_end), 0.80);
  EXPECT_GT(acceptedFraction(mask, lower_end, middle_end), 0.80);
  EXPECT_GT(acceptedFraction(mask, middle_end, cloud.size()), 0.80);
}

TEST(MouthGroundSurface, UsesSlopeEnvelopeInsteadOfAHeightConstant)
{
  constexpr double kPi = 3.14159265358979323846;
  MouthGroundSurfaceConfig config;
  config.maximum_slope = 30.0 * kPi / 180.0;

  pcl::PointCloud<pcl::PointXYZI> gentle;
  addPlane(gentle, 0.20, 0.90, -0.40, 0.40, 0.04, -0.40, 20.0 * kPi / 180.0);
  const auto gentle_mask = lego_loam_bor::classifyMouthGroundSurface(gentle, config);
  EXPECT_GT(acceptedFraction(gentle_mask, 0U, gentle.size()), 0.80);

  pcl::PointCloud<pcl::PointXYZI> steep;
  addPlane(steep, 0.20, 0.90, -0.40, 0.40, 0.04, -0.50, 40.0 * kPi / 180.0);
  const auto steep_mask = lego_loam_bor::classifyMouthGroundSurface(steep, config);
  EXPECT_LT(acceptedFraction(steep_mask, 0U, steep.size()), 0.20);
}

TEST(MouthGroundSurface, RejectsUnsupportedHorizontalPlatform)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.65, -0.40, 0.40, 0.04, -0.30);
  const std::size_t floor_end = cloud.size();
  addPlane(cloud, 0.85, 1.30, -0.30, 0.30, 0.04, 0.20);

  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(acceptedFraction(mask, 0U, floor_end), 0.80);
  EXPECT_LT(acceptedFraction(mask, floor_end, cloud.size()), 0.05);
}

TEST(MouthGroundSurface, RejectsSingleLowBoxInsideSupportFootprint)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.30);
  const std::size_t floor_end = cloud.size();
  // This top is only 18 cm above the floor and sits directly in the support
  // footprint. The support-height anchor and chain check must keep it out.
  addPlane(cloud, 0.35, 0.65, -0.28, 0.28, 0.04, -0.12);

  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(acceptedFraction(mask, 0U, floor_end), 0.80);
  EXPECT_LT(acceptedFraction(mask, floor_end, cloud.size()), 0.05);
}

TEST(MouthGroundSurface, UsesExistingGroundAsTheSupportSeedReference)
{
  MouthGroundSurfaceConfig config;
  // Deliberately widen the geometric band so the reference cloud, rather than
  // the Z limit, is what prevents the raised top from becoming a seed.
  config.support_seed_z_max = -0.10;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.30);
  const std::size_t floor_end = cloud.size();
  addPlane(cloud, 0.35, 0.65, -0.28, 0.28, 0.04, -0.12);

  pcl::PointCloud<pcl::PointXYZI> reference;
  addPlane(reference, 0.20, 0.70, -0.40, 0.40, 0.08, -0.30);
  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, config, &reference);
  EXPECT_GT(acceptedFraction(mask, 0U, floor_end), 0.80);
  EXPECT_LT(acceptedFraction(mask, floor_end, cloud.size()), 0.05);
}

TEST(MouthGroundSurface, EmptySupportReferenceFailsClosed)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.30);
  pcl::PointCloud<pcl::PointXYZI> empty_reference;

  const auto result = lego_loam_bor::classifyMouthGroundSurfaceDetailed(
    cloud, MouthGroundSurfaceConfig{}, &empty_reference);
  EXPECT_EQ(
    acceptedFraction(result.supported_ground, 0U, cloud.size()), 0.0);
  EXPECT_EQ(
    acceptedFraction(result.verified_obstacle, 0U, cloud.size()), 0.0);
}

TEST(MouthGroundSurface, RejectsSeedHeightPlatformOutsideSupportFootprint)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 1.00, 1.50, -0.35, 0.35, 0.04, -0.25);

  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_EQ(acceptedFraction(mask, 0U, cloud.size()), 0.0);
}

TEST(MouthGroundSurface, RejectsNarrowPatchInsideOtherwiseValidStairChain)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.30);
  const std::size_t floor_end = cloud.size();
  addPlane(cloud, 0.78, 1.28, -0.04, 0.04, 0.02, -0.12);
  const std::size_t narrow_end = cloud.size();
  addPlane(cloud, 1.36, 1.86, -0.40, 0.40, 0.04, 0.06);

  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(acceptedFraction(mask, 0U, floor_end), 0.80);
  EXPECT_LT(acceptedFraction(mask, floor_end, narrow_end), 0.05);
  EXPECT_LT(acceptedFraction(mask, narrow_end, cloud.size()), 0.05);
}

TEST(MouthGroundSurface, RejectsLineLikeReturns)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (double x = 0.20; x <= 1.20; x += 0.02) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(x);
    point.y = 0.0F;
    point.z = 0.0F;
    cloud.push_back(point);
  }
  const auto mask = lego_loam_bor::classifyMouthGroundSurface(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_EQ(acceptedFraction(mask, 0U, cloud.size()), 0.0);
}

TEST(MouthGroundSurface, KeepsUnsupportedHorizontalPlatformUnknown)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.65, -0.40, 0.40, 0.04, -0.30);
  const std::size_t floor_end = cloud.size();
  addPlane(cloud, 0.90, 1.35, -0.30, 0.30, 0.04, 0.20);

  const auto result = lego_loam_bor::classifyMouthGroundSurfaceDetailed(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(acceptedFraction(result.supported_ground, 0U, floor_end), 0.80);
  EXPECT_LT(
    acceptedFraction(result.supported_ground, floor_end, cloud.size()), 0.05);
  EXPECT_LT(
    acceptedFraction(result.verified_obstacle, floor_end, cloud.size()), 0.05);
}

TEST(MouthGroundSurface, MarksPlanarRiserAsVerifiedObstacle)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  addPlane(cloud, 0.20, 0.70, -0.40, 0.40, 0.04, -0.30);
  const std::size_t floor_end = cloud.size();
  addRiser(cloud, 0.78, -0.40, 0.40, -0.30, 0.10, 0.04);

  const auto result = lego_loam_bor::classifyMouthGroundSurfaceDetailed(
    cloud, MouthGroundSurfaceConfig{});
  EXPECT_GT(
    acceptedFraction(result.verified_obstacle, floor_end, cloud.size()), 0.60);
  EXPECT_LT(
    acceptedFraction(result.supported_ground, floor_end, cloud.size()), 0.10);
}

}  // namespace
