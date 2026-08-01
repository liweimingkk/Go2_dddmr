#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <mcl_3dl/particle_spread.h>
#include <mcl_3dl/residual_metrics.h>

namespace mcl_3dl
{
namespace
{

TEST(PositionSpreadAccumulator, HorizontalSurfaceMatchesLegacyWorldAxes)
{
  PositionSpreadAccumulator accumulator(Vec3(0.0, 0.0, 1.0));
  accumulator.add(Vec3(0.3, 0.4, 0.2), 1.0);
  accumulator.add(Vec3(-0.3, -0.4, -0.2), 1.0);

  const PositionSpread spread = accumulator.spread();
  EXPECT_DOUBLE_EQ(spread.tangent, spread.raw_xy);
  EXPECT_DOUBLE_EQ(spread.normal, spread.raw_z);
  EXPECT_NEAR(spread.tangent, 0.5, 1e-7);
  EXPECT_NEAR(spread.normal, 0.2, 1e-7);
}

TEST(PositionSpreadAccumulator, RampElevationIsTangentUncertainty)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double slope_angle = 20.0 * pi / 180.0;
  const Vec3 surface_normal(
      -std::sin(slope_angle), 0.0, std::cos(slope_angle));
  const Vec3 uphill_tangent(
      std::cos(slope_angle), 0.0, std::sin(slope_angle));
  PositionSpreadAccumulator accumulator(surface_normal);
  accumulator.add(uphill_tangent * 0.4f, 1.0);
  accumulator.add(uphill_tangent * -0.4f, 1.0);

  const PositionSpread spread = accumulator.spread();
  EXPECT_NEAR(spread.tangent, 0.4, 1e-6);
  EXPECT_NEAR(spread.normal, 0.0, 1e-6);
  EXPECT_GT(spread.raw_z, 0.13);
}

TEST(PositionSpreadAccumulator, RampNormalErrorRemainsVisible)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double slope_angle = 20.0 * pi / 180.0;
  const Vec3 surface_normal(
      -std::sin(slope_angle), 0.0, std::cos(slope_angle));
  PositionSpreadAccumulator accumulator(surface_normal);
  accumulator.add(surface_normal * 0.06f, 1.0);
  accumulator.add(surface_normal * -0.06f, 1.0);

  const PositionSpread spread = accumulator.spread();
  EXPECT_NEAR(spread.tangent, 0.0, 2e-6);
  EXPECT_NEAR(spread.normal, 0.06, 1e-6);
}

TEST(PositionSpreadAccumulator, InvalidNormalFallsBackToWorldAxes)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const Vec3 invalid_normal : {Vec3(), Vec3(nan, 0.0, 1.0)})
  {
    PositionSpreadAccumulator accumulator(invalid_normal);
    accumulator.add(Vec3(0.3, 0.4, 0.2), 1.0);

    const PositionSpread spread = accumulator.spread();
    EXPECT_EQ(accumulator.normal(), Vec3(0.0, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(spread.tangent, spread.raw_xy);
    EXPECT_DOUBLE_EQ(spread.normal, spread.raw_z);
  }
}

TEST(SlopeCompensation, ActivatesOnlyForConfirmedSlope)
{
  constexpr double minimum_tilt = 0.05;
  const Vec3 horizontal(0.0, 0.0, 1.0);
  const Vec3 below_threshold(-std::sin(0.04), 0.0, std::cos(0.04));
  const Vec3 ramp(-std::sin(0.20), 0.0, std::cos(0.20));

  EXPECT_FALSE(slopeCompensationActive(
      true, true, true, true, horizontal, minimum_tilt));
  EXPECT_FALSE(slopeCompensationActive(
      true, true, true, true, below_threshold, minimum_tilt));
  EXPECT_TRUE(slopeCompensationActive(
      true, true, true, true, ramp, minimum_tilt));
  EXPECT_FALSE(slopeCompensationActive(
      false, true, true, true, ramp, minimum_tilt));
  EXPECT_FALSE(slopeCompensationActive(
      true, true, false, true, ramp, minimum_tilt));
  EXPECT_FALSE(slopeCompensationActive(
      true, true, true, false, ramp, minimum_tilt));
}

TEST(SlopeCompensation, InvalidNormalCannotActivate)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(std::isinf(surfaceTiltFromUp(Vec3())));
  EXPECT_TRUE(std::isinf(surfaceTiltFromUp(Vec3(nan, 0.0, 1.0))));
  EXPECT_FALSE(slopeCompensationActive(
      true, true, true, true, Vec3(), 0.0));
  EXPECT_FALSE(slopeCompensationActive(
      true, true, true, true, Vec3(nan, 0.0, 1.0), 0.0));
}

TEST(PositionSpreadAccumulator, RejectsInvalidSamplesConservatively)
{
  PositionSpreadAccumulator accumulator(Vec3(0.0, 0.0, 1.0));
  accumulator.add(Vec3(0.1, 0.0, 0.0), 0.0);
  accumulator.add(
      Vec3(std::numeric_limits<float>::quiet_NaN(), 0.0, 0.0), 1.0);

  const PositionSpread spread = accumulator.spread();
  EXPECT_TRUE(std::isinf(spread.tangent));
  EXPECT_TRUE(std::isinf(spread.normal));
}

TEST(ResidualAccumulator, SeparatesCoverageFromMatchedGeometry)
{
  ResidualAccumulator high_coverage(0.30);
  ResidualAccumulator low_coverage(0.30);
  for (int i = 0; i < 8; ++i)
  {
    high_coverage.add(0.12, true);
  }
  for (int i = 0; i < 2; ++i)
  {
    high_coverage.add(0.30, false);
  }
  for (int i = 0; i < 3; ++i)
  {
    low_coverage.add(0.12, true);
  }
  for (int i = 0; i < 7; ++i)
  {
    low_coverage.add(0.30, false);
  }

  const ResidualMetrics high = high_coverage.metrics();
  const ResidualMetrics low = low_coverage.metrics();
  EXPECT_NEAR(high.match_ratio, 0.8, 1e-12);
  EXPECT_NEAR(low.match_ratio, 0.3, 1e-12);
  EXPECT_NEAR(high.matched, 0.12, 1e-12);
  EXPECT_NEAR(low.matched, 0.12, 1e-12);
  EXPECT_NEAR(high.capped, 0.156, 1e-12);
  EXPECT_NEAR(low.capped, 0.246, 1e-12);
}

TEST(ResidualAccumulator, NoMatchesRemainFailClosed)
{
  ResidualAccumulator accumulator(0.30);
  accumulator.add(0.30, false);
  accumulator.add(std::numeric_limits<double>::quiet_NaN(), true);

  const ResidualMetrics residual = accumulator.metrics();
  EXPECT_DOUBLE_EQ(residual.match_ratio, 0.0);
  EXPECT_DOUBLE_EQ(residual.capped, 0.30);
  EXPECT_TRUE(std::isinf(residual.matched));
}

}  // namespace
}  // namespace mcl_3dl
