#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <trajectory_generators/dd_rotate_inplace_theory.h>

namespace trajectory_generators
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

TEST(DDRotateInplaceCollisionHorizon, LegacyPureYawUsesFullRotation)
{
  constexpr double angular_velocity = 0.4;
  constexpr double configured_simulation_time = 2.0;

  const double horizon = DDRotateInplaceTheory::computeCollisionSimulationTime(
    0.0, angular_velocity, configured_simulation_time, true);

  EXPECT_NEAR(horizon, kTwoPi / angular_velocity, 1e-12);
}

TEST(DDRotateInplaceCollisionHorizon, PureYawCanUseConfiguredFiniteHorizon)
{
  constexpr double configured_simulation_time = 1.25;

  const double positive_horizon = DDRotateInplaceTheory::computeCollisionSimulationTime(
    0.0, 0.4, configured_simulation_time, false);
  const double negative_horizon = DDRotateInplaceTheory::computeCollisionSimulationTime(
    0.0, -0.4, configured_simulation_time, false);

  EXPECT_DOUBLE_EQ(positive_horizon, configured_simulation_time);
  EXPECT_DOUBLE_EQ(negative_horizon, configured_simulation_time);
}

TEST(DDRotateInplaceCollisionHorizon, ForwardArcAlwaysUsesConfiguredHorizon)
{
  constexpr double configured_simulation_time = 1.5;

  const double horizon = DDRotateInplaceTheory::computeCollisionSimulationTime(
    0.1, 0.4, configured_simulation_time, true);

  EXPECT_DOUBLE_EQ(horizon, configured_simulation_time);
}

TEST(DDRotateInplaceCollisionHorizon, InvalidInputsFailFast)
{
  EXPECT_THROW(
    DDRotateInplaceTheory::computeCollisionSimulationTime(0.0, 0.4, 0.0, false),
    std::invalid_argument);
  EXPECT_THROW(
    DDRotateInplaceTheory::computeCollisionSimulationTime(
      0.0, 0.0, 2.0, true),
    std::invalid_argument);
  EXPECT_THROW(
    DDRotateInplaceTheory::computeCollisionSimulationTime(
      std::numeric_limits<double>::quiet_NaN(), 0.4, 2.0, true),
    std::invalid_argument);
}

}  // namespace
}  // namespace trajectory_generators
