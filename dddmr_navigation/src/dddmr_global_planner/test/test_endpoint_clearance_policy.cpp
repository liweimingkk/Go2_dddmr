#include <gtest/gtest.h>

#include "global_planner/endpoint_clearance_policy.h"

namespace global_planner
{
namespace
{

TEST(EndpointClearancePolicy, AcceptsFusedClearanceNormally)
{
  const auto decision = evaluateEndpointClearance(
    true, true, true, 0.30, 0.26, true, 0.40);
  EXPECT_TRUE(decision.traversable);
  EXPECT_FALSE(decision.uses_dynamic_start_exception);
}

TEST(EndpointClearancePolicy, AllowsStaticSafeStartInsideDynamicInflation)
{
  const auto decision = evaluateEndpointClearance(
    true, true, true, 0.08, 0.26, true, 0.60);
  EXPECT_TRUE(decision.traversable);
  EXPECT_TRUE(decision.uses_dynamic_start_exception);
}

TEST(EndpointClearancePolicy, NeverAllowsDynamicInflationExceptionForGoal)
{
  const auto decision = evaluateEndpointClearance(
    true, false, true, 0.08, 0.26, true, 0.60);
  EXPECT_FALSE(decision.traversable);
}

TEST(EndpointClearancePolicy, RejectsStartWhenStaticMapIsAlsoBlocked)
{
  const auto decision = evaluateEndpointClearance(
    true, true, true, 0.08, 0.26, true, 0.12);
  EXPECT_FALSE(decision.traversable);
}

TEST(EndpointClearancePolicy, RejectsStartWhenStaticLayerIsUnavailable)
{
  const auto decision = evaluateEndpointClearance(
    true, true, true, 0.08, 0.26, false, 999.0);
  EXPECT_FALSE(decision.traversable);
}

TEST(EndpointClearancePolicy, DisabledProjectionPreservesLegacySelection)
{
  const auto decision = evaluateEndpointClearance(
    false, false, false, 0.0, 0.26, false, 0.0);
  EXPECT_TRUE(decision.traversable);
  EXPECT_FALSE(decision.uses_dynamic_start_exception);
}

}  // namespace
}  // namespace global_planner
