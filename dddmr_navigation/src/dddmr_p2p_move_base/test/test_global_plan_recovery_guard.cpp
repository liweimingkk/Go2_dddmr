#include <gtest/gtest.h>

#include "p2p_move_base/global_plan_recovery_guard.h"

namespace p2p_move_base
{
namespace
{

TEST(GlobalPlanRecoveryGuard, SuppressesRecoveryBeforeFirstValidPlan)
{
  GlobalPlanRecoveryGuard guard;

  EXPECT_FALSE(guard.shouldAttemptRecovery());
}

TEST(GlobalPlanRecoveryGuard, AllowsRecoveryAfterValidPlan)
{
  GlobalPlanRecoveryGuard guard;

  guard.recordValidPlan();

  EXPECT_TRUE(guard.shouldAttemptRecovery());
}

TEST(GlobalPlanRecoveryGuard, ResetAppliesToNextGoal)
{
  GlobalPlanRecoveryGuard guard;
  guard.recordValidPlan();

  guard.reset();

  EXPECT_FALSE(guard.shouldAttemptRecovery());
}

}  // namespace
}  // namespace p2p_move_base
