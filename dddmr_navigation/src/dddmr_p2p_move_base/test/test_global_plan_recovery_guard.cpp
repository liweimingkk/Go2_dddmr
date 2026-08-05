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
  EXPECT_EQ(
    guard.timeoutAction(true, false, 0, 10),
    PlanningTimeoutAction::ABORT_NO_VALID_PLAN);
}

TEST(GlobalPlanRecoveryGuard, AllowsRecoveryAfterValidPlan)
{
  GlobalPlanRecoveryGuard guard;

  guard.recordValidPlan();

  EXPECT_TRUE(guard.shouldAttemptRecovery());
  EXPECT_EQ(
    guard.timeoutAction(false, true, 0, 10),
    PlanningTimeoutAction::ATTEMPT_RECOVERY);
}

TEST(GlobalPlanRecoveryGuard, HoldsStoppedAfterPreviouslyValidPlan)
{
  GlobalPlanRecoveryGuard guard;
  guard.recordValidPlan();

  EXPECT_EQ(
    guard.timeoutAction(true, false, 0, 10),
    PlanningTimeoutAction::HOLD_AND_REPLAN);
  EXPECT_EQ(
    guard.timeoutAction(true, false, 9, 10),
    PlanningTimeoutAction::HOLD_AND_REPLAN);
}

TEST(GlobalPlanRecoveryGuard, AbortsWhenStoppedRetriesAreExhausted)
{
  GlobalPlanRecoveryGuard guard;
  guard.recordValidPlan();

  EXPECT_EQ(
    guard.timeoutAction(true, false, 10, 10),
    PlanningTimeoutAction::ABORT_RETRIES_EXHAUSTED);
  EXPECT_EQ(
    guard.timeoutAction(true, false, 0, 0),
    PlanningTimeoutAction::ABORT_RETRIES_EXHAUSTED);
}

TEST(GlobalPlanRecoveryGuard, AbortsWhenAllRecoveryIsDisabled)
{
  GlobalPlanRecoveryGuard guard;
  guard.recordValidPlan();

  EXPECT_EQ(
    guard.timeoutAction(false, false, 0, 10),
    PlanningTimeoutAction::ABORT_RETRIES_EXHAUSTED);
}

TEST(GlobalPlanRecoveryGuard, ResetAppliesToNextGoal)
{
  GlobalPlanRecoveryGuard guard;
  guard.recordValidPlan();

  guard.reset();

  EXPECT_FALSE(guard.shouldAttemptRecovery());
  EXPECT_EQ(
    guard.timeoutAction(true, false, 0, 10),
    PlanningTimeoutAction::ABORT_NO_VALID_PLAN);
}

}  // namespace
}  // namespace p2p_move_base
