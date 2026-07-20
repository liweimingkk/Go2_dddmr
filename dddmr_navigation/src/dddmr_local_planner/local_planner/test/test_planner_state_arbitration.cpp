#include <gtest/gtest.h>

#include "planner_state_arbitration.h"

namespace local_planner
{
namespace
{

TEST(PlannerStateArbitration, SafeTrajectoryWinsOverBlockedReferencePath)
{
  EXPECT_EQ(
    arbitratePlannerState(2.5, true, false),
    dddmr_sys_core::TRAJECTORY_FOUND);
  EXPECT_EQ(
    arbitratePlannerState(2.5, false, true),
    dddmr_sys_core::TRAJECTORY_FOUND);
}

TEST(PlannerStateArbitration, BlockedPathIsFallbackWhenEveryTrajectoryFails)
{
  EXPECT_EQ(
    arbitratePlannerState(-1.0, true, false),
    dddmr_sys_core::PATH_BLOCKED_WAIT);
  EXPECT_EQ(
    arbitratePlannerState(-1.0, false, true),
    dddmr_sys_core::PATH_BLOCKED_REPLANNING);
}

TEST(PlannerStateArbitration, FailedTrajectoriesWithoutBlockedPathReportFailure)
{
  EXPECT_EQ(
    arbitratePlannerState(-1.0, false, false),
    dddmr_sys_core::ALL_TRAJECTORIES_FAIL);
}

}  // namespace
}  // namespace local_planner
