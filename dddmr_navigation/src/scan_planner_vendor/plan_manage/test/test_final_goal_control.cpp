#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "plan_manage/final_goal_control.hpp"
#include "plan_manage/replan_distance_policy.hpp"

namespace scan_planner
{
namespace
{

TEST(FinalGoalControl, TracksUntilTrajectoryFinishes)
{
  const auto status = evaluateFinalGoal(
    false, 0.0, true, 1.0, 0.0, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::TRACKING);
}

TEST(FinalGoalControl, RequiresPositionBeforeYawAlignment)
{
  const auto status = evaluateFinalGoal(
    true, 0.16, true, 1.0, 0.0, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::REACQUIRE_POSITION);
}

TEST(FinalGoalControl, AlignsInPlaceWhenOnlyYawIsOutsideTolerance)
{
  const auto status = evaluateFinalGoal(
    true, 0.10, true, 1.0, 0.0, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::ALIGN_YAW);
  EXPECT_DOUBLE_EQ(status.yaw_error, 1.0);
}

TEST(FinalGoalControl, CompletesOnlyWhenBothTolerancesAreMet)
{
  const auto status = evaluateFinalGoal(
    true, 0.15, true, 1.0, 0.86, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::COMPLETE);
  EXPECT_NEAR(status.yaw_error, 0.14, 1e-12);
}

TEST(FinalGoalControl, UsesShortestYawErrorAcrossPiBoundary)
{
  constexpr double kPi = 3.14159265358979323846;
  const auto status = evaluateFinalGoal(
    true, 0.10, true, -kPi + 0.05, kPi - 0.05, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::COMPLETE);
  EXPECT_NEAR(status.yaw_error, 0.10, 1e-12);
}

TEST(FinalGoalControl, KeepsLegacyPositionOnlyGoalsCompatible)
{
  const auto status = evaluateFinalGoal(
    true, 0.10, false, 0.0, 2.0, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::COMPLETE);
}

TEST(FinalGoalControl, RejectsNonfiniteGoalState)
{
  const auto status = evaluateFinalGoal(
    true, 0.10, true, std::numeric_limits<double>::quiet_NaN(),
    0.0, 0.15, 0.15);

  EXPECT_EQ(status.phase, FinalGoalPhase::INVALID);
}

TEST(ReplanDistancePolicy, ReplansAtObservedResidualOutsideFinishTolerance)
{
  constexpr double kObservedResidual = 0.155;
  constexpr double kFinishDistance = 0.15;
  constexpr double kMinReplanDistance = 0.02;

  const auto status = evaluateFinalGoal(
    true, kObservedResidual, false, 0.0, 0.0,
    kFinishDistance, 0.15);

  ASSERT_EQ(status.phase, FinalGoalPhase::REACQUIRE_POSITION);
  ASSERT_TRUE(
    isValidReplanDistanceConfiguration(
      kMinReplanDistance, kFinishDistance));
  EXPECT_FALSE(
    isReplanDistanceBelowMinimum(
      kObservedResidual, kMinReplanDistance));
}

TEST(ReplanDistancePolicy, RejectsOverlappingFinishAndReplanThresholds)
{
  EXPECT_FALSE(isValidReplanDistanceConfiguration(0.15, 0.15));
  EXPECT_FALSE(isValidReplanDistanceConfiguration(0.20, 0.15));
  EXPECT_TRUE(isReplanDistanceBelowMinimum(0.019, 0.02));
}

}  // namespace
}  // namespace scan_planner
