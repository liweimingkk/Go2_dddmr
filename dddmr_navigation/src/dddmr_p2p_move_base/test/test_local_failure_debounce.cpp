#include <gtest/gtest.h>

#include <stdexcept>

#include <p2p_move_base/local_failure_debounce.h>

namespace p2p_move_base
{
namespace
{

TEST(LocalFailureDebounce, RequiresConfiguredConsecutiveFailures)
{
  LocalFailureDebounce debounce;
  debounce.configure(2U);

  EXPECT_FALSE(debounce.recordFailure());
  EXPECT_EQ(debounce.failureCycles(), 1U);
  EXPECT_TRUE(debounce.recordFailure());
  EXPECT_EQ(debounce.failureCycles(), 2U);
}

TEST(LocalFailureDebounce, SafeCycleClearsPendingFailure)
{
  LocalFailureDebounce debounce;
  debounce.configure(2U);

  EXPECT_FALSE(debounce.recordFailure());
  debounce.recordSafeCycle();
  EXPECT_EQ(debounce.failureCycles(), 0U);
  EXPECT_FALSE(debounce.recordFailure());
}

TEST(LocalFailureDebounce, ThreeCycleProfileLeavesTwoStoppedRetryCycles)
{
  LocalFailureDebounce debounce;
  debounce.configure(3U);

  EXPECT_FALSE(debounce.recordFailure());
  EXPECT_FALSE(debounce.recordFailure());
  EXPECT_TRUE(debounce.recordFailure());
  EXPECT_EQ(debounce.failureCycles(), 3U);
}

TEST(LocalFailureDebounce, OneCycleConfigurationEscalatesImmediately)
{
  LocalFailureDebounce debounce;
  debounce.configure(1U);

  EXPECT_TRUE(debounce.recordFailure());
}

TEST(LocalFailureDebounce, FailureCounterSaturatesAtThreshold)
{
  LocalFailureDebounce debounce;
  debounce.configure(2U);

  EXPECT_FALSE(debounce.recordFailure());
  EXPECT_TRUE(debounce.recordFailure());
  EXPECT_TRUE(debounce.recordFailure());
  EXPECT_EQ(debounce.failureCycles(), 2U);
}

TEST(LocalFailureDebounce, RejectsZeroConfirmationCycles)
{
  LocalFailureDebounce debounce;
  EXPECT_THROW(debounce.configure(0U), std::invalid_argument);
}

}  // namespace
}  // namespace p2p_move_base
