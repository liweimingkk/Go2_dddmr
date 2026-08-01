#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include <p2p_move_base/goal_execution_gate.h>

namespace p2p_move_base
{
namespace
{

using namespace std::chrono_literals;

TEST(GoalExecutionGate, StartsReplacementOnlyAfterCurrentExecutionFinishes)
{
  GoalExecutionGate gate;
  const auto first = gate.acceptLatest();
  ASSERT_TRUE(gate.waitToStart(first));

  const auto replacement = gate.acceptLatest();
  auto replacement_start = std::async(
    std::launch::async,
    [&gate, replacement]() {return gate.waitToStart(replacement);});

  EXPECT_EQ(replacement_start.wait_for(20ms), std::future_status::timeout);
  gate.finish(first);
  ASSERT_EQ(replacement_start.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(replacement_start.get());
  gate.finish(replacement);
}

TEST(GoalExecutionGate, DropsQueuedGoalSupersededByNewerGoal)
{
  GoalExecutionGate gate;
  const auto first = gate.acceptLatest();
  ASSERT_TRUE(gate.waitToStart(first));

  const auto stale = gate.acceptLatest();
  auto stale_start = std::async(
    std::launch::async,
    [&gate, stale]() {return gate.waitToStart(stale);});
  EXPECT_EQ(stale_start.wait_for(20ms), std::future_status::timeout);

  const auto latest = gate.acceptLatest();
  gate.finish(first);
  ASSERT_EQ(stale_start.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(stale_start.get());
  EXPECT_TRUE(gate.waitToStart(latest));
  gate.finish(latest);
}

}  // namespace
}  // namespace p2p_move_base
