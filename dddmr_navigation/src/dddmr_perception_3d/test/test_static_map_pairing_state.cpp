#include "perception_3d/static_map_pairing_state.h"

#include <gtest/gtest.h>

namespace perception_3d
{
namespace
{

TEST(StaticMapPairingState, RequiresBothMessagesForEveryCommit)
{
  StaticMapPairingState state;
  EXPECT_FALSE(state.pairReady());
  EXPECT_FALSE(state.commit().committed);

  state.noteMapMessage();
  EXPECT_FALSE(state.pairReady());
  state.noteGroundMessage();
  ASSERT_TRUE(state.pairReady());

  const auto first = state.commit();
  ASSERT_TRUE(first.committed);
  EXPECT_EQ(first.pair_generation, 1U);
  EXPECT_EQ(first.map_message_generation, 1U);
  EXPECT_EQ(first.ground_message_generation, 1U);
  EXPECT_FALSE(state.pairReady());

  state.noteGroundMessage();
  EXPECT_FALSE(state.pairReady());
  state.noteMapMessage();
  const auto second = state.commit();
  ASSERT_TRUE(second.committed);
  EXPECT_EQ(second.pair_generation, 2U);
  EXPECT_EQ(second.map_message_generation, 2U);
  EXPECT_EQ(second.ground_message_generation, 2U);
}

TEST(StaticMapPairingState, SameCardinalityReplacementsStillAdvanceByMessageEpoch)
{
  StaticMapPairingState state;
  // There is deliberately no cloud-size input. Two replacements with the same
  // cardinality are still distinct messages and require a fresh paired commit.
  state.noteMapMessage();
  state.noteGroundMessage();
  ASSERT_TRUE(state.commit().committed);

  state.noteMapMessage();
  state.noteGroundMessage();
  const auto same_size_replacement = state.commit();
  EXPECT_TRUE(same_size_replacement.committed);
  EXPECT_EQ(same_size_replacement.pair_generation, 2U);
}

TEST(StaticMapPairingState, LatestRepeatedSidePairsWithNextOtherSide)
{
  StaticMapPairingState state;
  state.noteMapMessage();
  state.noteMapMessage();
  state.noteGroundMessage();

  const auto commit = state.commit();
  ASSERT_TRUE(commit.committed);
  EXPECT_EQ(commit.map_message_generation, 2U);
  EXPECT_EQ(commit.ground_message_generation, 1U);
}

TEST(StaticMapPairingState, EmptyPayloadsCannotFormAUsablePair)
{
  EXPECT_FALSE(staticMapPayloadPairIsValid(0U, 1U));
  EXPECT_FALSE(staticMapPayloadPairIsValid(1U, 0U));
  EXPECT_FALSE(staticMapPayloadPairIsValid(0U, 0U));
  EXPECT_TRUE(staticMapPayloadPairIsValid(1U, 1U));
}

}  // namespace
}  // namespace perception_3d
