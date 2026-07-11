#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <vector>

#include <local_planner/in_place_rotation_hysteresis.h>

namespace local_planner
{
namespace
{

using Clock = std::chrono::steady_clock;

Clock::time_point atSeconds(const double seconds)
{
  return Clock::time_point{} +
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

InPlaceRotationCandidate rotation(const double angular_z, const double cost)
{
  InPlaceRotationCandidate candidate;
  candidate.linear_x = 0.0;
  candidate.angular_z = angular_z;
  candidate.cost = cost;
  return candidate;
}

InPlaceRotationCandidate forward(const double linear_x, const double cost)
{
  InPlaceRotationCandidate candidate;
  candidate.linear_x = linear_x;
  candidate.angular_z = 0.0;
  candidate.cost = cost;
  return candidate;
}

TEST(InPlaceRotationHysteresis, HoldsDirectionAcrossAlternatingSmallCostAdvantages)
{
  InPlaceRotationHysteresis policy;
  policy.configure(1.0, 0.05, 2.0);

  std::vector<InPlaceRotationCandidate> candidates{
    rotation(-0.20, 1.01), rotation(0.20, 1.00)};
  EXPECT_EQ(policy.select(candidates, 1U, atSeconds(0.0)), 1U);

  candidates[0].cost = 0.99;
  candidates[1].cost = 1.00;
  EXPECT_EQ(policy.select(candidates, 0U, atSeconds(0.2)), 1U);
  EXPECT_EQ(policy.select(candidates, 0U, atSeconds(1.2)), 1U);
}

TEST(InPlaceRotationHysteresis, MinimumHoldBlocksEarlyMaterialSwitch)
{
  InPlaceRotationHysteresis policy;
  policy.configure(1.0, 0.05, 2.0);

  std::vector<InPlaceRotationCandidate> candidates{
    rotation(-0.20, 1.20), rotation(0.20, 1.00)};
  ASSERT_EQ(policy.select(candidates, 1U, atSeconds(0.0)), 1U);

  candidates[0].cost = 0.80;
  candidates[1].cost = 1.00;
  EXPECT_EQ(policy.select(candidates, 0U, atSeconds(0.5)), 1U);
}

TEST(InPlaceRotationHysteresis, MaterialAdvantageSwitchesAfterHold)
{
  InPlaceRotationHysteresis policy;
  policy.configure(1.0, 0.05, 2.0);

  std::vector<InPlaceRotationCandidate> candidates{
    rotation(-0.20, 1.20), rotation(0.20, 1.00)};
  ASSERT_EQ(policy.select(candidates, 1U, atSeconds(0.0)), 1U);

  candidates[0].cost = 0.80;
  candidates[1].cost = 1.00;
  EXPECT_EQ(policy.select(candidates, 0U, atSeconds(1.1)), 0U);
}

TEST(InPlaceRotationHysteresis, RejectedLockedDirectionIsNeverReused)
{
  InPlaceRotationHysteresis policy;
  policy.configure(5.0, 1.0, 10.0);

  std::vector<InPlaceRotationCandidate> candidates{
    rotation(-0.20, 1.20), rotation(0.20, 1.00)};
  ASSERT_EQ(policy.select(candidates, 1U, atSeconds(0.0)), 1U);

  candidates[0].cost = 0.90;
  candidates[1].cost = -1.0;
  EXPECT_EQ(policy.select(candidates, 0U, atSeconds(0.1)), 0U);
}

TEST(InPlaceRotationHysteresis, ForwardWinnerClearsDirectionLock)
{
  InPlaceRotationHysteresis policy;
  policy.configure(5.0, 1.0, 10.0);

  std::vector<InPlaceRotationCandidate> rotations{
    rotation(-0.20, 1.10), rotation(0.20, 1.00)};
  ASSERT_EQ(policy.select(rotations, 1U, atSeconds(0.0)), 1U);

  std::vector<InPlaceRotationCandidate> forward_candidates{
    rotation(-0.20, 1.00), forward(0.20, 0.50)};
  ASSERT_EQ(policy.select(forward_candidates, 1U, atSeconds(0.1)), 1U);

  rotations[0].cost = 0.99;
  rotations[1].cost = 1.00;
  EXPECT_EQ(policy.select(rotations, 0U, atSeconds(0.2)), 0U);
}

TEST(InPlaceRotationHysteresis, LongGapStartsANewDirectionEpisode)
{
  InPlaceRotationHysteresis policy;
  policy.configure(5.0, 1.0, 2.0);

  std::vector<InPlaceRotationCandidate> candidates{
    rotation(-0.20, 1.10), rotation(0.20, 1.00)};
  ASSERT_EQ(policy.select(candidates, 1U, atSeconds(0.0)), 1U);

  candidates[0].cost = 0.99;
  candidates[1].cost = 1.00;
  EXPECT_EQ(policy.select(candidates, 0U, atSeconds(2.1)), 0U);
}

TEST(InPlaceRotationHysteresis, NoAcceptedPreferredCandidateReturnsNoSelection)
{
  InPlaceRotationHysteresis policy;
  policy.configure(1.0, 0.05, 2.0);

  const std::vector<InPlaceRotationCandidate> candidates{
    rotation(-0.20, -1.0), rotation(0.20, -1.0)};
  EXPECT_EQ(
    policy.select(candidates, candidates.size(), atSeconds(0.0)),
    candidates.size());
}

}  // namespace
}  // namespace local_planner
