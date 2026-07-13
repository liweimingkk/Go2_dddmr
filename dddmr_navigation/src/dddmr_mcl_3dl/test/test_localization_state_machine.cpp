#include <gtest/gtest.h>

#include <mcl_3dl/localization_state_machine.h>

namespace mcl_3dl
{
namespace
{

LocalizationStateConfig testConfig()
{
  LocalizationStateConfig config;
  config.tracking_match_ratio = 0.5;
  config.lost_match_ratio = 0.2;
  config.tracking_max_xy_std = 0.5;
  config.tracking_max_yaw_std = 0.25;
  config.lost_max_xy_std = 1.0;
  config.lost_max_yaw_std = 0.5;
  config.tracking_good_frames = 3;
  config.lost_bad_frames = 2;
  return config;
}

TEST(LocalizationStateMachine, RequiresConsecutiveConvergedObservations)
{
  LocalizationStateMachine machine(testConfig());
  EXPECT_EQ(machine.state(), LocalizationState::UNINITIALIZED);
  EXPECT_TRUE(machine.startLocalizing());

  const LocalizationObservation good{0.7, 0.2, 0.1, true};
  EXPECT_FALSE(machine.observe(good));
  EXPECT_FALSE(machine.observe(good));
  EXPECT_EQ(machine.state(), LocalizationState::LOCALIZING);
  EXPECT_TRUE(machine.observe(good));
  EXPECT_EQ(machine.state(), LocalizationState::TRACKING);
}

TEST(LocalizationStateMachine, ResetsGoodCountOnAmbiguousObservation)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();

  EXPECT_FALSE(machine.observe({0.7, 0.2, 0.1, true}));
  EXPECT_EQ(machine.goodFrames(), 1U);
  EXPECT_FALSE(machine.observe({0.7, 0.2, 0.1, false}));
  EXPECT_EQ(machine.goodFrames(), 0U);
}

TEST(LocalizationStateMachine, DeclaresLostAfterConsecutiveBadFrames)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();
  const LocalizationObservation good{0.7, 0.2, 0.1, true};
  machine.observe(good);
  machine.observe(good);
  machine.observe(good);
  ASSERT_EQ(machine.state(), LocalizationState::TRACKING);

  EXPECT_FALSE(machine.observe({0.1, 0.2, 0.1, true}));
  EXPECT_EQ(machine.state(), LocalizationState::TRACKING);
  EXPECT_TRUE(machine.observe({0.1, 0.2, 0.1, true}));
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
}

TEST(LocalizationStateMachine, SupportsExplicitResetAndRecovery)
{
  LocalizationStateMachine machine(testConfig());
  EXPECT_TRUE(machine.markLost());
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
  EXPECT_TRUE(machine.startLocalizing());
  EXPECT_EQ(machine.state(), LocalizationState::LOCALIZING);
  EXPECT_TRUE(machine.reset());
  EXPECT_EQ(machine.state(), LocalizationState::UNINITIALIZED);
}

}  // namespace
}  // namespace mcl_3dl
