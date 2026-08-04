#include <gtest/gtest.h>

#include <limits>

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
  config.tracking_max_z_std = 0.10;
  config.tracking_max_roll_std = 0.05;
  config.tracking_max_pitch_std = 0.05;
  config.tracking_max_yaw_std = 0.25;
  config.tracking_max_residual = 0.15;
  config.tracking_max_map_odom_tilt = 0.03;
  config.tracking_max_ground_normal_error = 0.10;
  config.tracking_max_base_height_error = 0.05;
  config.tracking_max_pose_height_error = 0.03;
  config.lost_max_xy_std = 1.0;
  config.lost_max_z_std = 0.20;
  config.lost_max_roll_std = 0.10;
  config.lost_max_pitch_std = 0.10;
  config.lost_max_yaw_std = 0.5;
  config.lost_max_residual = 0.25;
  config.lost_max_map_odom_tilt = 0.08;
  config.lost_max_ground_normal_error = 0.20;
  config.lost_max_base_height_error = 0.10;
  config.lost_max_pose_height_error = 0.08;
  config.require_ground_health = true;
  config.tracking_good_frames = 3;
  config.lost_bad_frames = 2;
  return config;
}

LocalizationObservation goodObservation()
{
  LocalizationObservation observation;
  observation.match_ratio = 0.7;
  observation.xy_std = 0.2;
  observation.z_std = 0.02;
  observation.roll_std = 0.01;
  observation.pitch_std = 0.01;
  observation.yaw_std = 0.1;
  observation.residual = 0.08;
  observation.map_odom_tilt = 0.01;
  observation.ground_normal_error = 0.04;
  observation.base_height_error = 0.02;
  observation.pose_height_error = 0.01;
  observation.ground_valid = true;
  observation.particle_count_converged = true;
  return observation;
}

TEST(LocalizationStateMachine, RequiresConsecutiveConvergedObservations)
{
  LocalizationStateMachine machine(testConfig());
  EXPECT_EQ(machine.state(), LocalizationState::UNINITIALIZED);
  EXPECT_TRUE(machine.startLocalizing());

  const LocalizationObservation good = goodObservation();
  EXPECT_FALSE(machine.observe(good));
  EXPECT_FALSE(machine.observe(good));
  EXPECT_EQ(machine.state(), LocalizationState::LOCALIZING);
  EXPECT_TRUE(machine.observe(good));
  EXPECT_EQ(machine.state(), LocalizationState::TRACKING);
}

TEST(LocalizationStateMachine, UsesSlopeSpecificNormalSpreadLimits)
{
  LocalizationStateConfig config = testConfig();
  config.tracking_good_frames = 1;
  config.lost_bad_frames = 1;
  config.tracking_max_z_std = 0.08;
  config.tracking_max_slope_normal_std = 0.12;
  config.lost_max_z_std = 0.10;
  config.lost_max_slope_normal_std = 0.15;
  LocalizationStateMachine machine(config);
  machine.startLocalizing();

  LocalizationObservation observation = goodObservation();
  observation.z_std = 0.10;
  EXPECT_FALSE(machine.observe(observation));
  EXPECT_EQ(machine.state(), LocalizationState::LOCALIZING);

  observation.slope_compensated = true;
  EXPECT_TRUE(machine.observe(observation));
  ASSERT_EQ(machine.state(), LocalizationState::TRACKING);

  observation.z_std = 0.14;
  EXPECT_FALSE(machine.observe(observation));
  EXPECT_EQ(machine.state(), LocalizationState::TRACKING);

  observation.slope_compensated = false;
  EXPECT_TRUE(machine.observe(observation));
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
}

TEST(LocalizationStateMachine, RequiresOperatorInitializationWhenConfigured)
{
  LocalizationStateConfig config = testConfig();
  config.tracking_good_frames = 1;
  config.require_operator_initialization = true;
  LocalizationStateMachine machine(config);
  machine.startLocalizing();

  LocalizationObservation observation = goodObservation();
  EXPECT_FALSE(machine.observe(observation));
  EXPECT_EQ(machine.state(), LocalizationState::LOCALIZING);

  observation.operator_initialization_confirmed = true;
  EXPECT_TRUE(machine.observe(observation));
  EXPECT_EQ(machine.state(), LocalizationState::TRACKING);
}

TEST(LocalizationStateMachine, ResetsGoodCountOnAmbiguousObservation)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();

  EXPECT_FALSE(machine.observe(goodObservation()));
  EXPECT_EQ(machine.goodFrames(), 1U);
  LocalizationObservation ambiguous = goodObservation();
  ambiguous.particle_count_converged = false;
  EXPECT_FALSE(machine.observe(ambiguous));
  EXPECT_EQ(machine.goodFrames(), 0U);
}

TEST(LocalizationStateMachine, DeclaresLostAfterConsecutiveBadFrames)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();
  const LocalizationObservation good = goodObservation();
  machine.observe(good);
  machine.observe(good);
  machine.observe(good);
  ASSERT_EQ(machine.state(), LocalizationState::TRACKING);

  LocalizationObservation bad = good;
  bad.match_ratio = 0.1;
  EXPECT_FALSE(machine.observe(bad));
  EXPECT_EQ(machine.state(), LocalizationState::TRACKING);
  EXPECT_TRUE(machine.observe(bad));
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
}

TEST(LocalizationStateMachine, RejectsGeometricallyInvalidTracking)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();

  LocalizationObservation tilted = goodObservation();
  tilted.map_odom_tilt = 0.04;
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FALSE(machine.observe(tilted));
  }
  EXPECT_EQ(machine.state(), LocalizationState::LOCALIZING);

  LocalizationObservation wrong_height = goodObservation();
  wrong_height.base_height_error = 0.06;
  EXPECT_FALSE(machine.observe(wrong_height));
  EXPECT_EQ(machine.goodFrames(), 0U);
}

TEST(LocalizationStateMachine, DeclaresLostOnFinalResidualOrGroundFailure)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();
  const LocalizationObservation good = goodObservation();
  machine.observe(good);
  machine.observe(good);
  machine.observe(good);
  ASSERT_EQ(machine.state(), LocalizationState::TRACKING);

  LocalizationObservation bad = good;
  bad.residual = 0.30;
  EXPECT_FALSE(machine.observe(bad));
  bad = good;
  bad.ground_valid = false;
  EXPECT_TRUE(machine.observe(bad));
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
}

TEST(LocalizationStateMachine, DeclaresLostOnNonFiniteGeometry)
{
  LocalizationStateMachine machine(testConfig());
  machine.startLocalizing();
  const LocalizationObservation good = goodObservation();
  machine.observe(good);
  machine.observe(good);
  machine.observe(good);
  ASSERT_EQ(machine.state(), LocalizationState::TRACKING);

  LocalizationObservation invalid = good;
  invalid.map_odom_tilt = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(machine.observe(invalid));
  EXPECT_TRUE(machine.observe(invalid));
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

TEST(LocalizationConvergenceTimer, StartsOnlyAfterCompletedMeasurement)
{
  LocalizationConvergenceTimer timer;
  constexpr int64_t second = 1000000000LL;

  EXPECT_EQ(timer.startedNs(), 0);
  const auto waiting = timer.snapshot();
  EXPECT_FALSE(LocalizationConvergenceTimer::expired(
    waiting, 100 * second, 8.0));

  const uint64_t generation = timer.generation();
  EXPECT_TRUE(timer.armAfterCompletedMeasurement(100 * second));
  EXPECT_TRUE(timer.isGenerationCurrent(generation));
  EXPECT_EQ(timer.startedNs(), 100 * second);
  EXPECT_FALSE(timer.armAfterCompletedMeasurement(101 * second));
  const auto armed = timer.snapshot();
  EXPECT_FALSE(LocalizationConvergenceTimer::expired(
    armed, 108 * second, 8.0));
  EXPECT_TRUE(LocalizationConvergenceTimer::expired(
    armed, 108 * second + 1, 8.0));
}

TEST(LocalizationConvergenceTimer, ResetDefersAReplacementSeed)
{
  LocalizationConvergenceTimer timer;
  constexpr int64_t second = 1000000000LL;

  ASSERT_TRUE(timer.armAfterCompletedMeasurement(10 * second));
  const auto old_attempt = timer.snapshot();
  EXPECT_TRUE(LocalizationConvergenceTimer::expired(
    old_attempt, 19 * second, 8.0));

  timer.reset();
  EXPECT_EQ(timer.startedNs(), 0);
  EXPECT_FALSE(timer.isCurrent(old_attempt));
  EXPECT_FALSE(timer.isGenerationCurrent(old_attempt.generation));
  const auto waiting = timer.snapshot();
  EXPECT_FALSE(LocalizationConvergenceTimer::expired(
    waiting, 100 * second, 8.0));
  EXPECT_TRUE(timer.armAfterCompletedMeasurement(100 * second));
  const auto replacement_attempt = timer.snapshot();
  EXPECT_TRUE(timer.isCurrent(replacement_attempt));
  EXPECT_FALSE(LocalizationConvergenceTimer::expired(
    replacement_attempt, 101 * second, 8.0));
}

TEST(LocalizationAttemptKind, SeparatesTimeoutAndWarmupPolicies)
{
  EXPECT_FALSE(usesRecoveryTimeout(
    LocalizationAttemptKind::CONFIGURED_INITIAL));
  EXPECT_FALSE(switchesToCurrentMapAfterWarmup(
    LocalizationAttemptKind::CONFIGURED_INITIAL));

  EXPECT_FALSE(usesRecoveryTimeout(LocalizationAttemptKind::FIXED_POSE));
  EXPECT_TRUE(switchesToCurrentMapAfterWarmup(
    LocalizationAttemptKind::FIXED_POSE));

  EXPECT_TRUE(usesRecoveryTimeout(
    LocalizationAttemptKind::LOCAL_RECOVERY));
  EXPECT_TRUE(switchesToCurrentMapAfterWarmup(
    LocalizationAttemptKind::LOCAL_RECOVERY));

  EXPECT_FALSE(usesRecoveryTimeout(
    LocalizationAttemptKind::GLOBAL_RECOVERY));
  EXPECT_FALSE(switchesToCurrentMapAfterWarmup(
    LocalizationAttemptKind::GLOBAL_RECOVERY));
}

TEST(LocalizationSensorTimeoutPolicy, OnlyRevokesAcceptedTracking)
{
  EXPECT_FALSE(sensorTimeoutCausesLost(
    LocalizationState::UNINITIALIZED));
  EXPECT_FALSE(sensorTimeoutCausesLost(
    LocalizationState::LOCALIZING));
  EXPECT_TRUE(sensorTimeoutCausesLost(
    LocalizationState::TRACKING));
  EXPECT_FALSE(sensorTimeoutCausesLost(
    LocalizationState::LOST));
}

}  // namespace
}  // namespace mcl_3dl
