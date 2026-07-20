#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <mcl_3dl/adaptive_particle_policy.h>
#include <mcl_3dl/localization_state_machine.h>

namespace mcl_3dl
{
namespace
{

AdaptiveParticlePolicyConfig adaptiveConfig()
{
  AdaptiveParticlePolicyConfig config;
  config.enabled = true;
  config.min_particles = 60;
  config.max_particles = 240;
  config.confidence_frames = 2;
  config.failure_frames_before_reseed = 3;
  config.max_reduction_fraction = 0.25;
  return config;
}

AdaptiveParticleObservation stableObservation(
    const std::size_t particles, const std::size_t occupied_bins = 1)
{
  AdaptiveParticleObservation observation;
  observation.measurement_valid = true;
  observation.pose_delta_valid = true;
  observation.current_particles = particles;
  observation.occupied_bins = occupied_bins;
  observation.weighted_match_ratio = 0.80;
  observation.xy_std = 0.20;
  observation.yaw_std = 0.05;
  observation.effective_sample_ratio = 0.80;
  observation.dominant_mode_mass = 0.90;
  observation.pose_delta_xy = 0.05;
  observation.pose_delta_yaw = 0.02;
  return observation;
}

TEST(AdaptiveParticlePolicy, DisabledPolicyPreservesLegacyReductionSchedule)
{
  AdaptiveParticlePolicyConfig config;
  config.enabled = false;
  config.min_particles = 60;
  AdaptiveParticlePolicy policy(config);

  std::size_t particles = 240;
  const std::vector<std::size_t> expected{180, 135, 101, 75, 60};
  for (const std::size_t target : expected)
  {
    auto observation = stableObservation(particles);
    const auto decision = policy.observe(observation);
    EXPECT_EQ(decision.action, ParticlePolicyAction::REDUCE);
    EXPECT_EQ(decision.target_particles, target);
    EXPECT_FALSE(decision.posterior_stable);
    EXPECT_TRUE(decision.resample_required);
    particles = target;
  }

  const auto converged = policy.observe(stableObservation(particles));
  EXPECT_EQ(converged.action, ParticlePolicyAction::HOLD);
  EXPECT_TRUE(converged.posterior_stable);
}

TEST(AdaptiveParticlePolicy, RequiresConsecutiveStablePosteriorBeforeReduction)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  const auto first = policy.observe(stableObservation(240));
  EXPECT_EQ(first.action, ParticlePolicyAction::HOLD);
  EXPECT_FALSE(first.posterior_stable);
  EXPECT_EQ(first.confidence_frames, 1U);

  const auto second = policy.observe(stableObservation(240));
  EXPECT_EQ(second.action, ParticlePolicyAction::REDUCE);
  EXPECT_TRUE(second.posterior_stable);
  EXPECT_EQ(second.target_particles, 180U);
  EXPECT_EQ(second.kld_required_particles, 60U);
  EXPECT_TRUE(second.safe_to_reduce);
  EXPECT_TRUE(second.resample_required);
}

TEST(AdaptiveParticlePolicy, AmbiguousOrInvalidPosteriorHoldsParticleCount)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  auto ambiguous = stableObservation(240);
  ambiguous.dominant_mode_mass = 0.45;
  auto decision = policy.observe(ambiguous);
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(decision.target_particles, 240U);
  EXPECT_FALSE(decision.posterior_stable);
  EXPECT_FALSE(decision.resample_required);

  auto invalid = stableObservation(240);
  invalid.weighted_match_ratio = std::numeric_limits<double>::quiet_NaN();
  decision = policy.observe(invalid);
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(decision.confidence_frames, 0U);
  EXPECT_FALSE(decision.resample_required);
}

TEST(AdaptiveParticlePolicy, StrictReductionThresholdsDoNotBlockTrackingEvidence)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  auto moderate = stableObservation(240);
  moderate.weighted_match_ratio = 0.25;
  moderate.xy_std = 0.60;
  moderate.yaw_std = 0.30;

  policy.observe(moderate);
  const auto decision = policy.observe(moderate);
  EXPECT_TRUE(decision.posterior_stable);
  EXPECT_FALSE(decision.safe_to_reduce);
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(decision.target_particles, 240U);
}

TEST(AdaptiveParticlePolicy, KldComplexityCanPreventPrematureReduction)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240, 100));
  const auto decision = policy.observe(stableObservation(240, 100));
  EXPECT_TRUE(decision.posterior_stable);
  EXPECT_EQ(decision.kld_required_particles, 240U);
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(decision.target_particles, 240U);
}

TEST(AdaptiveParticlePolicy, MaterialInitialUnderProvisionBlocksTrackingEvidence)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  const auto decision = policy.observe(stableObservation(60, 100));
  EXPECT_EQ(decision.kld_required_particles, 240U);
  EXPECT_FALSE(decision.raw_posterior_stable);
  EXPECT_FALSE(decision.posterior_stable);
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
}

TEST(AdaptiveParticlePolicy, SevereRegressionAfterReductionRequestsReseed)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240));
  ASSERT_EQ(
      policy.observe(stableObservation(240)).action,
      ParticlePolicyAction::REDUCE);

  auto unstable = stableObservation(180);
  unstable.xy_std = 2.0;
  EXPECT_EQ(policy.observe(unstable).action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(policy.observe(unstable).action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(policy.observe(unstable).action, ParticlePolicyAction::RESEED);
}

TEST(AdaptiveParticlePolicy, OrdinaryPoseInstabilityDoesNotRequestReseed)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240));
  ASSERT_EQ(
      policy.observe(stableObservation(240)).action,
      ParticlePolicyAction::REDUCE);

  auto unstable = stableObservation(180);
  unstable.pose_delta_xy = 1.0;
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(policy.observe(unstable).action, ParticlePolicyAction::HOLD);
  }
}

TEST(AdaptiveParticlePolicy, KldDemandIncreaseAfterReductionRequestsReseed)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240));
  ASSERT_EQ(
      policy.observe(stableObservation(240)).action,
      ParticlePolicyAction::REDUCE);

  const auto under_provisioned = stableObservation(180, 100);
  auto decision = policy.observe(under_provisioned);
  EXPECT_FALSE(decision.raw_posterior_stable);
  EXPECT_FALSE(decision.posterior_stable);
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(policy.observe(under_provisioned).action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(policy.observe(under_provisioned).action, ParticlePolicyAction::RESEED);
}

TEST(AdaptiveParticlePolicy, SmallKldDemandFluctuationKeepsTrackingEvidence)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240));
  ASSERT_EQ(
      policy.observe(stableObservation(240)).action,
      ParticlePolicyAction::REDUCE);

  // 26 bins require 222 particles. This is above the current 180 but inside
  // the explicit 25% KLD regression tolerance (225).
  const auto jitter = stableObservation(180, 26);
  for (int i = 0; i < 5; ++i)
  {
    const auto decision = policy.observe(jitter);
    EXPECT_TRUE(decision.raw_posterior_stable);
    EXPECT_TRUE(decision.posterior_stable);
    EXPECT_NE(decision.action, ParticlePolicyAction::RESEED);
  }
}

TEST(AdaptiveParticlePolicy, RecoveredFrameClearsKldRegressionHistory)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240));
  ASSERT_EQ(
      policy.observe(stableObservation(240)).action,
      ParticlePolicyAction::REDUCE);

  const auto severe = stableObservation(180, 100);
  EXPECT_EQ(policy.observe(severe).action, ParticlePolicyAction::HOLD);
  EXPECT_TRUE(policy.observe(stableObservation(180)).raw_posterior_stable);
  EXPECT_EQ(policy.observe(severe).action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(policy.observe(severe).action, ParticlePolicyAction::HOLD);
  EXPECT_EQ(policy.observe(severe).action, ParticlePolicyAction::RESEED);
}

TEST(AdaptiveParticlePolicy, LowEssRequestsSameSizeResampleWithoutReduction)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  auto observation = stableObservation(240);
  observation.effective_sample_ratio = 0.20;
  policy.observe(observation);
  const auto decision = policy.observe(observation);
  EXPECT_TRUE(decision.posterior_stable);
  EXPECT_FALSE(decision.safe_to_reduce);
  EXPECT_EQ(decision.target_particles, 240U);
  EXPECT_TRUE(decision.resample_required);
}

TEST(AdaptiveParticlePolicy, ResetClearsConfidenceAndFailureHistory)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  policy.observe(stableObservation(240));
  policy.observe(stableObservation(240));
  policy.reset();

  const auto decision = policy.observe(stableObservation(180));
  EXPECT_EQ(decision.action, ParticlePolicyAction::HOLD);
  EXPECT_FALSE(decision.posterior_stable);
  EXPECT_EQ(decision.confidence_frames, 1U);
}

TEST(AdaptiveParticlePolicy, KldRequirementIsBoundedAndMonotonic)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  EXPECT_EQ(policy.requiredParticles(1), 60U);
  EXPECT_GE(policy.requiredParticles(10), policy.requiredParticles(2));
  EXPECT_GE(policy.requiredParticles(100), policy.requiredParticles(10));
  EXPECT_EQ(policy.requiredParticles(1000), 240U);
}

TEST(AdaptiveParticlePolicy, TrackingEvidenceIsIndependentFromKldParticleTarget)
{
  AdaptiveParticlePolicy policy(adaptiveConfig());
  LocalizationStateConfig state_config;
  state_config.tracking_match_ratio = 0.15;
  state_config.tracking_max_xy_std = 0.75;
  state_config.tracking_max_yaw_std = 0.35;
  state_config.tracking_good_frames = 3;
  state_config.tracking_min_dominant_mode_mass = 0.90;
  LocalizationStateMachine state_machine(state_config);
  state_machine.startLocalizing();

  const auto observation = stableObservation(240, 100);
  for (int frame = 0; frame < 4; ++frame)
  {
    const auto decision = policy.observe(observation);
    state_machine.observe(LocalizationObservation{
        observation.weighted_match_ratio,
        observation.xy_std,
        observation.yaw_std,
        decision.raw_posterior_stable,
        observation.dominant_mode_mass});
    EXPECT_EQ(decision.target_particles, 240U);
  }
  EXPECT_EQ(state_machine.state(), LocalizationState::TRACKING);
}

TEST(AdaptiveParticlePolicy, TrackingRequiresConfiguredDominantModeMass)
{
  LocalizationStateConfig state_config;
  state_config.tracking_good_frames = 1;
  state_config.tracking_min_dominant_mode_mass = 0.90;
  LocalizationStateMachine state_machine(state_config);
  state_machine.startLocalizing();

  EXPECT_FALSE(state_machine.observe(
      LocalizationObservation{0.8, 0.2, 0.05, true, 0.89}));
  EXPECT_EQ(state_machine.state(), LocalizationState::LOCALIZING);
  EXPECT_TRUE(state_machine.observe(
      LocalizationObservation{0.8, 0.2, 0.05, true, 0.90}));
  EXPECT_EQ(state_machine.state(), LocalizationState::TRACKING);
}

TEST(AdaptiveParticlePolicy, DisablingAdaptivePolicyDisablesTrackingModeGate)
{
  EXPECT_DOUBLE_EQ(effectiveTrackingMinDominantModeMass(false, 0.90), 0.0);
  EXPECT_DOUBLE_EQ(effectiveTrackingMinDominantModeMass(true, 0.90), 0.90);
}

}  // namespace
}  // namespace mcl_3dl
