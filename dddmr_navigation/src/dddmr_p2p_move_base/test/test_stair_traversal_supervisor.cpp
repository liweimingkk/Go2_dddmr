#include "p2p_move_base/stair_traversal_supervisor.h"

#include <gtest/gtest.h>

namespace
{

p2p_move_base::StairObservation validObservation()
{
  p2p_move_base::StairObservation observation;
  observation.fresh = true;
  observation.gait_fresh = true;
  observation.gait_unchanged = true;
  observation.terrain_accepted = true;
  observation.stair_candidate = true;
  observation.entry_valid = true;
  observation.staircase_id = 7;
  observation.step_count = 4;
  observation.snapshot_version = 11;
  observation.static_ground_generation = 3;
  observation.confidence = 0.95;
  observation.support_ratio = 0.90;
  observation.heading_error_rad = 0.02;
  observation.lateral_error_m = 0.01;
  observation.allow_forward = true;
  return observation;
}

p2p_move_base::StairTraversalSupervisor configuredSupervisor()
{
  p2p_move_base::StairTraversalSupervisor supervisor;
  p2p_move_base::StairSupervisorConfig config;
  config.enabled = true;
  config.confirmation_cycles = 2;
  config.committed_max_yaw_rps = 0.0;
  supervisor.configure(config);
  return supervisor;
}

TEST(StairTraversalSupervisor, CompletesNormalGaitStateSequence)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();

  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::PRECHECK);
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::APPROACH);

  observation.at_entry = true;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::ALIGN);
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::COMMITTED);

  observation.landing_valid = true;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::COMMITTED);

  observation.at_entry = false;
  observation.on_stair = true;
  observation.landing_valid = false;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::COMMITTED);

  observation.on_stair = false;
  observation.landing_valid = true;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::LANDING_VERIFY);

  observation.full_body_on_landing = true;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::NORMAL);
}

TEST(StairTraversalSupervisor, StartupAndMissingGaitAreFailClosed)
{
  auto supervisor = configuredSupervisor();

  const auto startup = supervisor.filterCommand(0.30, 0.0, 0.0);
  EXPECT_FALSE(startup.allowed);
  EXPECT_EQ(startup.reason, "terrain_no_status");
  EXPECT_FALSE(supervisor.inputReady());

  auto observation = validObservation();
  observation.stair_candidate = false;
  observation.gait_fresh = false;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::NORMAL);
  EXPECT_FALSE(supervisor.filterCommand(0.30, 0.0, 0.0).allowed);
  EXPECT_EQ(supervisor.inputHoldReason(), "stale_gait");

  observation.gait_fresh = true;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::NORMAL);
  EXPECT_TRUE(supervisor.inputReady());
  EXPECT_TRUE(supervisor.filterCommand(0.30, 0.0, 0.0).allowed);
}

TEST(StairTraversalSupervisor, ActiveTraversalInputLossLatchesFault)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  supervisor.update(observation);
  ASSERT_EQ(supervisor.state(), p2p_move_base::StairTraversalState::APPROACH);

  EXPECT_EQ(supervisor.refreshInputHealth(false, true, true),
    p2p_move_base::StairTraversalState::FAULT_LATCH);
  EXPECT_EQ(supervisor.faultReason(), "stale_terrain");
  EXPECT_FALSE(supervisor.filterCommand(0.30, 0.0, 0.0).allowed);
}

TEST(StairTraversalSupervisor, NormalTerrainInputLossHoldsAndHazardsLatch)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  observation.stair_candidate = false;
  supervisor.update(observation);
  ASSERT_TRUE(supervisor.inputReady());

  EXPECT_EQ(supervisor.refreshInputHealth(false, true, true),
    p2p_move_base::StairTraversalState::NORMAL);
  EXPECT_FALSE(supervisor.faultLatched());
  EXPECT_FALSE(supervisor.filterCommand(0.30, 0.0, 0.0).allowed);

  supervisor.update(observation);
  observation.dynamic_obstacle = true;
  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::FAULT_LATCH);
  EXPECT_EQ(supervisor.faultReason(), "dynamic_obstacle");

  observation.dynamic_obstacle = false;
  observation.landing_valid = true;
  observation.full_body_on_landing = true;
  ASSERT_TRUE(supervisor.resetFault(true, observation));
  EXPECT_EQ(supervisor.refreshInputHealth(true, true, false),
    p2p_move_base::StairTraversalState::FAULT_LATCH);
  EXPECT_EQ(supervisor.faultReason(), "gait_changed");
}

TEST(StairTraversalSupervisor, ApproachPreservesScoredCommandExactly)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  supervisor.update(observation);

  const auto accepted = supervisor.filterCommand(0.30, 0.0, 0.12);
  EXPECT_TRUE(accepted.allowed);
  EXPECT_FALSE(accepted.latch_fault);
  EXPECT_DOUBLE_EQ(accepted.x, 0.30);
  EXPECT_DOUBLE_EQ(accepted.y, 0.0);
  EXPECT_DOUBLE_EQ(accepted.yaw, 0.12);
  EXPECT_EQ(accepted.reason, "stair_approach");
}

TEST(StairTraversalSupervisor, AlignRejectsCommandThatWouldNeedPostScoreClipping)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  supervisor.update(observation);
  observation.at_entry = true;
  supervisor.update(observation);
  ASSERT_EQ(supervisor.state(), p2p_move_base::StairTraversalState::ALIGN);

  const auto unsafe = supervisor.filterCommand(0.30, 0.0, 0.40);
  EXPECT_FALSE(unsafe.allowed);
  EXPECT_TRUE(unsafe.latch_fault);
  EXPECT_EQ(unsafe.reason, "unscored_stair_align_command");

  const auto scored_pure_yaw = supervisor.filterCommand(0.0, 0.0, 0.20);
  EXPECT_TRUE(scored_pure_yaw.allowed);
  EXPECT_DOUBLE_EQ(scored_pure_yaw.x, 0.0);
  EXPECT_DOUBLE_EQ(scored_pure_yaw.y, 0.0);
  EXPECT_DOUBLE_EQ(scored_pure_yaw.yaw, 0.20);
}

TEST(StairTraversalSupervisor, CommittedCommandMustAlreadyMatchConstraints)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  supervisor.update(observation);
  observation.at_entry = true;
  supervisor.update(observation);
  supervisor.update(observation);

  const auto accepted = supervisor.filterCommand(0.30, 0.0, 0.0);
  EXPECT_TRUE(accepted.allowed);
  EXPECT_DOUBLE_EQ(accepted.x, 0.30);
  EXPECT_DOUBLE_EQ(accepted.y, 0.0);
  EXPECT_DOUBLE_EQ(accepted.yaw, 0.0);

  const auto reverse = supervisor.filterCommand(-0.1, 0.0, 0.0);
  EXPECT_FALSE(reverse.allowed);
  EXPECT_TRUE(reverse.latch_fault);
  EXPECT_EQ(reverse.reason, "unscored_stair_committed_command");

  const auto clipped_yaw = supervisor.filterCommand(0.30, 0.0, 0.1);
  EXPECT_FALSE(clipped_yaw.allowed);
  EXPECT_TRUE(clipped_yaw.latch_fault);
  EXPECT_EQ(clipped_yaw.reason, "unscored_stair_committed_command");
  EXPECT_FALSE(supervisor.recoveryAllowed());
  EXPECT_FALSE(supervisor.replanningAllowed());
}

TEST(StairTraversalSupervisor, FaultResetRequiresStoppedFreshHealthyLanding)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  observation.drop_detected = true;

  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::FAULT_LATCH);
  EXPECT_TRUE(supervisor.faultLatched());
  EXPECT_EQ(supervisor.faultReason(), "drop_detected");
  EXPECT_FALSE(supervisor.filterCommand(0.3, 0.0, 0.0).allowed);
  observation.drop_detected = false;
  observation.landing_valid = true;
  observation.full_body_on_landing = true;
  EXPECT_FALSE(supervisor.resetFault(false, observation));

  observation.gait_fresh = false;
  EXPECT_FALSE(supervisor.resetFault(true, observation));

  observation.gait_fresh = true;
  observation.terrain_accepted = false;
  EXPECT_FALSE(supervisor.resetFault(true, observation));

  observation.terrain_accepted = true;
  EXPECT_TRUE(supervisor.resetFault(true, observation));
  EXPECT_EQ(supervisor.state(), p2p_move_base::StairTraversalState::NORMAL);
  EXPECT_TRUE(supervisor.inputReady());
}

TEST(StairTraversalSupervisor, StaticGroundGenerationChangeFailsClosed)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  observation.static_ground_generation = 4;

  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::FAULT_LATCH);
  EXPECT_EQ(supervisor.faultReason(), "stair_model_changed");
}

TEST(StairTraversalSupervisor, OnlineSnapshotRefreshDoesNotChangeStaticStairModel)
{
  auto supervisor = configuredSupervisor();
  auto observation = validObservation();
  supervisor.update(observation);
  observation.snapshot_version = 12;

  EXPECT_EQ(supervisor.update(observation),
    p2p_move_base::StairTraversalState::APPROACH);
  EXPECT_FALSE(supervisor.faultLatched());
}

TEST(StairTraversalSupervisor, DisabledSupervisorPreservesNormalCommands)
{
  p2p_move_base::StairTraversalSupervisor supervisor;
  p2p_move_base::StairSupervisorConfig config;
  config.enabled = false;
  supervisor.configure(config);

  const auto decision = supervisor.filterCommand(0.3, 0.1, 0.2);
  EXPECT_TRUE(decision.allowed);
  EXPECT_DOUBLE_EQ(decision.x, 0.3);
  EXPECT_DOUBLE_EQ(decision.y, 0.1);
  EXPECT_DOUBLE_EQ(decision.yaw, 0.2);
}

}  // namespace
