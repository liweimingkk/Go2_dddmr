/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#ifndef TRAJECTORY_GENERATORS__TERRAIN_FOLLOWING_DD_THEORY_H_
#define TRAJECTORY_GENERATORS__TERRAIN_FOLLOWING_DD_THEORY_H_

#include <trajectory_generators/terrain_trajectory_projector.h>
#include <trajectory_generators/trajectory_generator_theory.h>

#include <memory>

namespace trajectory_generators
{

// Optional differential-drive generator that delegates x/y/yaw sampling to
// DDSimpleTrajectoryGeneratorTheory, then fail-closed projects every complete
// candidate onto one leased terrain context.  It is not selected by any
// shipped robot configuration and remains disabled unless explicitly enabled.
class TerrainFollowingDDTheory final : public TrajectoryGeneratorTheory
{
public:
  TerrainFollowingDDTheory() = default;

  bool hasMoreTrajectories() override;
  bool nextTrajectory(base_trajectory::Trajectory & trajectory) override;
  void initialise() override;

protected:
  void onInitialize() override;

private:
  bool contextStillCurrent() const noexcept;
  void recordRejection(
    TerrainTrajectoryRejection rejection,
    std::size_t pose_index,
    base_trajectory::Trajectory * trajectory = nullptr);
  bool rebuildProjectedTrajectory(
    const base_trajectory::Trajectory & source,
    const TerrainTrajectoryProjectionResult & projection,
    base_trajectory::Trajectory * output) const;

  bool enabled_{false};
  bool fail_closed_{true};
  double nominal_body_clearance_m_{0.24};
  TerrainTrajectoryProjectionLimits limits_;
  std::shared_ptr<TrajectoryGeneratorTheory> planar_generator_;
  std::unique_ptr<TerrainTrajectoryProjector> projector_;
  TerrainProjectionContextConstPtr cycle_context_;
  bool cycle_invalidated_{false};
  TerrainTrajectoryRejection cycle_rejection_{TerrainTrajectoryRejection::TERRAIN_DISABLED};
};

}  // namespace trajectory_generators

#endif  // TRAJECTORY_GENERATORS__TERRAIN_FOLLOWING_DD_THEORY_H_
