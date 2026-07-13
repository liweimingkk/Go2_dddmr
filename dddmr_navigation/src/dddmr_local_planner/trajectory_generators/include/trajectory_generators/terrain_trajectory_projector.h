/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#ifndef TRAJECTORY_GENERATORS__TERRAIN_TRAJECTORY_PROJECTOR_H_
#define TRAJECTORY_GENERATORS__TERRAIN_TRAJECTORY_PROJECTOR_H_

#include <perception_3d/terrain_model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace trajectory_generators
{

enum class TerrainTrajectoryRejection : std::uint8_t
{
  NONE = 0,
  TERRAIN_DISABLED,
  FAIL_OPEN_CONFIGURATION,
  MISSING_SNAPSHOT,
  INVALID_SNAPSHOT,
  STATIC_GROUND_GENERATION_MISSING,
  SNAPSHOT_GROUND_MISMATCH,
  SNAPSHOT_CHANGED,
  EMPTY_TRAJECTORY,
  INVALID_GEOMETRY,
  NO_SUPPORT,
  UNKNOWN,
  EDGE,
  DROP,
  RISER,
  LOW_CONFIDENCE,
  LOW_SUPPORT,
  INVALID_NORMAL,
  NORMAL_CHANGE,
  SURFACE_CHANGE,
  STAIR_DISABLED,
  STAIR_MODEL_MISSING,
  OUTSIDE_CORRIDOR,
  ONLINE_CONFIRMATION_MISSING,
  SKIP_STEP,
  TRANSITION_WAYPOINT_MISSING,
  TRANSITION_WAYPOINT_MISSED,
  NO_LANDING,
  DIRECTION_DISABLED,
  BAD_ALIGNMENT
};

const char * toString(TerrainTrajectoryRejection reason) noexcept;

struct TerrainTrajectoryProjectionLimits
{
  double max_support_xy_distance_m{0.15};
  double max_support_vertical_distance_m{0.30};
  double min_support_ratio{0.80};
  double min_confidence{0.90};
  double min_upward_normal_z{0.20};
  double max_normal_change_rad{0.35};
  double stair_transition_waypoint_tolerance_m{0.15};
  double max_stair_transition_span_m{0.45};
  double max_stair_heading_error_rad{0.20};
  bool stairs_enabled{false};
  bool require_manual_corridor{true};
  bool require_online_confirmation{true};
};

bool terrainTrajectoryProjectionLimitsValid(
  const TerrainTrajectoryProjectionLimits & limits) noexcept;

struct TerrainBodyPose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
};

struct TerrainProjectedPose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ground_normal{Eigen::Vector3d::UnitZ()};
  double support_ratio{0.0};
  double confidence{0.0};
  std::size_t ground_index{0U};
  std::int32_t surface_id{-1};
  std::int32_t staircase_id{-1};
  std::int32_t step_index{-1};
  perception_3d::TerrainClass terrain_class{perception_3d::TerrainClass::UNKNOWN};
};

struct TerrainTrajectoryProjectionResult
{
  TerrainTrajectoryRejection rejection{TerrainTrajectoryRejection::NONE};
  std::size_t rejected_pose_index{0U};
  std::vector<TerrainProjectedPose> poses;

  bool ok() const noexcept {return rejection == TerrainTrajectoryRejection::NONE;}
};

// One immutable copy of mapground paired with one immutable TerrainSnapshot.
// The context owns its XY index and precomputed, online-confirmed adjacent stair
// transition waypoints.  Consumers retain one shared_ptr for a complete local
// trajectory generation cycle.
class TerrainProjectionContext final
{
public:
  TerrainProjectionContext(
    perception_3d::TerrainSnapshotConstPtr snapshot,
    std::uint64_t static_ground_generation,
    std::vector<Eigen::Vector3d> ground_points,
    double spatial_index_resolution_m = 0.05);

  bool valid() const noexcept {return valid_;}
  const std::string & validationError() const noexcept {return validation_error_;}
  const perception_3d::TerrainSnapshotConstPtr & snapshot() const noexcept {return snapshot_;}
  std::uint64_t staticGroundGeneration() const noexcept {return static_ground_generation_;}
  const std::vector<Eigen::Vector3d> & groundPoints() const noexcept {return ground_points_;}

private:
  friend class TerrainTrajectoryProjector;

  struct CellKey
  {
    std::int64_t x{0};
    std::int64_t y{0};

    bool operator==(const CellKey & other) const noexcept
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & key) const noexcept;
  };

  struct StairLocationKey
  {
    std::int32_t staircase_id{-1};
    std::int32_t location{-2};

    bool operator==(const StairLocationKey & other) const noexcept
    {
      return staircase_id == other.staircase_id && location == other.location;
    }
  };

  struct StairLocationKeyHash
  {
    std::size_t operator()(const StairLocationKey & key) const noexcept;
  };

  struct StairTransitionKey
  {
    std::int32_t staircase_id{-1};
    std::int32_t low_location{-2};

    bool operator==(const StairTransitionKey & other) const noexcept
    {
      return staircase_id == other.staircase_id && low_location == other.low_location;
    }
  };

  struct StairTransitionKeyHash
  {
    std::size_t operator()(const StairTransitionKey & key) const noexcept;
  };

  struct StairTransitionWaypoint
  {
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double support_span_m{0.0};
  };

  CellKey cellFor(double x, double y) const noexcept;
  void buildSpatialIndex();
  void buildStairTransitionWaypoints();

  perception_3d::TerrainSnapshotConstPtr snapshot_;
  std::uint64_t static_ground_generation_{0U};
  std::vector<Eigen::Vector3d> ground_points_;
  double spatial_index_resolution_m_{0.05};
  bool valid_{false};
  std::string validation_error_;
  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> spatial_index_;
  std::unordered_map<StairTransitionKey, StairTransitionWaypoint, StairTransitionKeyHash>
    stair_transition_waypoints_;
};

using TerrainProjectionContextConstPtr = std::shared_ptr<const TerrainProjectionContext>;

// Pointer identity plus both identities carried inside the context form a
// lease.  A different immutable context, snapshot pointer, snapshot version,
// or static generation invalidates a trajectory before it can be returned.
bool terrainProjectionContextIdentityMatches(
  const TerrainProjectionContextConstPtr & leased,
  const TerrainProjectionContextConstPtr & current) noexcept;

class TerrainTrajectoryProjector final
{
public:
  explicit TerrainTrajectoryProjector(
    TerrainTrajectoryProjectionLimits limits = TerrainTrajectoryProjectionLimits{});

  const TerrainTrajectoryProjectionLimits & limits() const noexcept {return limits_;}

  TerrainTrajectoryProjectionResult project(
    const TerrainProjectionContextConstPtr & context,
    const TerrainBodyPose & reference_body_pose,
    double body_clearance_m,
    const std::vector<TerrainBodyPose> & future_body_poses) const;

private:
  TerrainTrajectoryProjectionLimits limits_;
};

}  // namespace trajectory_generators

#endif  // TRAJECTORY_GENERATORS__TERRAIN_TRAJECTORY_PROJECTOR_H_
