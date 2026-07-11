/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#ifndef GLOBAL_PLANNER__PLANNER_SAFETY_H_
#define GLOBAL_PLANNER__PLANNER_SAFETY_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <perception_3d/terrain_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/header.hpp>

namespace global_planner
{
namespace planner_safety
{

enum class GroundCandidateStatus : uint8_t
{
  SUCCESS = 0,
  INVALID_INPUT,
  NO_CANDIDATE,
  AMBIGUOUS_LAYER
};

struct GroundCandidateSelection
{
  GroundCandidateStatus status{GroundCandidateStatus::NO_CANDIDATE};
  unsigned int index{0};
  double squared_distance{0.0};
};

struct GroundCandidateCriteria
{
  double max_horizontal_distance{0.5};
  double max_vertical_distance{0.5};
  double layer_separation{0.35};
  double ambiguity_xy_distance{0.25};
  double max_above_query{0.5};
};

struct GoalSupportCriteria
{
  double radius{0.4};
  double layer_z_tolerance{0.2};
  std::size_t minimum_points{3};
  std::size_t minimum_sectors{3};
};

// Identity of every immutable/static input used to produce one path.  The
// binding travels alongside nav_msgs::Path inside the planners because Path
// itself has no metadata fields for map or terrain versions.
struct PlanningDataBinding
{
  bool valid{false};
  bool terrain_enabled{false};
  std::uint64_t static_ground_generation{0U};
  std::string map_hash;
  std::uint64_t terrain_snapshot_version{0U};
};

/// Compare the identities used by two planning operations.  Static-ground
/// generation is always enforced.  Terrain identity is additionally enforced
/// when terrain validation is enabled; disabled mode preserves legacy paths
/// without requiring a terrain snapshot.
bool planningBindingsMatch(
  const PlanningDataBinding & expected,
  const PlanningDataBinding & observed,
  std::string * rejection_reason = nullptr);

/// Select one nearby ground node while rejecting vertically overlapping layers.
GroundCandidateSelection selectGroundCandidate(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const pcl::PointXYZI & query,
  const std::vector<int> & candidate_indices,
  const GroundCandidateCriteria & criteria);

/// Verify that ground points surround a requested raw goal on the selected layer.
bool hasGoalSupport(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<int> & candidate_indices,
  const geometry_msgs::msg::Point & goal,
  double support_layer_z,
  const GoalSupportCriteria & criteria);

/// Build a finite, index-checked path. The final pose inherits a valid path heading.
bool buildInterpolatedPath(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<unsigned int> & path_indices,
  const std_msgs::msg::Header & header,
  double interpolation_resolution,
  nav_msgs::msg::Path & output);

/// Build a path tied to one immutable terrain snapshot. Continuous surfaces
/// retain legacy interpolation, while valid adjacent stair transitions emit
/// only their surveyed endpoint waypoints so no pose is invented on a riser.
/// Invalid snapshot identity or inconsistent stair metadata fails closed.
bool buildTerrainAwarePath(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<unsigned int> & path_indices,
  const perception_3d::TerrainSnapshotConstPtr & terrain_snapshot,
  const PlanningDataBinding & binding,
  const std_msgs::msg::Header & header,
  double interpolation_resolution,
  nav_msgs::msg::Path & output,
  std::string * rejection_reason = nullptr);

/// Join a freshly replanned prefix to an existing path, rejecting gaps and invalid paths.
bool splicePathsFailClosed(
  const nav_msgs::msg::Path & replanned_prefix,
  const nav_msgs::msg::Path & original_path,
  std::size_t original_pivot,
  double maximum_segment_length,
  nav_msgs::msg::Path & output,
  std::string * rejection_reason = nullptr);

/// Terrain-aware DWA splice. Segments within the normal continuity threshold
/// use the legacy rule. A longer segment is accepted only when both endpoints
/// map uniquely to the bound snapshot and form an explicit adjacent stair or
/// landing transition; every other gap fails closed.
bool spliceTerrainPathsFailClosed(
  const nav_msgs::msg::Path & replanned_prefix,
  const nav_msgs::msg::Path & original_path,
  std::size_t original_pivot,
  double maximum_segment_length,
  const pcl::PointCloud<pcl::PointXYZI> & ground_cloud,
  const perception_3d::TerrainSnapshotConstPtr & terrain_snapshot,
  const PlanningDataBinding & binding,
  double waypoint_mapping_tolerance,
  nav_msgs::msg::Path & output,
  std::string * rejection_reason = nullptr);

bool isFinitePoint(const geometry_msgs::msg::Point & point);

}  // namespace planner_safety
}  // namespace global_planner

#endif  // GLOBAL_PLANNER__PLANNER_SAFETY_H_
