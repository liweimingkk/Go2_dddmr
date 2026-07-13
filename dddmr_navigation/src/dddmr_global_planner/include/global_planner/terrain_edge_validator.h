/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#ifndef GLOBAL_PLANNER__TERRAIN_EDGE_VALIDATOR_H_
#define GLOBAL_PLANNER__TERRAIN_EDGE_VALIDATOR_H_

#include <perception_3d/shared_data.h>
#include <perception_3d/terrain_edge_policy.h>
#include <global_planner/planner_safety.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_types.h>

namespace global_planner
{

// Capture the shared static/terrain identity while holding the perception
// ground mutex.  The mutex is recursive, so callers that already hold it can
// use the same helper without changing the global planner -> perception lock
// order.
planner_safety::PlanningDataBinding capturePlanningDataBinding(
  const std::shared_ptr<perception_3d::SharedData> & shared_data,
  bool terrain_enabled);

struct TerrainEdgeValidatorConfig
{
  perception_3d::TerrainEdgePolicyConfig policy;
  std::string expected_map_hash;
  std::uint64_t required_snapshot_version{0U};
  float support_sample_spacing_m{0.10F};
  float support_search_radius_m{0.20F};
  float support_vertical_tolerance_m{0.15F};
  float continuous_height_residual_m{0.05F};
  float ground_index_alignment_tolerance_m{0.01F};
};

constexpr std::size_t kTerrainRejectionReasonCount =
  static_cast<std::size_t>(perception_3d::TerrainRejectionReason::NO_LANDING) + 1U;

struct TerrainEdgeRejectionStatistics
{
  bool terrain_enabled{false};
  std::string map_hash;
  std::uint64_t snapshot_version{0U};
  std::uint64_t evaluated{0U};
  std::uint64_t accepted{0U};
  std::array<std::uint64_t, kTerrainRejectionReasonCount> rejected_by_reason{};

  void record(const perception_3d::TerrainEdgeResult & result);
  std::uint64_t rejected() const noexcept {return evaluated - accepted;}
  std::string toStructuredString() const;
};

struct TerrainEdgeValidationResult
{
  perception_3d::TerrainEdgeMode mode{perception_3d::TerrainEdgeMode::UNKNOWN};
  perception_3d::TerrainEdgeResult policy_result;
};

// One context is created at the beginning of one A* search.  It retains the
// immutable snapshot pointer, so every edge in that search is evaluated
// against the same map hash and terrain version even if a producer publishes a
// newer snapshot concurrently.
class TerrainEdgeSearchContext
{
public:
  bool terrainEnabled() const noexcept {return statistics_.terrain_enabled;}
  bool ready() const noexcept {return ready_;}
  const TerrainEdgeRejectionStatistics & statistics() const noexcept {return statistics_;}

private:
  friend class TerrainEdgeValidator;

  bool ready_{false};
  perception_3d::TerrainSnapshotConstPtr snapshot_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr ground_kdtree_;
  TerrainEdgeRejectionStatistics statistics_;
};

// Thin adapter between SharedData/mapground and the pure TerrainEdgePolicy.
class TerrainEdgeValidator final
{
public:
  explicit TerrainEdgeValidator(
    TerrainEdgeValidatorConfig config = TerrainEdgeValidatorConfig{});

  static bool validateConfiguration(
    const TerrainEdgeValidatorConfig & config,
    std::string * error = nullptr);

  TerrainEdgeSearchContext beginSearch(
    const std::shared_ptr<perception_3d::SharedData> & shared_data) const;

  TerrainEdgeValidationResult evaluate(
    TerrainEdgeSearchContext & context,
    std::uint32_t from_index,
    std::uint32_t to_index,
    const pcl::PointXYZI & from_position,
    const pcl::PointXYZI & to_position,
    bool dynamic_obstacle = false) const;

  bool validateEndpoint(
    TerrainEdgeSearchContext & context,
    std::uint32_t index,
    const pcl::PointXYZI & position) const;

  bool enabled() const noexcept {return config_.policy.enabled;}
  const TerrainEdgeValidatorConfig & config() const noexcept {return config_;}

private:
  perception_3d::TerrainEdgeMode classifyMode(
    const perception_3d::TerrainSnapshot & snapshot,
    std::uint32_t from_index,
    std::uint32_t to_index,
    const Eigen::Vector3f & from_position,
    const Eigen::Vector3f & to_position) const;

  bool buildSupportSamples(
    const TerrainEdgeSearchContext & context,
    const perception_3d::TerrainEdgeRequest & request,
    std::vector<perception_3d::TerrainSupportSample> * samples) const;

  TerrainEdgeValidatorConfig config_;
  bool configuration_valid_{false};
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__TERRAIN_EDGE_VALIDATOR_H_
