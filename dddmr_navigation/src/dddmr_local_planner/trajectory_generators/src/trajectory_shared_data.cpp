/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include <trajectory_generators/trajectory_shared_data.h>

#include <memory>
#include <utility>
#include <vector>

namespace trajectory_generators
{

void TrajectoryGeneratorSharedData::requestTerrainProjectionData() noexcept
{
  terrain_projection_data_requested_.store(true, std::memory_order_release);
}

bool TrajectoryGeneratorSharedData::terrainProjectionDataRequested() const noexcept
{
  return terrain_projection_data_requested_.load(std::memory_order_acquire);
}

void TrajectoryGeneratorSharedData::updateTerrainProjectionData(
  perception_3d::TerrainSnapshotConstPtr snapshot,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & ground,
  std::uint64_t static_ground_generation)
{
  const auto existing = terrainProjectionData();
  if (existing && snapshot && existing->snapshot().get() == snapshot.get() &&
    existing->staticGroundGeneration() == static_ground_generation && ground &&
    existing->groundPoints().size() == ground->points.size())
  {
    return;
  }

  if (!snapshot || !ground || ground->points.empty()) {
    std::atomic_store_explicit(
      &terrain_projection_context_, TerrainProjectionContextConstPtr{},
      std::memory_order_release);
    return;
  }

  std::vector<Eigen::Vector3d> ground_points;
  ground_points.reserve(ground->points.size());
  for (const auto & point : ground->points) {
    ground_points.emplace_back(point.x, point.y, point.z);
  }
  TerrainProjectionContextConstPtr context =
    std::make_shared<const TerrainProjectionContext>(
    std::move(snapshot), static_ground_generation, std::move(ground_points));
  std::atomic_store_explicit(
    &terrain_projection_context_, std::move(context), std::memory_order_release);
}

TerrainProjectionContextConstPtr
TrajectoryGeneratorSharedData::terrainProjectionData() const noexcept
{
  return std::atomic_load_explicit(
    &terrain_projection_context_, std::memory_order_acquire);
}

}  // namespace trajectory_generators
