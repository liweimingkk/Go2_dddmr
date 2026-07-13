/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "perception_3d/terrain_model_builder.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

namespace perception_3d
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr float kGeometryEpsilon = 1.0e-6F;

bool finitePositive(float value)
{
  return std::isfinite(value) && value > 0.0F;
}

bool finiteNonnegative(float value)
{
  return std::isfinite(value) && value >= 0.0F;
}

bool finiteRatio(float value)
{
  return finiteNonnegative(value) && value <= 1.0F;
}

bool validConfig(const TerrainModelBuilderConfig & config)
{
  return finitePositive(config.normal_radius_m) && config.min_normal_neighbors >= 3U &&
         finitePositive(config.max_plane_residual_m) &&
         finiteNonnegative(config.flat_slope_threshold_rad) &&
         config.flat_slope_threshold_rad <= config.max_model_slope_rad &&
         finitePositive(config.max_model_slope_rad) && config.max_model_slope_rad <= kPi / 2.0F &&
         finitePositive(config.support_radius_m) &&
         finitePositive(config.support_plane_tolerance_m) &&
         config.support_sector_count >= 4U &&
         finiteRatio(config.min_observed_support_ratio) &&
         finiteRatio(config.edge_support_ratio) &&
         config.min_observed_support_ratio <= config.edge_support_ratio &&
         finitePositive(config.surface_connectivity_radius_m) &&
         finiteNonnegative(config.max_surface_height_delta_m) &&
         finiteNonnegative(config.max_surface_normal_change_rad) &&
         config.max_surface_normal_change_rad <= kPi;
}

struct CellKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const CellKey & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const noexcept
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

class SpatialIndex
{
public:
  SpatialIndex(const std::vector<Eigen::Vector3f> & points, float cell_size)
  : points_(points), cell_size_(cell_size)
  {
    for (std::size_t index = 0; index < points_.size(); ++index) {
      if (points_[index].allFinite()) {
        cells_[keyFor(points_[index])].push_back(index);
      }
    }
  }

  std::vector<std::size_t> radiusSearch(
    const Eigen::Vector3f & center,
    float radius) const
  {
    std::vector<std::size_t> neighbors;
    if (!center.allFinite() || !finitePositive(radius)) {
      return neighbors;
    }

    const CellKey center_key = keyFor(center);
    const std::int64_t cell_range =
      static_cast<std::int64_t>(std::ceil(radius / cell_size_));
    const float squared_radius = radius * radius;
    for (std::int64_t dx = -cell_range; dx <= cell_range; ++dx) {
      for (std::int64_t dy = -cell_range; dy <= cell_range; ++dy) {
        for (std::int64_t dz = -cell_range; dz <= cell_range; ++dz) {
          const CellKey key{center_key.x + dx, center_key.y + dy, center_key.z + dz};
          const auto cell = cells_.find(key);
          if (cell == cells_.end()) {
            continue;
          }
          for (std::size_t index : cell->second) {
            if ((points_[index] - center).squaredNorm() <= squared_radius) {
              neighbors.push_back(index);
            }
          }
        }
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    return neighbors;
  }

private:
  CellKey keyFor(const Eigen::Vector3f & point) const
  {
    return CellKey{
      static_cast<std::int64_t>(std::floor(point.x() / cell_size_)),
      static_cast<std::int64_t>(std::floor(point.y() / cell_size_)),
      static_cast<std::int64_t>(std::floor(point.z() / cell_size_))};
  }

  const std::vector<Eigen::Vector3f> & points_;
  float cell_size_;
  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> cells_;
};

class DisjointSets
{
public:
  explicit DisjointSets(std::size_t size) : parent_(size), rank_(size, 0U)
  {
    std::iota(parent_.begin(), parent_.end(), 0U);
  }

  std::size_t find(std::size_t value)
  {
    if (parent_[value] != value) {
      parent_[value] = find(parent_[value]);
    }
    return parent_[value];
  }

  void merge(std::size_t first, std::size_t second)
  {
    first = find(first);
    second = find(second);
    if (first == second) {
      return;
    }
    // Rank keeps this near constant time; index tie-breaking keeps the result
    // independent of hash bucket iteration order.
    if (rank_[first] < rank_[second] ||
      (rank_[first] == rank_[second] && first > second))
    {
      std::swap(first, second);
    }
    parent_[second] = first;
    if (rank_[first] == rank_[second]) {
      ++rank_[first];
    }
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<std::uint8_t> rank_;
};

bool estimatePlane(
  const std::vector<Eigen::Vector3f> & points,
  const std::vector<std::size_t> & neighbors,
  Eigen::Vector3f * normal,
  float * roughness)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (std::size_t index : neighbors) {
    centroid += points[index].cast<double>();
  }
  centroid /= static_cast<double>(neighbors.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t index : neighbors) {
    const Eigen::Vector3d centered = points[index].cast<double>() - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(neighbors.size());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return false;
  }
  if (solver.eigenvalues().y() <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double smallest_eigenvalue = std::max(0.0, solver.eigenvalues().x());
  Eigen::Vector3d estimated_normal = solver.eigenvectors().col(0);
  if (!estimated_normal.allFinite() || estimated_normal.norm() <= kGeometryEpsilon) {
    return false;
  }
  estimated_normal.normalize();
  if (estimated_normal.z() < 0.0) {
    estimated_normal = -estimated_normal;
  }
  *normal = estimated_normal.cast<float>();
  *roughness = static_cast<float>(std::sqrt(smallest_eigenvalue));
  return std::isfinite(*roughness);
}

float supportRatio(
  const Eigen::Vector3f & point,
  const Eigen::Vector3f & normal,
  const std::vector<Eigen::Vector3f> & support_points,
  const SpatialIndex & support_index,
  const TerrainModelBuilderConfig & config)
{
  std::vector<bool> occupied(config.support_sector_count, false);
  for (std::size_t index : support_index.radiusSearch(point, config.support_radius_m)) {
    const Eigen::Vector3f delta = support_points[index] - point;
    const float horizontal_distance = delta.head<2>().norm();
    if (horizontal_distance <= kGeometryEpsilon ||
      std::abs(delta.dot(normal)) > config.support_plane_tolerance_m)
    {
      continue;
    }
    float angle = std::atan2(delta.y(), delta.x());
    if (angle < 0.0F) {
      angle += 2.0F * kPi;
    }
    std::size_t sector = static_cast<std::size_t>(
      std::floor(angle * static_cast<float>(config.support_sector_count) / (2.0F * kPi)));
    sector = std::min(sector, config.support_sector_count - 1U);
    occupied[sector] = true;
  }
  const std::size_t occupied_count = static_cast<std::size_t>(
    std::count(occupied.begin(), occupied.end(), true));
  return static_cast<float>(occupied_count) /
         static_cast<float>(config.support_sector_count);
}

bool surfaceEligible(const TerrainNode & node)
{
  return node.terrain_class == TerrainClass::FLAT ||
         node.terrain_class == TerrainClass::RAMP;
}

float normalAngle(const TerrainNode & first, const TerrainNode & second)
{
  Eigen::Vector3f first_normal = first.normal.normalized();
  Eigen::Vector3f second_normal = second.normal.normalized();
  if (first_normal.z() < 0.0F) {
    first_normal = -first_normal;
  }
  if (second_normal.z() < 0.0F) {
    second_normal = -second_normal;
  }
  return std::acos(std::clamp(first_normal.dot(second_normal), -1.0F, 1.0F));
}

std::size_t assignSurfaces(
  const std::vector<Eigen::Vector3f> & points,
  const SpatialIndex & ground_index,
  const TerrainModelBuilderConfig & config,
  std::vector<TerrainNode> * nodes)
{
  DisjointSets sets(nodes->size());
  for (std::size_t index = 0; index < nodes->size(); ++index) {
    if (!surfaceEligible((*nodes)[index])) {
      continue;
    }
    const auto neighbors = ground_index.radiusSearch(
      points[index], config.surface_connectivity_radius_m);
    for (std::size_t neighbor : neighbors) {
      if (neighbor <= index || !surfaceEligible((*nodes)[neighbor])) {
        continue;
      }
      if (std::abs(points[neighbor].z() - points[index].z()) >
        config.max_surface_height_delta_m)
      {
        continue;
      }
      if (normalAngle((*nodes)[index], (*nodes)[neighbor]) >
        config.max_surface_normal_change_rad)
      {
        continue;
      }
      sets.merge(index, neighbor);
    }
  }

  std::unordered_map<std::size_t, std::int32_t> root_to_surface;
  std::int32_t next_surface = 0;
  for (std::size_t index = 0; index < nodes->size(); ++index) {
    if (!surfaceEligible((*nodes)[index])) {
      continue;
    }
    const std::size_t root = sets.find(index);
    auto insertion = root_to_surface.emplace(root, next_surface);
    if (insertion.second) {
      ++next_surface;
    }
    (*nodes)[index].surface_id = insertion.first->second;
  }

  // EDGE points receive the nearest already established surface identity but
  // never union two surfaces.  An isolated edge gets its own stable identity.
  for (std::size_t index = 0; index < nodes->size(); ++index) {
    if ((*nodes)[index].terrain_class != TerrainClass::EDGE) {
      continue;
    }
    std::int32_t nearest_surface = -1;
    float nearest_squared_distance = std::numeric_limits<float>::infinity();
    for (std::size_t neighbor : ground_index.radiusSearch(
      points[index], config.surface_connectivity_radius_m))
    {
      if (!surfaceEligible((*nodes)[neighbor]) || (*nodes)[neighbor].surface_id < 0) {
        continue;
      }
      const float squared_distance = (points[neighbor] - points[index]).squaredNorm();
      if (squared_distance < nearest_squared_distance ||
        (squared_distance == nearest_squared_distance &&
        (*nodes)[neighbor].surface_id < nearest_surface))
      {
        nearest_squared_distance = squared_distance;
        nearest_surface = (*nodes)[neighbor].surface_id;
      }
    }
    if (nearest_surface < 0) {
      nearest_surface = next_surface++;
    }
    (*nodes)[index].surface_id = nearest_surface;
  }
  return static_cast<std::size_t>(next_surface);
}

}  // namespace

TerrainModelBuildResult TerrainModelBuilder::build(
  const TerrainModelBuildInput & input,
  const TerrainModelBuilderConfig & config)
{
  TerrainModelBuildResult result;
  if (!validConfig(config)) {
    result.error = "invalid terrain model builder configuration";
    return result;
  }
  if (input.map_hash.empty() || input.version == 0U || input.stamp_nanoseconds < 0) {
    result.error = "map hash, snapshot version, or timestamp is invalid";
    return result;
  }
  if (input.mapground_points.empty()) {
    result.error = "mapground input is empty";
    return result;
  }

  const float cell_size = std::max({
    config.normal_radius_m,
    config.support_radius_m,
    config.surface_connectivity_radius_m});
  const SpatialIndex ground_index(input.mapground_points, cell_size);
  const SpatialIndex support_index(input.support_points, cell_size);
  std::vector<TerrainNode> nodes(input.mapground_points.size());

  for (std::size_t index = 0; index < input.mapground_points.size(); ++index) {
    TerrainNode & node = nodes[index];
    node.ground_index = static_cast<std::uint32_t>(index);
    node.flags = TERRAIN_NODE_STATIC_MAP;
    const Eigen::Vector3f & point = input.mapground_points[index];
    if (!point.allFinite()) {
      ++result.statistics.invalid_ground_point_count;
      ++result.statistics.unknown_count;
      continue;
    }

    const auto neighbors = ground_index.radiusSearch(point, config.normal_radius_m);
    if (neighbors.size() < config.min_normal_neighbors ||
      !estimatePlane(input.mapground_points, neighbors, &node.normal, &node.roughness_m))
    {
      ++result.statistics.unknown_count;
      continue;
    }

    node.slope_rad = std::acos(std::clamp(node.normal.z(), -1.0F, 1.0F));
    node.support_ratio = supportRatio(
      point, node.normal, input.support_points, support_index, config);
    const float plane_confidence = std::clamp(
      1.0F - node.roughness_m / config.max_plane_residual_m, 0.0F, 1.0F);
    node.confidence = std::min(plane_confidence, node.support_ratio);

    if (node.roughness_m > config.max_plane_residual_m ||
      node.slope_rad > config.max_model_slope_rad)
    {
      node.terrain_class = TerrainClass::UNKNOWN;
      node.confidence = 0.0F;
      ++result.statistics.unknown_count;
      continue;
    }
    node.flags |= TERRAIN_NODE_OBSERVED;
    if (node.support_ratio < config.min_observed_support_ratio) {
      node.terrain_class = TerrainClass::UNKNOWN;
      node.confidence = 0.0F;
      ++result.statistics.unknown_count;
    } else if (node.support_ratio < config.edge_support_ratio) {
      node.terrain_class = TerrainClass::EDGE;
      ++result.statistics.edge_count;
    } else if (node.slope_rad <= config.flat_slope_threshold_rad) {
      node.terrain_class = TerrainClass::FLAT;
      ++result.statistics.flat_count;
    } else {
      node.terrain_class = TerrainClass::RAMP;
      ++result.statistics.ramp_count;
    }
  }

  for (const auto & point : input.support_points) {
    if (!point.allFinite()) {
      ++result.statistics.invalid_support_point_count;
    }
  }

  result.statistics.surface_count = assignSurfaces(
    input.mapground_points, ground_index, config, &nodes);
  result.snapshot = std::make_shared<const TerrainSnapshot>(
    input.map_hash, input.version, input.stamp_nanoseconds, std::move(nodes));
  if (!result.snapshot->valid(&result.error)) {
    result.snapshot.reset();
  }
  return result;
}

}  // namespace perception_3d
