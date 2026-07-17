#include "mouth_ground_surface.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>

namespace lego_loam_bor
{

bool validateMouthGroundSurfaceConfig(
  const MouthGroundSurfaceConfig & config, std::string * error)
{
  const auto fail = [error](const std::string & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
  const double values[] = {
    config.normal_radius,
    config.maximum_slope,
    config.minimum_obstacle_slope,
    config.maximum_planarity_residual,
    config.minimum_planar_spread,
    config.support_seed_z_min,
    config.support_seed_z_max,
    config.support_seed_x_max,
    config.support_seed_y_abs,
    config.support_reference_radius,
    config.support_connection_radius,
    config.maximum_step_height,
    config.same_surface_height_tolerance,
    config.minimum_patch_span,
    config.minimum_patch_minor_span};
  if (!std::all_of(
      std::begin(values), std::end(values), [](double value) {
        return std::isfinite(value);
      }))
  {
    return fail("mouth ground surface parameters must be finite");
  }
  constexpr double kHalfPi = 1.5707963267948966;
  if (config.normal_radius <= 0.0 || config.minimum_neighbors < 3U ||
    config.maximum_slope < 0.0 || config.maximum_slope >= kHalfPi ||
    config.minimum_obstacle_slope <= config.maximum_slope ||
    config.minimum_obstacle_slope >= kHalfPi ||
    config.maximum_planarity_residual <= 0.0 ||
    config.minimum_planar_spread <= 0.0 ||
    config.support_seed_z_min >= config.support_seed_z_max ||
    config.support_seed_x_max <= 0.0 || config.support_seed_y_abs <= 0.0 ||
    config.support_reference_radius <= 0.0 ||
    config.support_connection_radius <= 0.0 ||
    config.maximum_step_height <= 0.0 ||
    config.maximum_step_height > config.support_connection_radius ||
    config.same_surface_height_tolerance <= 0.0 ||
    config.same_surface_height_tolerance >= config.maximum_step_height ||
    config.minimum_patch_span <= 0.0 ||
    config.minimum_patch_minor_span <= 0.0 ||
    config.minimum_patch_minor_span > config.minimum_patch_span ||
    config.minimum_supported_component_points == 0U ||
    config.minimum_stair_height_levels == 0U)
  {
    return fail("invalid mouth ground surface bounds");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

MouthGroundSurfaceClassification classifyMouthGroundSurfaceDetailed(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const MouthGroundSurfaceConfig & config,
  const pcl::PointCloud<pcl::PointXYZI> * support_reference)
{
  std::string error;
  if (!validateMouthGroundSurfaceConfig(config, &error)) {
    throw std::invalid_argument(error);
  }

  MouthGroundSurfaceClassification classification;
  classification.supported_ground.assign(cloud.size(), 0U);
  classification.verified_obstacle.assign(cloud.size(), 0U);
  std::vector<std::uint8_t> planar_candidate(cloud.size(), 0U);
  if (cloud.empty()) {
    return classification;
  }

  const auto input = cloud.makeShared();
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(input);
  pcl::KdTreeFLANN<pcl::PointXYZI> support_tree;
  const bool require_support_reference = support_reference != nullptr;
  const bool support_reference_available =
    require_support_reference && !support_reference->empty();
  if (support_reference_available) {
    support_tree.setInputCloud(support_reference->makeShared());
  }
  const double minimum_normal_z = std::cos(config.maximum_slope);
  const double maximum_obstacle_normal_z =
    std::cos(config.minimum_obstacle_slope);

  std::vector<int> indices;
  std::vector<float> squared_distances;
  for (std::size_t point_index = 0; point_index < cloud.size(); ++point_index) {
    indices.clear();
    squared_distances.clear();
    if (tree.radiusSearch(
        cloud.points[point_index], config.normal_radius,
        indices, squared_distances) <
      static_cast<int>(config.minimum_neighbors))
    {
      continue;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    std::size_t finite_count = 0U;
    for (const int index : indices) {
      if (index < 0 || static_cast<std::size_t>(index) >= cloud.size()) {
        continue;
      }
      const auto & point = cloud.points[static_cast<std::size_t>(index)];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      centroid += Eigen::Vector3d(point.x, point.y, point.z);
      ++finite_count;
    }
    if (finite_count < config.minimum_neighbors) {
      continue;
    }
    centroid /= static_cast<double>(finite_count);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const int index : indices) {
      if (index < 0 || static_cast<std::size_t>(index) >= cloud.size()) {
        continue;
      }
      const auto & point = cloud.points[static_cast<std::size_t>(index)];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const Eigen::Vector3d delta(
        static_cast<double>(point.x) - centroid.x(),
        static_cast<double>(point.y) - centroid.y(),
        static_cast<double>(point.z) - centroid.z());
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<double>(finite_count);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    const auto eigenvalues = solver.eigenvalues();
    if (!eigenvalues.allFinite() || eigenvalues[0] < -1e-9) {
      continue;
    }
    const double residual = std::sqrt(std::max(0.0, eigenvalues[0]));
    const double planar_spread = std::sqrt(std::max(0.0, eigenvalues[1]));
    const Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (residual <= config.maximum_planarity_residual &&
      planar_spread >= config.minimum_planar_spread)
    {
      const double normal_z = std::abs(normal.z());
      if (normal_z >= minimum_normal_z) {
        planar_candidate[point_index] = 1U;
      } else if (normal_z <= maximum_obstacle_normal_z) {
        classification.verified_obstacle[point_index] = 1U;
      }
    }
  }

  // First form near-equal-height planar patches. This makes point count and
  // physical span requirements apply to every tread independently rather
  // than allowing a tiny platform to borrow support from a large floor.
  const std::size_t no_patch = cloud.size();
  std::vector<std::size_t> point_patch(cloud.size(), no_patch);
  std::vector<std::vector<std::size_t>> patches;
  std::vector<int> support_neighbors;
  std::vector<float> support_squared_distances;
  for (std::size_t start = 0; start < cloud.size(); ++start) {
    if (planar_candidate[start] == 0U || point_patch[start] != no_patch) {
      continue;
    }
    const std::size_t patch_index = patches.size();
    patches.emplace_back();
    std::deque<std::size_t> queue;
    queue.push_back(start);
    point_patch[start] = patch_index;
    while (!queue.empty()) {
      const std::size_t current_index = queue.front();
      queue.pop_front();
      patches.back().push_back(current_index);
      support_neighbors.clear();
      support_squared_distances.clear();
      tree.radiusSearch(
        cloud.points[current_index], config.support_connection_radius,
        support_neighbors, support_squared_distances);
      for (const int neighbor : support_neighbors) {
        if (neighbor < 0 || static_cast<std::size_t>(neighbor) >= cloud.size()) {
          continue;
        }
        const std::size_t neighbor_index = static_cast<std::size_t>(neighbor);
        if (point_patch[neighbor_index] != no_patch ||
          planar_candidate[neighbor_index] == 0U)
        {
          continue;
        }
        const double dz = std::abs(
          static_cast<double>(cloud.points[neighbor_index].z) -
          static_cast<double>(cloud.points[current_index].z));
        if (dz > config.same_surface_height_tolerance) {
          continue;
        }
        point_patch[neighbor_index] = patch_index;
        queue.push_back(neighbor_index);
      }
    }
  }

  const std::size_t patch_count = patches.size();
  std::vector<std::uint8_t> patch_qualified(patch_count, 0U);
  std::vector<std::uint8_t> patch_seed(patch_count, 0U);
  std::vector<double> patch_mean_z(patch_count, 0.0);
  for (std::size_t patch_index = 0; patch_index < patch_count; ++patch_index) {
    const auto & patch = patches[patch_index];
    if (patch.size() < config.minimum_supported_component_points) {
      continue;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double sum_z = 0.0;
    bool has_seed_point = false;
    std::vector<int> reference_neighbors;
    std::vector<float> reference_squared_distances;
    for (const std::size_t point_index : patch) {
      const auto & point = cloud.points[point_index];
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
      sum_z += static_cast<double>(point.z);
      bool near_support_reference = !require_support_reference;
      if (support_reference_available) {
        reference_neighbors.clear();
        reference_squared_distances.clear();
        near_support_reference = support_tree.radiusSearch(
          point, config.support_reference_radius, reference_neighbors,
          reference_squared_distances, 1) > 0;
      }
      has_seed_point = has_seed_point ||
        (near_support_reference && point.x <= config.support_seed_x_max &&
        std::abs(point.y) <= config.support_seed_y_abs &&
        point.z >= config.support_seed_z_min &&
        point.z <= config.support_seed_z_max);
    }
    const double span_x = max_x - min_x;
    const double span_y = max_y - min_y;
    if (std::max(span_x, span_y) < config.minimum_patch_span ||
      std::min(span_x, span_y) < config.minimum_patch_minor_span)
    {
      continue;
    }
    patch_qualified[patch_index] = 1U;
    patch_seed[patch_index] = has_seed_point ? 1U : 0U;
    patch_mean_z[patch_index] = sum_z / static_cast<double>(patch.size());
  }

  // Build a graph between independently qualified patches using only the
  // generic step capability envelope.
  std::vector<std::vector<std::size_t>> patch_graph(patch_count);
  for (std::size_t point_index = 0; point_index < cloud.size(); ++point_index) {
    const std::size_t from_patch = point_patch[point_index];
    if (from_patch == no_patch || patch_qualified[from_patch] == 0U) {
      continue;
    }
    support_neighbors.clear();
    support_squared_distances.clear();
    tree.radiusSearch(
      cloud.points[point_index], config.support_connection_radius,
      support_neighbors, support_squared_distances);
    for (const int neighbor : support_neighbors) {
      if (neighbor < 0 || static_cast<std::size_t>(neighbor) >= cloud.size()) {
        continue;
      }
      const std::size_t neighbor_index = static_cast<std::size_t>(neighbor);
      const std::size_t to_patch = point_patch[neighbor_index];
      if (to_patch == no_patch || to_patch == from_patch ||
        patch_qualified[to_patch] == 0U)
      {
        continue;
      }
      const double dz = std::abs(
        static_cast<double>(cloud.points[neighbor_index].z) -
        static_cast<double>(cloud.points[point_index].z));
      if (dz <= config.maximum_step_height) {
        patch_graph[from_patch].push_back(to_patch);
      }
    }
  }

  for (auto & neighbors : patch_graph) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }

  std::vector<std::uint8_t> visited_patch(patch_count, 0U);
  for (std::size_t seed_patch = 0; seed_patch < patch_count; ++seed_patch) {
    if (patch_qualified[seed_patch] == 0U || patch_seed[seed_patch] == 0U ||
      visited_patch[seed_patch] != 0U)
    {
      continue;
    }
    std::deque<std::size_t> queue;
    std::vector<std::size_t> region;
    queue.push_back(seed_patch);
    visited_patch[seed_patch] = 1U;
    while (!queue.empty()) {
      const std::size_t current_patch = queue.front();
      queue.pop_front();
      region.push_back(current_patch);
      for (const std::size_t neighbor_patch : patch_graph[current_patch]) {
        if (visited_patch[neighbor_patch] == 0U) {
          visited_patch[neighbor_patch] = 1U;
          queue.push_back(neighbor_patch);
        }
      }
    }

    std::vector<double> height_levels;
    std::vector<double> seed_heights;
    height_levels.reserve(region.size());
    for (const std::size_t patch_index : region) {
      height_levels.push_back(patch_mean_z[patch_index]);
      if (patch_seed[patch_index] != 0U) {
        seed_heights.push_back(patch_mean_z[patch_index]);
      }
    }
    std::sort(height_levels.begin(), height_levels.end());
    std::size_t distinct_height_levels = 0U;
    double previous_height = 0.0;
    for (const double height : height_levels) {
      if (distinct_height_levels == 0U ||
        height - previous_height > 2.0 * config.same_surface_height_tolerance)
      {
        ++distinct_height_levels;
        previous_height = height;
      }
    }

    const bool is_stair_chain =
      distinct_height_levels >= config.minimum_stair_height_levels;
    for (const std::size_t patch_index : region) {
      bool keep_patch = is_stair_chain;
      if (!keep_patch) {
        keep_patch = std::any_of(
          seed_heights.begin(), seed_heights.end(),
          [&](double seed_height) {
            return std::abs(patch_mean_z[patch_index] - seed_height) <=
            2.0 * config.same_surface_height_tolerance;
          });
      }
      if (keep_patch) {
        for (const std::size_t point_index : patches[patch_index]) {
          classification.supported_ground[point_index] = 1U;
        }
      }
    }
  }

  return classification;
}

std::vector<std::uint8_t> classifyMouthGroundSurface(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const MouthGroundSurfaceConfig & config,
  const pcl::PointCloud<pcl::PointXYZI> * support_reference)
{
  return classifyMouthGroundSurfaceDetailed(
    cloud, config, support_reference).supported_ground;
}

}  // namespace lego_loam_bor
