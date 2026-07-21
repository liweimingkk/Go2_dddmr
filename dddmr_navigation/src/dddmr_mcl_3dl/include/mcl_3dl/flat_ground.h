/*
 * Copyright (c) 2026, DDDMR Navigation contributors
 * All rights reserved.
 */

#ifndef MCL_3DL_FLAT_GROUND_H
#define MCL_3DL_FLAT_GROUND_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <mcl_3dl/vec3.h>
#include <utilities.h>

namespace mcl_3dl
{

struct FlatGroundEstimate
{
  bool valid{false};
  double z{std::numeric_limits<double>::quiet_NaN()};
  Vec3 normal{0.0, 0.0, 1.0};
  double roughness{std::numeric_limits<double>::infinity()};
  std::size_t point_count{0};
};

inline double median(std::vector<double> values)
{
  if (values.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0)
  {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (values[middle - 1] + upper);
}

// Estimate the local map-ground plane at (x, y). The query z selects the
// correct floor in multi-level maps; each neighbor normal projects its point
// onto the query x/y before the robust median is taken.
inline FlatGroundEstimate estimateFlatGround(
    pcl::KdTreeFLANN<pcl_t>& ground_tree,
    const pcl::PointCloud<pcl::Normal>& normals,
    const double x,
    const double y,
    const double expected_z,
    const double search_radius,
    const std::size_t min_points)
{
  FlatGroundEstimate estimate;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(expected_z) ||
      !std::isfinite(search_radius) || search_radius <= 0.0)
  {
    return estimate;
  }

  const auto ground = ground_tree.getInputCloud();
  if (!ground || ground->empty() || normals.empty())
  {
    return estimate;
  }

  pcl_t query;
  query.x = static_cast<float>(x);
  query.y = static_cast<float>(y);
  query.z = static_cast<float>(expected_z);
  std::vector<int> indices;
  std::vector<float> squared_distances;
  ground_tree.radiusSearch(query, search_radius, indices, squared_distances);

  std::vector<double> projected_heights;
  projected_heights.reserve(indices.size());
  Vec3 normal_sum;
  for (const int index : indices)
  {
    if (index < 0 || static_cast<std::size_t>(index) >= ground->size() ||
        static_cast<std::size_t>(index) >= normals.size())
    {
      continue;
    }
    const auto& point = ground->points[static_cast<std::size_t>(index)];
    const auto& pcl_normal = normals.points[static_cast<std::size_t>(index)];
    Vec3 normal(pcl_normal.normal_x, pcl_normal.normal_y, pcl_normal.normal_z);
    if (!std::isfinite(normal.x_) || !std::isfinite(normal.y_) ||
        !std::isfinite(normal.z_) || normal.norm() < 1e-6)
    {
      continue;
    }
    normal = normal.normalized();
    if (normal.z_ < 0.0)
    {
      normal = normal * -1.0;
    }
    if (normal.z_ < 0.35)
    {
      continue;
    }
    const double projected_z = point.z -
        (normal.x_ * (x - point.x) + normal.y_ * (y - point.y)) / normal.z_;
    if (!std::isfinite(projected_z))
    {
      continue;
    }
    projected_heights.push_back(projected_z);
    normal_sum += normal;
  }

  estimate.point_count = projected_heights.size();
  if (estimate.point_count < std::max<std::size_t>(1, min_points) ||
      normal_sum.norm() < 1e-6)
  {
    return estimate;
  }

  estimate.z = median(projected_heights);
  estimate.normal = normal_sum.normalized();
  std::vector<double> deviations;
  deviations.reserve(projected_heights.size());
  for (const double height : projected_heights)
  {
    deviations.push_back(std::abs(height - estimate.z));
  }
  estimate.roughness = median(deviations);
  estimate.valid = std::isfinite(estimate.z) && std::isfinite(estimate.roughness);
  return estimate;
}

}  // namespace mcl_3dl

#endif  // MCL_3DL_FLAT_GROUND_H
