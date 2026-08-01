/*
 * Copyright (c) 2026, DDDMR Navigation contributors
 * All rights reserved.
 */

#ifndef MCL_3DL_PARTICLE_SPREAD_H
#define MCL_3DL_PARTICLE_SPREAD_H

#include <algorithm>
#include <cmath>
#include <limits>

#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{

struct PositionSpread
{
  double tangent{std::numeric_limits<double>::infinity()};
  double normal{std::numeric_limits<double>::infinity()};
  double raw_xy{std::numeric_limits<double>::infinity()};
  double raw_z{std::numeric_limits<double>::infinity()};
};

inline bool isValidSurfaceNormal(const Vec3& surface_normal)
{
  const double norm_squared =
      static_cast<double>(surface_normal.x_) * surface_normal.x_ +
      static_cast<double>(surface_normal.y_) * surface_normal.y_ +
      static_cast<double>(surface_normal.z_) * surface_normal.z_;
  return std::isfinite(norm_squared) && norm_squared >= 1e-12;
}

inline Vec3 normalizedSurfaceNormalOrUp(const Vec3& surface_normal)
{
  if (!isValidSurfaceNormal(surface_normal))
  {
    return Vec3(0.0, 0.0, 1.0);
  }

  const double norm_squared =
      static_cast<double>(surface_normal.x_) * surface_normal.x_ +
      static_cast<double>(surface_normal.y_) * surface_normal.y_ +
      static_cast<double>(surface_normal.z_) * surface_normal.z_;
  Vec3 normal = surface_normal / static_cast<float>(std::sqrt(norm_squared));
  if (normal.z_ < 0.0f)
  {
    normal *= -1.0f;
  }
  return normal;
}

inline double surfaceTiltFromUp(const Vec3& surface_normal)
{
  if (!isValidSurfaceNormal(surface_normal))
  {
    return std::numeric_limits<double>::infinity();
  }
  const Vec3 normal = normalizedSurfaceNormalOrUp(surface_normal);
  return std::acos(std::clamp(
      static_cast<double>(normal.z_), -1.0, 1.0));
}

inline bool slopeCompensationActive(
    const bool enabled,
    const bool ground_mode_enabled,
    const bool state_ground_constrained,
    const bool measurement_ground_valid,
    const Vec3& surface_normal,
    const double minimum_tilt)
{
  if (!enabled || !ground_mode_enabled || !state_ground_constrained ||
      !measurement_ground_valid || !std::isfinite(minimum_tilt))
  {
    return false;
  }
  const double tilt = surfaceTiltFromUp(surface_normal);
  return std::isfinite(tilt) && tilt >= std::max(0.0, minimum_tilt);
}

class PositionSpreadAccumulator
{
public:
  explicit PositionSpreadAccumulator(const Vec3& surface_normal)
    : normal_(normalizedSurfaceNormalOrUp(surface_normal))
    , world_aligned_(normal_ == Vec3(0.0, 0.0, 1.0))
  {
  }

  void add(const Vec3& displacement, const double weight)
  {
    if (weight == 0.0)
    {
      return;
    }
    if (!std::isfinite(weight) || weight < 0.0 ||
        !std::isfinite(displacement.x_) ||
        !std::isfinite(displacement.y_) ||
        !std::isfinite(displacement.z_))
    {
      valid_ = false;
      return;
    }

    const double dx = displacement.x_;
    const double dy = displacement.y_;
    const double dz = displacement.z_;
    const double raw_xy_squared = dx * dx + dy * dy;
    const double raw_z_squared = dz * dz;
    double tangent_squared = raw_xy_squared;
    double normal_offset = dz;
    if (!world_aligned_)
    {
      normal_offset =
          dx * normal_.x_ + dy * normal_.y_ + dz * normal_.z_;
      const double total_squared = raw_xy_squared + raw_z_squared;
      tangent_squared = std::max(
          0.0, total_squared - normal_offset * normal_offset);
    }

    tangent_variance_ += weight * tangent_squared;
    normal_variance_ += weight * normal_offset * normal_offset;
    raw_xy_variance_ += weight * raw_xy_squared;
    raw_z_variance_ += weight * raw_z_squared;
    weight_sum_ += weight;
  }

  PositionSpread spread() const
  {
    if (!valid_ || weight_sum_ <= 0.0)
    {
      return {};
    }

    PositionSpread result;
    result.tangent = std::sqrt(std::max(0.0, tangent_variance_ / weight_sum_));
    result.normal = std::sqrt(std::max(0.0, normal_variance_ / weight_sum_));
    result.raw_xy = std::sqrt(std::max(0.0, raw_xy_variance_ / weight_sum_));
    result.raw_z = std::sqrt(std::max(0.0, raw_z_variance_ / weight_sum_));
    return result;
  }

  const Vec3& normal() const
  {
    return normal_;
  }

private:
  Vec3 normal_{0.0, 0.0, 1.0};
  bool world_aligned_{true};
  double tangent_variance_{0.0};
  double normal_variance_{0.0};
  double raw_xy_variance_{0.0};
  double raw_z_variance_{0.0};
  double weight_sum_{0.0};
  bool valid_{true};
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_PARTICLE_SPREAD_H
