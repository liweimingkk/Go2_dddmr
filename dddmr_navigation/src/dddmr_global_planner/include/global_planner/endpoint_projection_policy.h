#ifndef GLOBAL_PLANNER__ENDPOINT_PROJECTION_POLICY_H_
#define GLOBAL_PLANNER__ENDPOINT_PROJECTION_POLICY_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace global_planner
{

struct VerticalSurfaceBand
{
  double min_z;
  double max_z;
  std::size_t point_count;
};

inline std::vector<VerticalSurfaceBand> findSupportedVerticalSurfaceBands(
  std::vector<double> z_values,
  double max_adjacent_z_gap,
  std::size_t minimum_points)
{
  z_values.erase(
    std::remove_if(
      z_values.begin(), z_values.end(),
      [](double z) {return !std::isfinite(z);}),
    z_values.end());
  if (z_values.empty() || max_adjacent_z_gap <= 0.0 || minimum_points == 0U) {
    return {};
  }

  std::sort(z_values.begin(), z_values.end());
  std::vector<VerticalSurfaceBand> supported_bands;
  double band_min = z_values.front();
  double band_max = z_values.front();
  std::size_t band_count = 1U;

  const auto finish_band = [&]() {
      if (band_count >= minimum_points) {
        supported_bands.push_back({band_min, band_max, band_count});
      }
    };

  for (std::size_t index = 1U; index < z_values.size(); ++index) {
    const double z = z_values[index];
    if (z - band_max > max_adjacent_z_gap) {
      finish_band();
      band_min = z;
      band_count = 1U;
    } else {
      ++band_count;
    }
    band_max = z;
  }
  finish_band();
  return supported_bands;
}

inline bool isInsideVerticalSurfaceBand(
  double z, const VerticalSurfaceBand & band)
{
  return std::isfinite(z) && z >= band.min_z && z <= band.max_z;
}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__ENDPOINT_PROJECTION_POLICY_H_
