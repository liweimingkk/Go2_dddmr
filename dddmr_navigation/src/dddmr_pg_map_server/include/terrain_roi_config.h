/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 * All rights reserved.
 */

#ifndef DDDMR_PG_MAP_SERVER__TERRAIN_ROI_CONFIG_H_
#define DDDMR_PG_MAP_SERVER__TERRAIN_ROI_CONFIG_H_

#include <array>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cctype>
#include <set>
#include <string>

namespace dddmr_pg_map_server
{

inline bool isValidMapSha256(const std::string & value)
{
  return value.size() == 64U && std::all_of(
    value.begin(), value.end(), [](unsigned char character) {
      return std::isxdigit(character) != 0;
    });
}

inline std::string normalizeMapSha256(std::string value)
{
  if (!isValidMapSha256(value)) {
    return {};
  }
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

struct TerrainROIConfig
{
  bool enabled{false};
  double voxel_size{0.05};
  std::array<double, 3> minimum{{0.0, 0.0, 0.0}};
  std::array<double, 3> maximum{{0.0, 0.0, 0.0}};
};

struct ConfigValidationResult
{
  bool valid{true};
  std::string reason;
};

inline ConfigValidationResult validateLoadedMapIdentity(
  const bool terrain_enabled,
  const std::string & configured_sha256,
  const std::string & loaded_sha256)
{
  // Identity enforcement is additive. A legacy flat-map deployment remains
  // usable when terrain support is not requested.
  if (!terrain_enabled) {
    return {};
  }
  const auto configured = normalizeMapSha256(configured_sha256);
  if (configured.empty()) {
    return {
      false,
      "source_map_sha256 must be a complete 64-hex SHA256 when terrain is enabled"
    };
  }
  const auto loaded = normalizeMapSha256(loaded_sha256);
  if (loaded.empty()) {
    return {
      false,
      "the loaded map artifact SHA256 is unavailable or invalid"
    };
  }
  if (configured != loaded) {
    return {
      false,
      "source_map_sha256 does not match the loaded map artifact manifest"
    };
  }
  return {};
}

inline ConfigValidationResult validateVoxelSize(
  const std::string & parameter_name, const double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return {
      false,
      parameter_name + " must be finite and greater than zero"
    };
  }
  return {};
}

inline ConfigValidationResult validateTerrainROIConfig(
  const TerrainROIConfig & config, const double complete_ground_voxel_size)
{
  if (!config.enabled) {
    return {};
  }

  const auto complete_ground_voxel_result = validateVoxelSize(
    "complete_ground_voxel_size", complete_ground_voxel_size);
  if (!complete_ground_voxel_result.valid) {
    return complete_ground_voxel_result;
  }

  const auto voxel_result = validateVoxelSize(
    "terrain_roi_voxel_size", config.voxel_size);
  if (!voxel_result.valid) {
    return voxel_result;
  }

  if (config.voxel_size > complete_ground_voxel_size) {
    return {
      false,
      "terrain_roi_voxel_size must not exceed complete_ground_voxel_size"
    };
  }

  constexpr std::array<const char *, 3> axis_names{{"x", "y", "z"}};
  for (std::size_t axis = 0; axis < axis_names.size(); ++axis) {
    if (!std::isfinite(config.minimum[axis]) ||
      !std::isfinite(config.maximum[axis]))
    {
      return {
        false,
        std::string("terrain ROI ") + axis_names[axis] +
        " bounds must be finite"
      };
    }
    if (config.minimum[axis] >= config.maximum[axis]) {
      return {
        false,
        std::string("terrain_roi_min_") + axis_names[axis] +
        " must be less than terrain_roi_max_" + axis_names[axis]
      };
    }
    if (!std::isfinite(config.maximum[axis] - config.minimum[axis])) {
      return {
        false,
        std::string("terrain ROI ") + axis_names[axis] +
        " extent must be finite"
      };
    }
  }

  return {};
}

inline bool pointIsInsideTerrainROI(
  const TerrainROIConfig & config,
  const double x, const double y, const double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         x >= config.minimum[0] && x <= config.maximum[0] &&
         y >= config.minimum[1] && y <= config.maximum[1] &&
         z >= config.minimum[2] && z <= config.maximum[2];
}

template<typename PointContainer>
struct NavigationGroundMergeResult
{
  bool valid{false};
  std::string reason;
  PointContainer points;
  std::size_t coarse_points_retained{0};
  std::size_t terrain_points_added{0};
  std::size_t duplicate_points_removed{0};
};

template<typename PointContainer>
NavigationGroundMergeResult<PointContainer> mergeNavigationGroundPoints(
  const PointContainer & coarse_ground,
  const PointContainer & terrain_ground,
  const TerrainROIConfig & config,
  const double complete_ground_voxel_size)
{
  NavigationGroundMergeResult<PointContainer> result;

  const auto config_result = validateTerrainROIConfig(
    config, complete_ground_voxel_size);
  if (!config.enabled || !config_result.valid) {
    result.reason = config.enabled ?
      config_result.reason : "terrain ROI must be enabled for ROI replacement";
    return result;
  }
  if (terrain_ground.empty()) {
    result.reason = "terrain ground is empty";
    return result;
  }

  result.points.reserve(coarse_ground.size() + terrain_ground.size());
  std::set<std::array<double, 3>> unique_coordinates;

  const auto append_unique = [&result, &unique_coordinates](const auto & point) {
      const std::array<double, 3> key{{
        static_cast<double>(point.x),
        static_cast<double>(point.y),
        static_cast<double>(point.z)}};
      if (!unique_coordinates.insert(key).second) {
        ++result.duplicate_points_removed;
        return false;
      }
      result.points.push_back(point);
      return true;
    };

  for (const auto & point : coarse_ground) {
    const double x = static_cast<double>(point.x);
    const double y = static_cast<double>(point.y);
    const double z = static_cast<double>(point.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (!pointIsInsideTerrainROI(config, x, y, z) && append_unique(point)) {
      ++result.coarse_points_retained;
    }
  }

  for (const auto & point : terrain_ground) {
    const double x = static_cast<double>(point.x);
    const double y = static_cast<double>(point.y);
    const double z = static_cast<double>(point.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (!pointIsInsideTerrainROI(config, x, y, z)) {
      result.points.clear();
      result.coarse_points_retained = 0;
      result.terrain_points_added = 0;
      result.reason = "terrain ground contains a point outside the configured ROI";
      return result;
    }
    if (append_unique(point)) {
      ++result.terrain_points_added;
    }
  }

  if (result.terrain_points_added == 0) {
    result.points.clear();
    result.coarse_points_retained = 0;
    result.reason = "terrain ground has no finite unique points inside the ROI";
    return result;
  }

  result.valid = true;
  return result;
}

}  // namespace dddmr_pg_map_server

#endif  // DDDMR_PG_MAP_SERVER__TERRAIN_ROI_CONFIG_H_
