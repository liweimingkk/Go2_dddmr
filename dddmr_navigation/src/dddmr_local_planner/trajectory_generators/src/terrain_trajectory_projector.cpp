/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include <trajectory_generators/terrain_trajectory_projector.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace trajectory_generators
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDistanceTieTolerance = 1e-12;

bool hasFlag(
  const perception_3d::TerrainNode & node,
  perception_3d::TerrainNodeFlag flag) noexcept
{
  return (node.flags & static_cast<std::uint32_t>(flag)) != 0U;
}

bool isStairTread(const perception_3d::TerrainNode & node) noexcept
{
  return node.terrain_class == perception_3d::TerrainClass::STAIR_TREAD;
}

bool isLanding(const perception_3d::TerrainNode & node) noexcept
{
  return hasFlag(node, perception_3d::TERRAIN_NODE_LANDING);
}

std::optional<std::int32_t> stairLocation(
  const perception_3d::TerrainNode & node,
  const perception_3d::StaircaseModel * staircase) noexcept
{
  if (staircase == nullptr || node.staircase_id != staircase->id) {
    return std::nullopt;
  }
  if (isStairTread(node)) {
    if (node.step_index < 0 || node.step_index >= staircase->step_count) {
      return std::nullopt;
    }
    return node.step_index;
  }
  if (!isLanding(node)) {
    return std::nullopt;
  }
  const bool lower = hasFlag(node, perception_3d::TERRAIN_NODE_LOWER_LANDING);
  const bool upper = hasFlag(node, perception_3d::TERRAIN_NODE_UPPER_LANDING);
  if (lower == upper) {
    return std::nullopt;
  }
  return lower ? -1 : staircase->step_count;
}

double squaredDistanceXY(const Eigen::Vector3d & lhs, const Eigen::Vector3d & rhs) noexcept
{
  return (lhs.head<2>() - rhs.head<2>()).squaredNorm();
}

double pointToSegmentDistance(
  const Eigen::Vector2d & point,
  const Eigen::Vector2d & segment_start,
  const Eigen::Vector2d & segment_end) noexcept
{
  const Eigen::Vector2d segment = segment_end - segment_start;
  const double squared_length = segment.squaredNorm();
  if (squared_length <= kDistanceTieTolerance) {
    return (point - segment_start).norm();
  }
  const double interpolation = std::clamp(
    (point - segment_start).dot(segment) / squared_length, 0.0, 1.0);
  return (point - (segment_start + interpolation * segment)).norm();
}

double wrappedAngleDifference(double lhs, double rhs) noexcept
{
  return std::abs(std::remainder(lhs - rhs, 2.0 * kPi));
}

TerrainTrajectoryRejection validateNode(
  const perception_3d::TerrainSnapshot & snapshot,
  const perception_3d::TerrainNode & node,
  const TerrainTrajectoryProjectionLimits & limits)
{
  using perception_3d::TerrainClass;

  switch (node.terrain_class) {
    case TerrainClass::UNKNOWN:
      return TerrainTrajectoryRejection::UNKNOWN;
    case TerrainClass::EDGE:
      return TerrainTrajectoryRejection::EDGE;
    case TerrainClass::DROP:
      return TerrainTrajectoryRejection::DROP;
    case TerrainClass::STAIR_RISER:
      return TerrainTrajectoryRejection::RISER;
    case TerrainClass::FLAT:
    case TerrainClass::RAMP:
    case TerrainClass::STAIR_TREAD:
      break;
  }

  if (node.confidence < limits.min_confidence) {
    return TerrainTrajectoryRejection::LOW_CONFIDENCE;
  }
  if (node.support_ratio < limits.min_support_ratio) {
    return TerrainTrajectoryRejection::LOW_SUPPORT;
  }
  const Eigen::Vector3d normal = node.normal.cast<double>();
  if (!normal.allFinite() || normal.norm() <= 1e-9 ||
    normal.normalized().z() < limits.min_upward_normal_z)
  {
    return TerrainTrajectoryRejection::INVALID_NORMAL;
  }
  if (node.surface_id < 0) {
    return TerrainTrajectoryRejection::SURFACE_CHANGE;
  }
  // Online support is required for every future pose, not only stair treads.
  // Absence of a fresh local ground return is UNKNOWN support and therefore a
  // virtual lethal condition at ramp crests, descents, and platform edges.
  if (limits.require_online_confirmation &&
    !hasFlag(node, perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED))
  {
    return TerrainTrajectoryRejection::ONLINE_CONFIRMATION_MISSING;
  }

  if (isStairTread(node) || isLanding(node)) {
    if (!limits.stairs_enabled) {
      return TerrainTrajectoryRejection::STAIR_DISABLED;
    }
    const auto * staircase = snapshot.staircaseById(node.staircase_id);
    if (staircase == nullptr || !stairLocation(node, staircase).has_value()) {
      return TerrainTrajectoryRejection::STAIR_MODEL_MISSING;
    }
    if (staircase->confidence < limits.min_confidence) {
      return TerrainTrajectoryRejection::LOW_CONFIDENCE;
    }
    if (limits.require_manual_corridor &&
      !hasFlag(node, perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR))
    {
      return TerrainTrajectoryRejection::OUTSIDE_CORRIDOR;
    }
  }
  return TerrainTrajectoryRejection::NONE;
}

Eigen::Quaterniond terrainOrientation(
  const Eigen::Vector3d & input_normal,
  double yaw_rad,
  bool * valid) noexcept
{
  *valid = false;
  Eigen::Vector3d normal = input_normal;
  if (!normal.allFinite() || normal.norm() <= 1e-9 || !std::isfinite(yaw_rad)) {
    return Eigen::Quaterniond::Identity();
  }
  normal.normalize();
  const Eigen::Vector3d horizontal_heading(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
  Eigen::Vector3d forward = horizontal_heading - normal * horizontal_heading.dot(normal);
  if (!forward.allFinite() || forward.norm() <= 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  forward.normalize();
  Eigen::Vector3d left = normal.cross(forward);
  if (!left.allFinite() || left.norm() <= 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  left.normalize();
  Eigen::Matrix3d rotation;
  rotation.col(0) = forward;
  rotation.col(1) = left;
  rotation.col(2) = normal;
  Eigen::Quaterniond orientation(rotation);
  orientation.normalize();
  *valid = orientation.coeffs().allFinite();
  return orientation;
}

}  // namespace

const char * toString(TerrainTrajectoryRejection reason) noexcept
{
  switch (reason) {
    case TerrainTrajectoryRejection::NONE: return "NONE";
    case TerrainTrajectoryRejection::TERRAIN_DISABLED: return "TERRAIN_DISABLED";
    case TerrainTrajectoryRejection::FAIL_OPEN_CONFIGURATION:
      return "FAIL_OPEN_CONFIGURATION";
    case TerrainTrajectoryRejection::MISSING_SNAPSHOT: return "MISSING_SNAPSHOT";
    case TerrainTrajectoryRejection::INVALID_SNAPSHOT: return "INVALID_SNAPSHOT";
    case TerrainTrajectoryRejection::STATIC_GROUND_GENERATION_MISSING:
      return "STATIC_GROUND_GENERATION_MISSING";
    case TerrainTrajectoryRejection::SNAPSHOT_GROUND_MISMATCH:
      return "SNAPSHOT_GROUND_MISMATCH";
    case TerrainTrajectoryRejection::SNAPSHOT_CHANGED: return "SNAPSHOT_CHANGED";
    case TerrainTrajectoryRejection::EMPTY_TRAJECTORY: return "EMPTY_TRAJECTORY";
    case TerrainTrajectoryRejection::INVALID_GEOMETRY: return "INVALID_GEOMETRY";
    case TerrainTrajectoryRejection::NO_SUPPORT: return "NO_SUPPORT";
    case TerrainTrajectoryRejection::UNKNOWN: return "UNKNOWN";
    case TerrainTrajectoryRejection::EDGE: return "EDGE";
    case TerrainTrajectoryRejection::DROP: return "DROP";
    case TerrainTrajectoryRejection::RISER: return "RISER";
    case TerrainTrajectoryRejection::LOW_CONFIDENCE: return "LOW_CONFIDENCE";
    case TerrainTrajectoryRejection::LOW_SUPPORT: return "LOW_SUPPORT";
    case TerrainTrajectoryRejection::INVALID_NORMAL: return "INVALID_NORMAL";
    case TerrainTrajectoryRejection::NORMAL_CHANGE: return "NORMAL_CHANGE";
    case TerrainTrajectoryRejection::SURFACE_CHANGE: return "SURFACE_CHANGE";
    case TerrainTrajectoryRejection::STAIR_DISABLED: return "STAIR_DISABLED";
    case TerrainTrajectoryRejection::STAIR_MODEL_MISSING: return "STAIR_MODEL_MISSING";
    case TerrainTrajectoryRejection::OUTSIDE_CORRIDOR: return "OUTSIDE_CORRIDOR";
    case TerrainTrajectoryRejection::ONLINE_CONFIRMATION_MISSING:
      return "ONLINE_CONFIRMATION_MISSING";
    case TerrainTrajectoryRejection::SKIP_STEP: return "SKIP_STEP";
    case TerrainTrajectoryRejection::TRANSITION_WAYPOINT_MISSING:
      return "TRANSITION_WAYPOINT_MISSING";
    case TerrainTrajectoryRejection::TRANSITION_WAYPOINT_MISSED:
      return "TRANSITION_WAYPOINT_MISSED";
    case TerrainTrajectoryRejection::NO_LANDING: return "NO_LANDING";
    case TerrainTrajectoryRejection::DIRECTION_DISABLED: return "DIRECTION_DISABLED";
    case TerrainTrajectoryRejection::BAD_ALIGNMENT: return "BAD_ALIGNMENT";
  }
  return "INVALID_TERRAIN_TRAJECTORY_REJECTION";
}

bool terrainTrajectoryProjectionLimitsValid(
  const TerrainTrajectoryProjectionLimits & limits) noexcept
{
  return
    std::isfinite(limits.max_support_xy_distance_m) &&
    limits.max_support_xy_distance_m > 0.0 &&
    std::isfinite(limits.max_support_vertical_distance_m) &&
    limits.max_support_vertical_distance_m >= 0.0 &&
    std::isfinite(limits.min_support_ratio) && limits.min_support_ratio >= 0.0 &&
    limits.min_support_ratio <= 1.0 &&
    std::isfinite(limits.min_confidence) && limits.min_confidence >= 0.0 &&
    limits.min_confidence <= 1.0 &&
    std::isfinite(limits.min_upward_normal_z) && limits.min_upward_normal_z > 0.0 &&
    limits.min_upward_normal_z <= 1.0 &&
    std::isfinite(limits.max_normal_change_rad) && limits.max_normal_change_rad >= 0.0 &&
    limits.max_normal_change_rad <= kPi &&
    std::isfinite(limits.stair_transition_waypoint_tolerance_m) &&
    limits.stair_transition_waypoint_tolerance_m >= 0.0 &&
    std::isfinite(limits.max_stair_transition_span_m) &&
    limits.max_stair_transition_span_m > 0.0 &&
    std::isfinite(limits.max_stair_heading_error_rad) &&
    limits.max_stair_heading_error_rad >= 0.0 &&
    limits.max_stair_heading_error_rad <= kPi;
}

std::size_t TerrainProjectionContext::CellKeyHash::operator()(
  const CellKey & key) const noexcept
{
  const std::size_t first = std::hash<std::int64_t>{}(key.x);
  const std::size_t second = std::hash<std::int64_t>{}(key.y);
  return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) + (first >> 2U));
}

std::size_t TerrainProjectionContext::StairLocationKeyHash::operator()(
  const StairLocationKey & key) const noexcept
{
  const std::size_t first = std::hash<std::int32_t>{}(key.staircase_id);
  const std::size_t second = std::hash<std::int32_t>{}(key.location);
  return first ^ (second + 0x9e3779b9U + (first << 6U) + (first >> 2U));
}

std::size_t TerrainProjectionContext::StairTransitionKeyHash::operator()(
  const StairTransitionKey & key) const noexcept
{
  const std::size_t first = std::hash<std::int32_t>{}(key.staircase_id);
  const std::size_t second = std::hash<std::int32_t>{}(key.low_location);
  return first ^ (second + 0x9e3779b9U + (first << 6U) + (first >> 2U));
}

TerrainProjectionContext::TerrainProjectionContext(
  perception_3d::TerrainSnapshotConstPtr snapshot,
  std::uint64_t static_ground_generation,
  std::vector<Eigen::Vector3d> ground_points,
  double spatial_index_resolution_m)
: snapshot_(std::move(snapshot)),
  static_ground_generation_(static_ground_generation),
  ground_points_(std::move(ground_points)),
  spatial_index_resolution_m_(spatial_index_resolution_m)
{
  if (!snapshot_) {
    validation_error_ = "terrain snapshot is missing";
    return;
  }
  if (!snapshot_->valid(&validation_error_)) {
    return;
  }
  if (static_ground_generation_ == 0U) {
    validation_error_ = "static ground generation is zero";
    return;
  }
  if (!std::isfinite(spatial_index_resolution_m_) || spatial_index_resolution_m_ <= 0.0) {
    validation_error_ = "spatial index resolution is invalid";
    return;
  }
  if (ground_points_.empty() || ground_points_.size() != snapshot_->nodes().size()) {
    validation_error_ = "mapground and terrain snapshot sizes differ";
    return;
  }
  for (const auto & point : ground_points_) {
    if (!point.allFinite()) {
      validation_error_ = "mapground contains a non-finite point";
      return;
    }
  }
  buildSpatialIndex();
  buildStairTransitionWaypoints();
  valid_ = true;
  validation_error_.clear();
}

TerrainProjectionContext::CellKey TerrainProjectionContext::cellFor(
  double x, double y) const noexcept
{
  return CellKey{
    static_cast<std::int64_t>(std::floor(x / spatial_index_resolution_m_)),
    static_cast<std::int64_t>(std::floor(y / spatial_index_resolution_m_))};
}

void TerrainProjectionContext::buildSpatialIndex()
{
  spatial_index_.reserve(ground_points_.size());
  for (std::size_t index = 0U; index < ground_points_.size(); ++index) {
    const auto & point = ground_points_[index];
    spatial_index_[cellFor(point.x(), point.y())].push_back(index);
  }
}

void TerrainProjectionContext::buildStairTransitionWaypoints()
{
  using GroupMap = std::unordered_map<
    StairLocationKey, std::vector<std::size_t>, StairLocationKeyHash>;
  GroupMap verified_groups;

  for (std::size_t index = 0U; index < snapshot_->nodes().size(); ++index) {
    const auto & node = snapshot_->nodes()[index];
    if (!hasFlag(node, perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR) ||
      !hasFlag(node, perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED))
    {
      continue;
    }
    const auto * staircase = snapshot_->staircaseById(node.staircase_id);
    const auto location = stairLocation(node, staircase);
    if (!location.has_value()) {
      continue;
    }
    verified_groups[StairLocationKey{node.staircase_id, *location}].push_back(index);
  }

  const auto nearestTo = [this](
      const std::vector<std::size_t> & candidates,
      const Eigen::Vector2d & target) -> std::size_t
    {
      std::size_t best = candidates.front();
      double best_distance = std::numeric_limits<double>::infinity();
      for (const std::size_t index : candidates) {
        const double distance = (ground_points_[index].head<2>() - target).squaredNorm();
        if (distance < best_distance) {
          best = index;
          best_distance = distance;
        }
      }
      return best;
    };

  for (const auto & staircase : snapshot_->staircases()) {
    for (std::int32_t low_location = -1;
      low_location < staircase.step_count; ++low_location)
    {
      const auto low_group_it = verified_groups.find(
        StairLocationKey{staircase.id, low_location});
      const auto high_group_it = verified_groups.find(
        StairLocationKey{staircase.id, low_location + 1});
      if (low_group_it == verified_groups.end() || high_group_it == verified_groups.end() ||
        low_group_it->second.empty() || high_group_it->second.empty())
      {
        continue;
      }

      Eigen::Vector2d high_centroid = Eigen::Vector2d::Zero();
      for (const std::size_t index : high_group_it->second) {
        high_centroid += ground_points_[index].head<2>();
      }
      high_centroid /= static_cast<double>(high_group_it->second.size());

      std::size_t low_index = nearestTo(low_group_it->second, high_centroid);
      std::size_t high_index = nearestTo(
        high_group_it->second, ground_points_[low_index].head<2>());
      // One deterministic refinement avoids selecting the wrong side of a
      // broad landing while keeping context construction linear in group size.
      low_index = nearestTo(low_group_it->second, ground_points_[high_index].head<2>());
      high_index = nearestTo(high_group_it->second, ground_points_[low_index].head<2>());

      const Eigen::Vector2d low_point = ground_points_[low_index].head<2>();
      const Eigen::Vector2d high_point = ground_points_[high_index].head<2>();
      StairTransitionWaypoint waypoint;
      waypoint.position = 0.5 * (low_point + high_point);
      waypoint.support_span_m = (high_point - low_point).norm();
      stair_transition_waypoints_[StairTransitionKey{staircase.id, low_location}] = waypoint;
    }
  }
}

bool terrainProjectionContextIdentityMatches(
  const TerrainProjectionContextConstPtr & leased,
  const TerrainProjectionContextConstPtr & current) noexcept
{
  if (!leased || !current || leased.get() != current.get() ||
    leased->snapshot().get() != current->snapshot().get())
  {
    return false;
  }
  return leased->snapshot() && current->snapshot() &&
    leased->snapshot()->version() == current->snapshot()->version() &&
    leased->staticGroundGeneration() == current->staticGroundGeneration();
}

TerrainTrajectoryProjector::TerrainTrajectoryProjector(
  TerrainTrajectoryProjectionLimits limits)
: limits_(std::move(limits))
{
}

TerrainTrajectoryProjectionResult TerrainTrajectoryProjector::project(
  const TerrainProjectionContextConstPtr & context,
  const TerrainBodyPose & reference_body_pose,
  double body_clearance_m,
  const std::vector<TerrainBodyPose> & future_body_poses) const
{
  TerrainTrajectoryProjectionResult result;
  const auto reject = [&result](
      TerrainTrajectoryRejection rejection,
      std::size_t pose_index) -> TerrainTrajectoryProjectionResult
    {
      result.rejection = rejection;
      result.rejected_pose_index = pose_index;
      result.poses.clear();
      return result;
    };

  if (!terrainTrajectoryProjectionLimitsValid(limits_)) {
    return reject(TerrainTrajectoryRejection::FAIL_OPEN_CONFIGURATION, 0U);
  }
  if (!context || !context->snapshot_) {
    return reject(TerrainTrajectoryRejection::MISSING_SNAPSHOT, 0U);
  }
  if (context->static_ground_generation_ == 0U) {
    return reject(TerrainTrajectoryRejection::STATIC_GROUND_GENERATION_MISSING, 0U);
  }
  if (!context->snapshot_->valid()) {
    return reject(TerrainTrajectoryRejection::INVALID_SNAPSHOT, 0U);
  }
  if (!context->valid_ || context->ground_points_.size() != context->snapshot_->nodes().size()) {
    return reject(TerrainTrajectoryRejection::SNAPSHOT_GROUND_MISMATCH, 0U);
  }
  if (future_body_poses.empty()) {
    return reject(TerrainTrajectoryRejection::EMPTY_TRAJECTORY, 0U);
  }
  if (!reference_body_pose.position.allFinite() ||
    !std::isfinite(reference_body_pose.yaw_rad) ||
    !std::isfinite(body_clearance_m) || body_clearance_m < 0.0)
  {
    return reject(TerrainTrajectoryRejection::INVALID_GEOMETRY, 0U);
  }

  const auto nearestSupport = [&context, this](
      const Eigen::Vector3d & body_position,
      double expected_ground_z) -> std::optional<std::size_t>
    {
      const double xy_radius = limits_.max_support_xy_distance_m;
      const double xy_radius_squared = xy_radius * xy_radius;
      const auto min_cell = context->cellFor(
        body_position.x() - xy_radius, body_position.y() - xy_radius);
      const auto max_cell = context->cellFor(
        body_position.x() + xy_radius, body_position.y() + xy_radius);
      std::optional<std::size_t> best_index;
      double best_xy_distance = std::numeric_limits<double>::infinity();
      double best_z_distance = std::numeric_limits<double>::infinity();

      for (std::int64_t cell_x = min_cell.x; cell_x <= max_cell.x; ++cell_x) {
        for (std::int64_t cell_y = min_cell.y; cell_y <= max_cell.y; ++cell_y) {
          const auto bucket = context->spatial_index_.find(
            TerrainProjectionContext::CellKey{cell_x, cell_y});
          if (bucket == context->spatial_index_.end()) {
            continue;
          }
          for (const std::size_t index : bucket->second) {
            const auto & point = context->ground_points_[index];
            const double xy_distance = squaredDistanceXY(point, body_position);
            const double z_distance = std::abs(point.z() - expected_ground_z);
            if (xy_distance > xy_radius_squared ||
              z_distance > limits_.max_support_vertical_distance_m)
            {
              continue;
            }
            if (xy_distance + kDistanceTieTolerance < best_xy_distance ||
              (std::abs(xy_distance - best_xy_distance) <= kDistanceTieTolerance &&
              z_distance < best_z_distance))
            {
              best_index = index;
              best_xy_distance = xy_distance;
              best_z_distance = z_distance;
            }
          }
        }
      }
      return best_index;
    };

  const auto reference_support = nearestSupport(
    reference_body_pose.position,
    reference_body_pose.position.z() - body_clearance_m);
  if (!reference_support.has_value()) {
    return reject(TerrainTrajectoryRejection::NO_SUPPORT, 0U);
  }
  const auto * previous_node = context->snapshot_->nodeAt(*reference_support);
  if (previous_node == nullptr) {
    return reject(TerrainTrajectoryRejection::SNAPSHOT_GROUND_MISMATCH, 0U);
  }
  TerrainTrajectoryRejection node_rejection = validateNode(
    *context->snapshot_, *previous_node, limits_);
  if (node_rejection != TerrainTrajectoryRejection::NONE) {
    return reject(node_rejection, 0U);
  }

  Eigen::Vector3d previous_ground = context->ground_points_[*reference_support];
  TerrainBodyPose previous_body = reference_body_pose;
  Eigen::Vector3d previous_normal = previous_node->normal.cast<double>().normalized();
  result.poses.reserve(future_body_poses.size());

  const auto validateTopology = [this, &context](
      const perception_3d::TerrainNode & from_node,
      const perception_3d::TerrainNode & to_node,
      const TerrainBodyPose & from_body,
      const TerrainBodyPose & to_body) -> TerrainTrajectoryRejection
    {
      const bool from_tread = isStairTread(from_node);
      const bool to_tread = isStairTread(to_node);
      if (!from_tread && !to_tread) {
        if (from_node.surface_id != to_node.surface_id) {
          return TerrainTrajectoryRejection::SURFACE_CHANGE;
        }
        return TerrainTrajectoryRejection::NONE;
      }
      if (!limits_.stairs_enabled) {
        return TerrainTrajectoryRejection::STAIR_DISABLED;
      }
      if (from_node.staircase_id < 0 || from_node.staircase_id != to_node.staircase_id) {
        return (from_tread != to_tread || isLanding(from_node) || isLanding(to_node)) ?
          TerrainTrajectoryRejection::NO_LANDING :
          TerrainTrajectoryRejection::STAIR_MODEL_MISSING;
      }

      const auto * staircase = context->snapshot_->staircaseById(from_node.staircase_id);
      const auto from_location = stairLocation(from_node, staircase);
      const auto to_location = stairLocation(to_node, staircase);
      if (staircase == nullptr || !from_location.has_value() || !to_location.has_value()) {
        return (from_tread != to_tread) ?
          TerrainTrajectoryRejection::NO_LANDING :
          TerrainTrajectoryRejection::STAIR_MODEL_MISSING;
      }

      const Eigen::Vector2d movement =
        to_body.position.head<2>() - from_body.position.head<2>();
      const double movement_norm = movement.norm();
      if (movement_norm <= 1e-6) {
        return TerrainTrajectoryRejection::BAD_ALIGNMENT;
      }
      Eigen::Vector2d up_axis = staircase->up_axis.head<2>().cast<double>();
      if (!up_axis.allFinite() || up_axis.norm() <= 1e-9) {
        return TerrainTrajectoryRejection::STAIR_MODEL_MISSING;
      }
      up_axis.normalize();

      const std::int32_t location_delta = *to_location - *from_location;
      Eigen::Vector2d desired_axis = up_axis;
      if (location_delta < 0 || (location_delta == 0 && movement.dot(up_axis) < 0.0)) {
        desired_axis = -up_axis;
      }
      const bool ascending = desired_axis.dot(up_axis) > 0.0;
      if ((ascending && !staircase->allow_up) || (!ascending && !staircase->allow_down)) {
        return TerrainTrajectoryRejection::DIRECTION_DISABLED;
      }
      const double desired_yaw = std::atan2(desired_axis.y(), desired_axis.x());
      if (wrappedAngleDifference(from_body.yaw_rad, desired_yaw) >
          limits_.max_stair_heading_error_rad ||
        wrappedAngleDifference(to_body.yaw_rad, desired_yaw) >
          limits_.max_stair_heading_error_rad)
      {
        return TerrainTrajectoryRejection::BAD_ALIGNMENT;
      }

      if (location_delta == 0) {
        return TerrainTrajectoryRejection::NONE;
      }
      if (std::abs(location_delta) > 1) {
        return TerrainTrajectoryRejection::SKIP_STEP;
      }

      const std::int32_t low_location = std::min(*from_location, *to_location);
      const auto waypoint = context->stair_transition_waypoints_.find(
        TerrainProjectionContext::StairTransitionKey{staircase->id, low_location});
      if (waypoint == context->stair_transition_waypoints_.end() ||
        waypoint->second.support_span_m > limits_.max_stair_transition_span_m)
      {
        return TerrainTrajectoryRejection::TRANSITION_WAYPOINT_MISSING;
      }
      const double waypoint_distance = pointToSegmentDistance(
        waypoint->second.position,
        from_body.position.head<2>(), to_body.position.head<2>());
      if (waypoint_distance > limits_.stair_transition_waypoint_tolerance_m) {
        return TerrainTrajectoryRejection::TRANSITION_WAYPOINT_MISSED;
      }
      return TerrainTrajectoryRejection::NONE;
    };

  for (std::size_t pose_index = 0U; pose_index < future_body_poses.size(); ++pose_index) {
    const auto & requested_pose = future_body_poses[pose_index];
    if (!requested_pose.position.allFinite() || !std::isfinite(requested_pose.yaw_rad)) {
      return reject(TerrainTrajectoryRejection::INVALID_GEOMETRY, pose_index);
    }
    const auto support_index = nearestSupport(requested_pose.position, previous_ground.z());
    if (!support_index.has_value()) {
      return reject(TerrainTrajectoryRejection::NO_SUPPORT, pose_index);
    }
    const auto * node = context->snapshot_->nodeAt(*support_index);
    if (node == nullptr) {
      return reject(TerrainTrajectoryRejection::SNAPSHOT_GROUND_MISMATCH, pose_index);
    }
    node_rejection = validateNode(*context->snapshot_, *node, limits_);
    if (node_rejection != TerrainTrajectoryRejection::NONE) {
      return reject(node_rejection, pose_index);
    }

    const Eigen::Vector3d normal = node->normal.cast<double>().normalized();
    const double normal_angle = std::acos(std::clamp(previous_normal.dot(normal), -1.0, 1.0));
    if (!std::isfinite(normal_angle) || normal_angle > limits_.max_normal_change_rad) {
      return reject(TerrainTrajectoryRejection::NORMAL_CHANGE, pose_index);
    }
    const TerrainTrajectoryRejection topology_rejection = validateTopology(
      *previous_node, *node, previous_body, requested_pose);
    if (topology_rejection != TerrainTrajectoryRejection::NONE) {
      return reject(topology_rejection, pose_index);
    }

    bool orientation_valid = false;
    const Eigen::Quaterniond orientation = terrainOrientation(
      normal, requested_pose.yaw_rad, &orientation_valid);
    if (!orientation_valid) {
      return reject(TerrainTrajectoryRejection::INVALID_NORMAL, pose_index);
    }
    TerrainProjectedPose projected;
    projected.position = requested_pose.position;
    projected.position.z() = context->ground_points_[*support_index].z() + body_clearance_m;
    projected.orientation = orientation;
    projected.ground_normal = normal;
    projected.support_ratio = node->support_ratio;
    projected.confidence = node->confidence;
    projected.ground_index = *support_index;
    projected.surface_id = node->surface_id;
    projected.staircase_id = node->staircase_id;
    projected.step_index = node->step_index;
    projected.terrain_class = node->terrain_class;
    result.poses.push_back(projected);

    previous_node = node;
    previous_ground = context->ground_points_[*support_index];
    previous_body = requested_pose;
    previous_normal = normal;
  }

  result.rejection = TerrainTrajectoryRejection::NONE;
  result.rejected_pose_index = 0U;
  return result;
}

}  // namespace trajectory_generators
