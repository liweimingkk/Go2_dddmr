/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include <global_planner/planner_safety.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace global_planner
{
namespace planner_safety
{
namespace
{

enum class PathSegmentMode : std::uint8_t
{
  INTERPOLATE = 0,
  EXPLICIT_WAYPOINT_TRANSITION
};

constexpr double kMaximumTerrainWaypointMappingTolerance = 0.01;

bool isFinitePclPoint(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double squaredDistance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return dx * dx + dy * dy + dz * dz;
}

geometry_msgs::msg::Quaternion directionQuaternion(double x, double y, double z)
{
  geometry_msgs::msg::Quaternion result;
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    result.w = 1.0;
    return result;
  }

  tf2::Vector3 direction(x / norm, y / norm, z / norm);
  tf2::Quaternion quaternion = tf2::shortestArcQuat(tf2::Vector3(1.0, 0.0, 0.0), direction);
  quaternion.normalize();
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

bool pathIsFiniteAndContinuous(
  const nav_msgs::msg::Path & path,
  double maximum_segment_length)
{
  if (path.poses.empty() || !std::isfinite(maximum_segment_length) ||
    maximum_segment_length <= 0.0)
  {
    return false;
  }

  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & orientation = path.poses[index].pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!isFinitePoint(path.poses[index].pose.position) ||
      !std::isfinite(quaternion_norm) || quaternion_norm <= 1e-6)
    {
      return false;
    }
    if (index > 0) {
      const double distance = std::sqrt(squaredDistance(
          path.poses[index - 1].pose.position, path.poses[index].pose.position));
      if (!std::isfinite(distance) || distance > maximum_segment_length) {
        return false;
      }
    }
  }
  return true;
}

void setReason(std::string * rejection_reason, const char * reason)
{
  if (rejection_reason != nullptr) {
    *rejection_reason = reason;
  }
}

bool hasTerrainFlag(
  const perception_3d::TerrainNode & node,
  perception_3d::TerrainNodeFlag flag)
{
  return (node.flags & static_cast<std::uint32_t>(flag)) != 0U;
}

bool pointOnSegment(
  const Eigen::Vector2f & point,
  const Eigen::Vector2f & first,
  const Eigen::Vector2f & second)
{
  constexpr float tolerance = 1.0e-5F;
  const Eigen::Vector2f segment = second - first;
  const Eigen::Vector2f offset = point - first;
  if (segment.squaredNorm() <= tolerance * tolerance) {
    return offset.squaredNorm() <= tolerance * tolerance;
  }
  const float cross = segment.x() * offset.y() - segment.y() * offset.x();
  if (std::abs(cross) > tolerance) {
    return false;
  }
  const float projection = offset.dot(segment);
  return projection >= -tolerance &&
         projection <= segment.squaredNorm() + tolerance;
}

bool pointInPolygon(
  const Eigen::Vector2f & point,
  const std::vector<Eigen::Vector2f> & polygon)
{
  if (!point.allFinite() || polygon.size() < 3U) {
    return false;
  }
  bool inside = false;
  for (std::size_t index = 0U, previous = polygon.size() - 1U;
    index < polygon.size(); previous = index++)
  {
    const auto & first = polygon[previous];
    const auto & second = polygon[index];
    if (pointOnSegment(point, first, second)) {
      return true;
    }
    const bool crosses_y = (first.y() > point.y()) != (second.y() > point.y());
    if (crosses_y) {
      const float crossing_x =
        (second.x() - first.x()) * (point.y() - first.y()) /
        (second.y() - first.y()) + first.x();
      if (point.x() < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool validateTerrainPathNode(
  const perception_3d::TerrainSnapshot & snapshot,
  const perception_3d::TerrainNode & node,
  const pcl::PointXYZI & position,
  std::string * rejection_reason)
{
  using perception_3d::TerrainClass;
  const bool landing = hasTerrainFlag(node, perception_3d::TERRAIN_NODE_LANDING);
  const bool lower_landing = hasTerrainFlag(
    node, perception_3d::TERRAIN_NODE_LOWER_LANDING);
  const bool upper_landing = hasTerrainFlag(
    node, perception_3d::TERRAIN_NODE_UPPER_LANDING);

  if (node.terrain_class == TerrainClass::UNKNOWN ||
    node.terrain_class == TerrainClass::STAIR_RISER ||
    node.terrain_class == TerrainClass::EDGE ||
    node.terrain_class == TerrainClass::DROP)
  {
    setReason(rejection_reason, "NON_TRAVERSABLE_TERRAIN_NODE");
    return false;
  }
  if ((lower_landing || upper_landing) && !landing) {
    setReason(rejection_reason, "LANDING_SIDE_WITHOUT_LANDING");
    return false;
  }

  if (landing) {
    if ((node.terrain_class != TerrainClass::FLAT &&
      node.terrain_class != TerrainClass::RAMP) || lower_landing == upper_landing ||
      node.staircase_id < 0)
    {
      setReason(rejection_reason, "INVALID_LANDING_METADATA");
      return false;
    }
    const auto * staircase = snapshot.staircaseById(node.staircase_id);
    if (staircase == nullptr) {
      setReason(rejection_reason, "LANDING_STAIRCASE_MISSING");
      return false;
    }
    const Eigen::Vector2f position_xy(position.x, position.y);
    const auto & polygon = lower_landing ?
      staircase->lower_landing_polygon_xy : staircase->upper_landing_polygon_xy;
    if (!pointInPolygon(position_xy, polygon)) {
      setReason(rejection_reason, "LANDING_OUTSIDE_SURVEYED_POLYGON");
      return false;
    }
    return true;
  }

  if (node.terrain_class == TerrainClass::STAIR_TREAD) {
    if (node.staircase_id < 0 || node.step_index < 0) {
      setReason(rejection_reason, "INVALID_STAIR_TREAD_METADATA");
      return false;
    }
    const auto * staircase = snapshot.staircaseById(node.staircase_id);
    if (staircase == nullptr || node.step_index >= staircase->step_count) {
      setReason(rejection_reason, "STAIR_TREAD_MODEL_MISMATCH");
      return false;
    }
    if (!pointInPolygon(
        Eigen::Vector2f(position.x, position.y), staircase->corridor_polygon_xy))
    {
      setReason(rejection_reason, "STAIR_TREAD_OUTSIDE_CORRIDOR");
      return false;
    }
  }
  return true;
}

bool classifyTerrainPathSegment(
  const perception_3d::TerrainSnapshot & snapshot,
  const perception_3d::TerrainNode & from,
  const perception_3d::TerrainNode & to,
  const pcl::PointXYZI & from_position,
  const pcl::PointXYZI & to_position,
  PathSegmentMode * mode,
  std::string * rejection_reason)
{
  using perception_3d::TerrainClass;
  const bool from_tread = from.terrain_class == TerrainClass::STAIR_TREAD;
  const bool to_tread = to.terrain_class == TerrainClass::STAIR_TREAD;
  const bool from_landing = hasTerrainFlag(from, perception_3d::TERRAIN_NODE_LANDING);
  const bool to_landing = hasTerrainFlag(to, perception_3d::TERRAIN_NODE_LANDING);

  if (!from_tread && !to_tread) {
    if (from.surface_id != to.surface_id) {
      setReason(rejection_reason, "CONTINUOUS_SURFACE_ID_MISMATCH");
      return false;
    }
    *mode = PathSegmentMode::INTERPOLATE;
    return true;
  }

  if (from_tread && to_tread) {
    if (from.staircase_id != to.staircase_id) {
      setReason(rejection_reason, "STAIRCASE_ID_MISMATCH");
      return false;
    }
    if (from.step_index == to.step_index) {
      if (from.surface_id != to.surface_id) {
        setReason(rejection_reason, "STAIR_TREAD_SURFACE_ID_MISMATCH");
        return false;
      }
      *mode = PathSegmentMode::INTERPOLATE;
      return true;
    }
    if (std::abs(from.step_index - to.step_index) != 1) {
      setReason(rejection_reason, "STAIR_STEP_SKIP");
      return false;
    }
    if (from.surface_id == to.surface_id) {
      setReason(rejection_reason, "STAIR_TRANSITION_SURFACE_ID_COLLISION");
      return false;
    }
    const auto * staircase = snapshot.staircaseById(from.staircase_id);
    if (staircase == nullptr) {
      setReason(rejection_reason, "STAIRCASE_MODEL_MISSING");
      return false;
    }
    Eigen::Vector2f up_axis = staircase->up_axis.head<2>();
    const Eigen::Vector2f horizontal_difference(
      to_position.x - from_position.x, to_position.y - from_position.y);
    const float vertical_difference = to_position.z - from_position.z;
    const bool ascending = to.step_index > from.step_index;
    if (!up_axis.allFinite() || up_axis.norm() <= 1.0e-6F ||
      !horizontal_difference.allFinite() ||
      horizontal_difference.dot(up_axis.normalized()) * (ascending ? 1.0F : -1.0F) <=
      1.0e-6F || vertical_difference * (ascending ? 1.0F : -1.0F) <= 1.0e-6F)
    {
      setReason(rejection_reason, "STAIR_TRANSITION_DIRECTION_MISMATCH");
      return false;
    }
    *mode = PathSegmentMode::EXPLICIT_WAYPOINT_TRANSITION;
    return true;
  }

  const perception_3d::TerrainNode & tread = from_tread ? from : to;
  const perception_3d::TerrainNode & landing = from_landing ? from : to;
  if ((!from_landing && !to_landing) || (from_landing && to_landing) ||
    tread.staircase_id != landing.staircase_id)
  {
    setReason(rejection_reason, "INVALID_STAIR_LANDING_TRANSITION");
    return false;
  }
  const auto * staircase = snapshot.staircaseById(tread.staircase_id);
  if (staircase == nullptr) {
    setReason(rejection_reason, "STAIRCASE_MODEL_MISSING");
    return false;
  }
  const bool lower_landing = hasTerrainFlag(
    landing, perception_3d::TERRAIN_NODE_LOWER_LANDING);
  const std::int32_t expected_step = lower_landing ? 0 : staircase->step_count - 1;
  if (tread.step_index != expected_step) {
    setReason(rejection_reason, "LANDING_CONNECTED_TO_WRONG_TREAD");
    return false;
  }
  if (tread.surface_id == landing.surface_id) {
    setReason(rejection_reason, "STAIR_LANDING_SURFACE_ID_COLLISION");
    return false;
  }
  Eigen::Vector2f up_axis = staircase->up_axis.head<2>();
  const Eigen::Vector2f horizontal_difference(
    to_position.x - from_position.x, to_position.y - from_position.y);
  const float vertical_difference = to_position.z - from_position.z;
  const bool ascending = lower_landing ? from_landing : to_landing;
  if (!up_axis.allFinite() || up_axis.norm() <= 1.0e-6F ||
    !horizontal_difference.allFinite() ||
    horizontal_difference.dot(up_axis.normalized()) * (ascending ? 1.0F : -1.0F) <=
    1.0e-6F ||
    (lower_landing &&
    vertical_difference * (ascending ? 1.0F : -1.0F) <= 1.0e-6F))
  {
    setReason(rejection_reason, "STAIR_LANDING_DIRECTION_MISMATCH");
    return false;
  }
  *mode = PathSegmentMode::EXPLICIT_WAYPOINT_TRANSITION;
  return true;
}

bool validateTerrainSnapshotBinding(
  const pcl::PointCloud<pcl::PointXYZI> & ground_cloud,
  const perception_3d::TerrainSnapshotConstPtr & terrain_snapshot,
  const PlanningDataBinding & binding,
  std::string * rejection_reason)
{
  if (!binding.valid || !binding.terrain_enabled ||
    binding.static_ground_generation == 0U || binding.map_hash.empty() ||
    binding.terrain_snapshot_version == 0U)
  {
    setReason(rejection_reason, "INVALID_TERRAIN_PATH_BINDING");
    return false;
  }
  if (!terrain_snapshot) {
    setReason(rejection_reason, "MISSING_TERRAIN_SNAPSHOT");
    return false;
  }
  if (!terrain_snapshot->valid()) {
    setReason(rejection_reason, "INVALID_TERRAIN_SNAPSHOT");
    return false;
  }
  if (terrain_snapshot->mapHash() != binding.map_hash) {
    setReason(rejection_reason, "TERRAIN_PATH_MAP_HASH_MISMATCH");
    return false;
  }
  if (terrain_snapshot->version() != binding.terrain_snapshot_version) {
    setReason(rejection_reason, "TERRAIN_PATH_SNAPSHOT_VERSION_MISMATCH");
    return false;
  }
  if (terrain_snapshot->nodes().size() != ground_cloud.points.size()) {
    setReason(rejection_reason, "TERRAIN_GROUND_SIZE_MISMATCH");
    return false;
  }
  return true;
}

bool mapWaypointToUniqueTerrainNode(
  const geometry_msgs::msg::Point & waypoint,
  const pcl::PointCloud<pcl::PointXYZI> & ground_cloud,
  const perception_3d::TerrainSnapshot & terrain_snapshot,
  double mapping_tolerance,
  std::size_t * ground_index,
  std::string * rejection_reason)
{
  if (!isFinitePoint(waypoint) || !std::isfinite(mapping_tolerance) ||
    mapping_tolerance <= 0.0 ||
    mapping_tolerance > kMaximumTerrainWaypointMappingTolerance || ground_index == nullptr)
  {
    setReason(rejection_reason, "INVALID_TERRAIN_WAYPOINT_MAPPING_INPUT");
    return false;
  }
  const double squared_tolerance = mapping_tolerance * mapping_tolerance;
  std::size_t matched_index = 0U;
  std::size_t match_count = 0U;
  for (std::size_t index = 0U; index < ground_cloud.points.size(); ++index) {
    const auto & candidate = ground_cloud.points[index];
    if (!isFinitePclPoint(candidate)) {
      continue;
    }
    const double dx = waypoint.x - candidate.x;
    const double dy = waypoint.y - candidate.y;
    const double dz = waypoint.z - candidate.z;
    if (dx * dx + dy * dy + dz * dz <= squared_tolerance) {
      matched_index = index;
      ++match_count;
      if (match_count > 1U) {
        setReason(rejection_reason, "AMBIGUOUS_TERRAIN_WAYPOINT");
        return false;
      }
    }
  }
  if (match_count != 1U) {
    setReason(rejection_reason, "UNMAPPED_TERRAIN_WAYPOINT");
    return false;
  }
  const auto * node = terrain_snapshot.nodeAt(matched_index);
  if (node == nullptr || !validateTerrainPathNode(
      terrain_snapshot, *node, ground_cloud.points[matched_index], rejection_reason))
  {
    return false;
  }
  *ground_index = matched_index;
  return true;
}

bool isExplicitTerrainTransition(
  const geometry_msgs::msg::Point & from,
  const geometry_msgs::msg::Point & to,
  const pcl::PointCloud<pcl::PointXYZI> & ground_cloud,
  const perception_3d::TerrainSnapshot & terrain_snapshot,
  double waypoint_mapping_tolerance,
  std::string * rejection_reason)
{
  std::size_t from_index = 0U;
  std::size_t to_index = 0U;
  if (!mapWaypointToUniqueTerrainNode(
      from, ground_cloud, terrain_snapshot, waypoint_mapping_tolerance,
      &from_index, rejection_reason) ||
    !mapWaypointToUniqueTerrainNode(
      to, ground_cloud, terrain_snapshot, waypoint_mapping_tolerance,
      &to_index, rejection_reason))
  {
    return false;
  }
  PathSegmentMode mode = PathSegmentMode::INTERPOLATE;
  if (!classifyTerrainPathSegment(
      terrain_snapshot, *terrain_snapshot.nodeAt(from_index),
      *terrain_snapshot.nodeAt(to_index), ground_cloud.points[from_index],
      ground_cloud.points[to_index], &mode, rejection_reason))
  {
    return false;
  }
  if (mode != PathSegmentMode::EXPLICIT_WAYPOINT_TRANSITION) {
    setReason(rejection_reason, "OVERSIZED_NON_STAIR_SEGMENT");
    return false;
  }
  return true;
}

bool pathIsFiniteAndTerrainContinuous(
  const nav_msgs::msg::Path & path,
  double maximum_segment_length,
  const pcl::PointCloud<pcl::PointXYZI> & ground_cloud,
  const perception_3d::TerrainSnapshot & terrain_snapshot,
  double waypoint_mapping_tolerance,
  std::string * rejection_reason)
{
  if (path.poses.empty() || !std::isfinite(maximum_segment_length) ||
    maximum_segment_length <= 0.0 || !std::isfinite(waypoint_mapping_tolerance) ||
    waypoint_mapping_tolerance <= 0.0 ||
    waypoint_mapping_tolerance > kMaximumTerrainWaypointMappingTolerance)
  {
    setReason(rejection_reason, "INVALID_TERRAIN_PATH_CONTINUITY_INPUT");
    return false;
  }
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index].pose;
    const auto & orientation = pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!isFinitePoint(pose.position) || !std::isfinite(quaternion_norm) ||
      quaternion_norm <= 1e-6)
    {
      setReason(rejection_reason, "INVALID_TERRAIN_PATH_POSE");
      return false;
    }
    if (index == 0U) {
      continue;
    }
    const auto & previous = path.poses[index - 1U].pose.position;
    const double distance = std::sqrt(squaredDistance(previous, pose.position));
    if (!std::isfinite(distance)) {
      setReason(rejection_reason, "INVALID_TERRAIN_PATH_SEGMENT");
      return false;
    }
    if (distance > maximum_segment_length && !isExplicitTerrainTransition(
        previous, pose.position, ground_cloud, terrain_snapshot,
        waypoint_mapping_tolerance, rejection_reason))
    {
      return false;
    }
  }
  return true;
}

bool buildPathWithSegmentModes(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<unsigned int> & path_indices,
  const std::vector<PathSegmentMode> & segment_modes,
  const std_msgs::msg::Header & header,
  double interpolation_resolution,
  nav_msgs::msg::Path & output)
{
  output = nav_msgs::msg::Path();
  output.header = header;
  if (path_indices.empty() || segment_modes.size() + 1U != path_indices.size() ||
    !std::isfinite(interpolation_resolution) || interpolation_resolution <= 0.0)
  {
    return false;
  }

  for (const auto index : path_indices) {
    if (index >= cloud.points.size() || !isFinitePclPoint(cloud.points[index])) {
      output.poses.clear();
      return false;
    }
  }

  std::vector<geometry_msgs::msg::Quaternion> orientations(path_indices.size());
  geometry_msgs::msg::Quaternion last_valid_orientation;
  last_valid_orientation.w = 1.0;
  for (std::size_t index = 0; index + 1U < path_indices.size(); ++index) {
    const auto & current = cloud.points[path_indices[index]];
    std::size_t next_index = index + 1U;
    while (next_index < path_indices.size()) {
      const auto & next = cloud.points[path_indices[next_index]];
      const double dx = static_cast<double>(next.x) - current.x;
      const double dy = static_cast<double>(next.y) - current.y;
      const double dz = static_cast<double>(next.z) - current.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) >
        std::numeric_limits<double>::epsilon())
      {
        last_valid_orientation = directionQuaternion(dx, dy, dz);
        break;
      }
      ++next_index;
    }
    orientations[index] = last_valid_orientation;
  }
  orientations.back() = last_valid_orientation;

  for (std::size_t index = 0; index < path_indices.size(); ++index) {
    const auto & current = cloud.points[path_indices[index]];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = current.x;
    pose.pose.position.y = current.y;
    pose.pose.position.z = current.z;
    pose.pose.orientation = orientations[index];
    output.poses.push_back(pose);

    if (index + 1U >= path_indices.size() ||
      segment_modes[index] == PathSegmentMode::EXPLICIT_WAYPOINT_TRANSITION)
    {
      continue;
    }
    const auto & next = cloud.points[path_indices[index + 1U]];
    const double dx = static_cast<double>(next.x) - current.x;
    const double dy = static_cast<double>(next.y) - current.y;
    const double dz = static_cast<double>(next.z) - current.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance <= interpolation_resolution) {
      continue;
    }
    for (double travelled = interpolation_resolution; travelled < distance;
      travelled += interpolation_resolution)
    {
      const double ratio = travelled / distance;
      geometry_msgs::msg::PoseStamped interpolated = pose;
      interpolated.pose.position.x = current.x + dx * ratio;
      interpolated.pose.position.y = current.y + dy * ratio;
      interpolated.pose.position.z = current.z + dz * ratio;
      output.poses.push_back(interpolated);
    }
  }
  return !output.poses.empty();
}

}  // namespace

bool isFinitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool planningBindingsMatch(
  const PlanningDataBinding & expected,
  const PlanningDataBinding & observed,
  std::string * rejection_reason)
{
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  if (!expected.valid || expected.static_ground_generation == 0U) {
    setReason(rejection_reason, "INVALID_EXPECTED_BINDING");
    return false;
  }
  if (!observed.valid || observed.static_ground_generation == 0U) {
    setReason(rejection_reason, "INVALID_OBSERVED_BINDING");
    return false;
  }
  if (expected.terrain_enabled != observed.terrain_enabled) {
    setReason(rejection_reason, "TERRAIN_MODE_MISMATCH");
    return false;
  }
  if (expected.static_ground_generation != observed.static_ground_generation) {
    setReason(rejection_reason, "STATIC_GROUND_GENERATION_MISMATCH");
    return false;
  }
  if (!expected.terrain_enabled) {
    return true;
  }
  if (expected.map_hash.empty() || expected.terrain_snapshot_version == 0U) {
    setReason(rejection_reason, "INVALID_EXPECTED_TERRAIN_BINDING");
    return false;
  }
  if (observed.map_hash.empty() || observed.terrain_snapshot_version == 0U) {
    setReason(rejection_reason, "INVALID_OBSERVED_TERRAIN_BINDING");
    return false;
  }
  if (expected.map_hash != observed.map_hash) {
    setReason(rejection_reason, "TERRAIN_MAP_HASH_MISMATCH");
    return false;
  }
  if (expected.terrain_snapshot_version != observed.terrain_snapshot_version) {
    setReason(rejection_reason, "TERRAIN_SNAPSHOT_VERSION_MISMATCH");
    return false;
  }
  return true;
}

GroundCandidateSelection selectGroundCandidate(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const pcl::PointXYZI & query,
  const std::vector<int> & candidate_indices,
  const GroundCandidateCriteria & criteria)
{
  GroundCandidateSelection selection;
  if (!isFinitePclPoint(query) || !std::isfinite(criteria.max_horizontal_distance) ||
    !std::isfinite(criteria.max_vertical_distance) ||
    !std::isfinite(criteria.layer_separation) ||
    !std::isfinite(criteria.ambiguity_xy_distance) ||
    !std::isfinite(criteria.max_above_query) ||
    criteria.max_horizontal_distance <= 0.0 || criteria.max_vertical_distance < 0.0 ||
    criteria.layer_separation <= 0.0 || criteria.ambiguity_xy_distance < 0.0 ||
    criteria.max_above_query < 0.0)
  {
    selection.status = GroundCandidateStatus::INVALID_INPUT;
    return selection;
  }

  std::vector<unsigned int> valid_indices;
  valid_indices.reserve(candidate_indices.size());
  std::unordered_set<unsigned int> seen;
  double best_distance = std::numeric_limits<double>::infinity();

  for (const int raw_index : candidate_indices) {
    if (raw_index < 0) {
      continue;
    }
    const auto index = static_cast<unsigned int>(raw_index);
    if (index >= cloud.points.size() || !seen.insert(index).second) {
      continue;
    }
    const auto & candidate = cloud.points[index];
    if (!isFinitePclPoint(candidate)) {
      continue;
    }
    const double dx = static_cast<double>(candidate.x) - query.x;
    const double dy = static_cast<double>(candidate.y) - query.y;
    const double dz = static_cast<double>(candidate.z) - query.z;
    const double horizontal_distance = std::hypot(dx, dy);
    if (horizontal_distance > criteria.max_horizontal_distance ||
      std::abs(dz) > criteria.max_vertical_distance || dz > criteria.max_above_query)
    {
      continue;
    }

    valid_indices.push_back(index);
    const double distance = dx * dx + dy * dy + dz * dz;
    if (distance < best_distance) {
      best_distance = distance;
      selection.index = index;
    }
  }

  if (valid_indices.empty()) {
    selection.status = GroundCandidateStatus::NO_CANDIDATE;
    return selection;
  }

  const auto & best = cloud.points[selection.index];
  for (const auto index : valid_indices) {
    if (index == selection.index) {
      continue;
    }
    const auto & candidate = cloud.points[index];
    const double layer_xy_distance = std::hypot(
      static_cast<double>(candidate.x) - best.x,
      static_cast<double>(candidate.y) - best.y);
    if (layer_xy_distance <= criteria.ambiguity_xy_distance &&
      std::abs(static_cast<double>(candidate.z) - best.z) >= criteria.layer_separation)
    {
      selection.status = GroundCandidateStatus::AMBIGUOUS_LAYER;
      return selection;
    }
  }

  selection.status = GroundCandidateStatus::SUCCESS;
  selection.squared_distance = best_distance;
  return selection;
}

bool hasGoalSupport(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<int> & candidate_indices,
  const geometry_msgs::msg::Point & goal,
  double support_layer_z,
  const GoalSupportCriteria & criteria)
{
  if (!isFinitePoint(goal) || !std::isfinite(support_layer_z) ||
    !std::isfinite(criteria.radius) || !std::isfinite(criteria.layer_z_tolerance) ||
    criteria.radius <= 0.0 || criteria.layer_z_tolerance < 0.0 ||
    criteria.minimum_points == 0 || criteria.minimum_sectors == 0 ||
    criteria.minimum_sectors > 4)
  {
    return false;
  }

  std::size_t support_count = 0;
  std::array<bool, 4> occupied_sectors{{false, false, false, false}};
  std::unordered_set<unsigned int> seen;
  for (const int raw_index : candidate_indices) {
    if (raw_index < 0) {
      continue;
    }
    const auto index = static_cast<unsigned int>(raw_index);
    if (index >= cloud.points.size() || !seen.insert(index).second) {
      continue;
    }
    const auto & point = cloud.points[index];
    if (!isFinitePclPoint(point) ||
      std::abs(static_cast<double>(point.z) - support_layer_z) > criteria.layer_z_tolerance)
    {
      continue;
    }
    const double dx = static_cast<double>(point.x) - goal.x;
    const double dy = static_cast<double>(point.y) - goal.y;
    if (std::hypot(dx, dy) > criteria.radius) {
      continue;
    }
    ++support_count;
    const std::size_t sector = (dx < 0.0 ? 2U : 0U) + (dy < 0.0 ? 1U : 0U);
    occupied_sectors[sector] = true;
  }

  const auto sector_count = static_cast<std::size_t>(
    std::count(occupied_sectors.begin(), occupied_sectors.end(), true));
  return support_count >= criteria.minimum_points && sector_count >= criteria.minimum_sectors;
}

bool buildInterpolatedPath(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<unsigned int> & path_indices,
  const std_msgs::msg::Header & header,
  double interpolation_resolution,
  nav_msgs::msg::Path & output)
{
  if (path_indices.empty()) {
    output = nav_msgs::msg::Path();
    output.header = header;
    return false;
  }
  return buildPathWithSegmentModes(
    cloud, path_indices,
    std::vector<PathSegmentMode>(path_indices.size() - 1U, PathSegmentMode::INTERPOLATE),
    header, interpolation_resolution, output);
}

bool buildTerrainAwarePath(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<unsigned int> & path_indices,
  const perception_3d::TerrainSnapshotConstPtr & terrain_snapshot,
  const PlanningDataBinding & binding,
  const std_msgs::msg::Header & header,
  double interpolation_resolution,
  nav_msgs::msg::Path & output,
  std::string * rejection_reason)
{
  output = nav_msgs::msg::Path();
  output.header = header;
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  if (!validateTerrainSnapshotBinding(
      cloud, terrain_snapshot, binding, rejection_reason))
  {
    return false;
  }
  if (path_indices.empty() || !std::isfinite(interpolation_resolution) ||
    interpolation_resolution <= 0.0)
  {
    setReason(rejection_reason, "INVALID_TERRAIN_PATH_INPUT");
    return false;
  }

  for (const auto index : path_indices) {
    const auto * node = terrain_snapshot->nodeAt(index);
    if (index >= cloud.points.size() || node == nullptr ||
      !isFinitePclPoint(cloud.points[index]))
    {
      setReason(rejection_reason, "TERRAIN_PATH_INDEX_OUT_OF_RANGE");
      output.poses.clear();
      return false;
    }
    if (!validateTerrainPathNode(
        *terrain_snapshot, *node, cloud.points[index], rejection_reason))
    {
      output.poses.clear();
      return false;
    }
  }

  std::vector<PathSegmentMode> segment_modes;
  segment_modes.reserve(path_indices.size() - 1U);
  for (std::size_t index = 0U; index + 1U < path_indices.size(); ++index) {
    PathSegmentMode mode = PathSegmentMode::INTERPOLATE;
    if (!classifyTerrainPathSegment(
        *terrain_snapshot, *terrain_snapshot->nodeAt(path_indices[index]),
        *terrain_snapshot->nodeAt(path_indices[index + 1U]),
        cloud.points[path_indices[index]], cloud.points[path_indices[index + 1U]],
        &mode, rejection_reason))
    {
      output.poses.clear();
      return false;
    }
    segment_modes.push_back(mode);
  }

  if (!buildPathWithSegmentModes(
      cloud, path_indices, segment_modes, header, interpolation_resolution, output))
  {
    setReason(rejection_reason, "TERRAIN_PATH_CONSTRUCTION_FAILED");
    output.poses.clear();
    return false;
  }
  return true;
}

bool splicePathsFailClosed(
  const nav_msgs::msg::Path & replanned_prefix,
  const nav_msgs::msg::Path & original_path,
  std::size_t original_pivot,
  double maximum_segment_length,
  nav_msgs::msg::Path & output,
  std::string * rejection_reason)
{
  output = nav_msgs::msg::Path();
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  if (replanned_prefix.poses.empty()) {
    setReason(rejection_reason, "EMPTY_REPLANNED_PREFIX");
    return false;
  }
  if (original_path.poses.empty() || original_pivot >= original_path.poses.size()) {
    setReason(rejection_reason, "INVALID_ORIGINAL_PATH");
    return false;
  }
  if (!replanned_prefix.header.frame_id.empty() && !original_path.header.frame_id.empty() &&
    replanned_prefix.header.frame_id != original_path.header.frame_id)
  {
    setReason(rejection_reason, "FRAME_MISMATCH");
    return false;
  }
  if (!pathIsFiniteAndContinuous(replanned_prefix, maximum_segment_length) ||
    !pathIsFiniteAndContinuous(original_path, maximum_segment_length))
  {
    setReason(rejection_reason, "INVALID_OR_DISCONTINUOUS_PATH");
    return false;
  }

  const auto & prefix_end = replanned_prefix.poses.back().pose.position;
  const auto & pivot = original_path.poses[original_pivot].pose.position;
  const double join_distance = std::sqrt(squaredDistance(prefix_end, pivot));
  if (!std::isfinite(join_distance) || join_distance > maximum_segment_length) {
    setReason(rejection_reason, "SPLICE_GAP");
    return false;
  }

  output = replanned_prefix;
  if (output.header.frame_id.empty()) {
    output.header = original_path.header;
  }
  constexpr double duplicate_tolerance = 1e-6;
  std::size_t tail_index = original_pivot;
  if (join_distance <= duplicate_tolerance) {
    ++tail_index;
  }
  for (; tail_index < original_path.poses.size(); ++tail_index) {
    output.poses.push_back(original_path.poses[tail_index]);
  }

  if (!pathIsFiniteAndContinuous(output, maximum_segment_length)) {
    output.poses.clear();
    setReason(rejection_reason, "DISCONTINUOUS_RESULT");
    return false;
  }
  return true;
}

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
  std::string * rejection_reason)
{
  output = nav_msgs::msg::Path();
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  if (!validateTerrainSnapshotBinding(
      ground_cloud, terrain_snapshot, binding, rejection_reason))
  {
    return false;
  }
  if (replanned_prefix.poses.empty()) {
    setReason(rejection_reason, "EMPTY_REPLANNED_PREFIX");
    return false;
  }
  if (original_path.poses.empty() || original_pivot >= original_path.poses.size()) {
    setReason(rejection_reason, "INVALID_ORIGINAL_PATH");
    return false;
  }
  if (!replanned_prefix.header.frame_id.empty() && !original_path.header.frame_id.empty() &&
    replanned_prefix.header.frame_id != original_path.header.frame_id)
  {
    setReason(rejection_reason, "FRAME_MISMATCH");
    return false;
  }
  if (!pathIsFiniteAndTerrainContinuous(
      replanned_prefix, maximum_segment_length, ground_cloud, *terrain_snapshot,
      waypoint_mapping_tolerance, rejection_reason) ||
    !pathIsFiniteAndTerrainContinuous(
      original_path, maximum_segment_length, ground_cloud, *terrain_snapshot,
      waypoint_mapping_tolerance, rejection_reason))
  {
    output.poses.clear();
    return false;
  }

  const auto & prefix_end = replanned_prefix.poses.back().pose.position;
  const auto & pivot = original_path.poses[original_pivot].pose.position;
  const double join_distance = std::sqrt(squaredDistance(prefix_end, pivot));
  if (!std::isfinite(join_distance)) {
    setReason(rejection_reason, "INVALID_TERRAIN_SPLICE_JOIN");
    return false;
  }
  if (join_distance > maximum_segment_length && !isExplicitTerrainTransition(
      prefix_end, pivot, ground_cloud, *terrain_snapshot,
      waypoint_mapping_tolerance, rejection_reason))
  {
    return false;
  }

  output = replanned_prefix;
  if (output.header.frame_id.empty()) {
    output.header = original_path.header;
  }
  constexpr double duplicate_tolerance = 1e-6;
  std::size_t tail_index = original_pivot;
  if (join_distance <= duplicate_tolerance) {
    ++tail_index;
  }
  for (; tail_index < original_path.poses.size(); ++tail_index) {
    output.poses.push_back(original_path.poses[tail_index]);
  }
  if (!pathIsFiniteAndTerrainContinuous(
      output, maximum_segment_length, ground_cloud, *terrain_snapshot,
      waypoint_mapping_tolerance, rejection_reason))
  {
    output.poses.clear();
    return false;
  }
  return true;
}

}  // namespace planner_safety
}  // namespace global_planner
