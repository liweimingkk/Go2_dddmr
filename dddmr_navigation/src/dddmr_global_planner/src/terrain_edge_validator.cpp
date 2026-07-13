/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include <global_planner/terrain_edge_validator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

namespace global_planner
{
namespace
{

constexpr float kGeometryEpsilon = 1.0e-5F;
constexpr float kHalfPi = 1.57079632679489661923F;
constexpr float kPi = 3.14159265358979323846F;
constexpr std::size_t kMaximumSupportSegments = 4096U;

bool finiteNonnegative(float value)
{
  return std::isfinite(value) && value >= 0.0F;
}

bool finiteRatio(float value)
{
  return finiteNonnegative(value) && value <= 1.0F;
}

bool finitePoint(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Eigen::Vector3f eigenPoint(const pcl::PointXYZI & point)
{
  return Eigen::Vector3f(point.x, point.y, point.z);
}

perception_3d::TerrainEdgeResult rejectedResult(
  perception_3d::TerrainRejectionReason reason)
{
  perception_3d::TerrainEdgeResult result;
  result.accepted = false;
  result.reason = reason;
  return result;
}

bool setError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool policyConfigIsValid(const perception_3d::TerrainEdgePolicyConfig & config)
{
  const float nonnegative_values[] = {
    config.max_up_slope_rad,
    config.max_down_slope_rad,
    config.max_cross_slope_rad,
    config.max_roughness_m,
    config.max_normal_change_rad,
    config.max_step_up_m,
    config.max_step_down_m,
    config.max_support_sample_spacing_m,
    config.max_stair_riser_height_m,
    config.min_stair_tread_depth_m,
    config.max_stair_tread_depth_m,
    config.max_stair_riser_deviation_m,
    config.max_stair_tread_deviation_m,
    config.max_stair_heading_error_rad,
    config.distance_cost_weight,
    config.slope_cost_weight,
    config.cross_slope_cost_weight,
    config.roughness_cost_weight,
    config.risk_cost_weight,
    config.stair_transition_cost};
  for (const float value : nonnegative_values) {
    if (!finiteNonnegative(value)) {
      return false;
    }
  }
  return config.max_up_slope_rad <= kHalfPi &&
         config.max_down_slope_rad <= kHalfPi &&
         config.max_cross_slope_rad <= kHalfPi &&
         config.max_normal_change_rad <= kPi &&
         config.max_stair_heading_error_rad <= kPi &&
         finiteRatio(config.min_support_ratio) &&
         finiteRatio(config.max_unknown_ratio) &&
         finiteRatio(config.min_confidence) &&
         config.max_step_index_delta >= 0 &&
         config.max_stair_tread_depth_m >= config.min_stair_tread_depth_m;
}

perception_3d::TerrainSupportSample sampleFromNode(
  float fraction,
  const perception_3d::TerrainNode & node)
{
  perception_3d::TerrainSupportSample sample;
  sample.edge_fraction = fraction;
  sample.support_ratio = node.support_ratio;
  sample.unknown_ratio = std::clamp(1.0F - node.support_ratio, 0.0F, 1.0F);
  sample.confidence = node.confidence;
  sample.normal = node.normal;
  sample.roughness_m = node.roughness_m;
  sample.terrain_class = node.terrain_class;
  sample.surface_id = node.surface_id;
  sample.staircase_id = node.staircase_id;
  sample.step_index = node.step_index;
  sample.flags = node.flags;
  return sample;
}

}  // namespace

planner_safety::PlanningDataBinding capturePlanningDataBinding(
  const std::shared_ptr<perception_3d::SharedData> & shared_data,
  const bool terrain_enabled)
{
  planner_safety::PlanningDataBinding binding;
  binding.terrain_enabled = terrain_enabled;
  if (!shared_data) {
    return binding;
  }

  const std::unique_lock<std::recursive_mutex> lock(
    shared_data->ground_kdtree_cb_mutex_);
  binding.static_ground_generation = shared_data->getStaticGroundGeneration();
  if (!shared_data->is_static_layer_ready_ || binding.static_ground_generation == 0U) {
    return binding;
  }
  if (!terrain_enabled) {
    binding.valid = true;
    return binding;
  }

  const auto snapshot = shared_data->getTerrainSnapshot();
  if (!snapshot || !snapshot->valid()) {
    return binding;
  }
  binding.map_hash = snapshot->mapHash();
  binding.terrain_snapshot_version = snapshot->version();
  binding.valid = !binding.map_hash.empty() && binding.terrain_snapshot_version != 0U;
  return binding;
}

void TerrainEdgeRejectionStatistics::record(
  const perception_3d::TerrainEdgeResult & result)
{
  ++evaluated;
  if (result.accepted) {
    ++accepted;
    return;
  }
  const auto reason_index = static_cast<std::size_t>(result.reason);
  if (reason_index < rejected_by_reason.size()) {
    ++rejected_by_reason[reason_index];
  }
}

std::string TerrainEdgeRejectionStatistics::toStructuredString() const
{
  std::ostringstream stream;
  stream << "terrain_edge_stats enabled=" << (terrain_enabled ? 1 : 0)
         << " map_hash=" << (map_hash.empty() ? "none" : map_hash)
         << " snapshot_version=" << snapshot_version
         << " evaluated=" << evaluated
         << " accepted=" << accepted
         << " rejected=" << rejected();
  for (std::size_t index = 0; index < rejected_by_reason.size(); ++index) {
    if (rejected_by_reason[index] == 0U) {
      continue;
    }
    const auto reason = static_cast<perception_3d::TerrainRejectionReason>(index);
    stream << " reason." << perception_3d::toString(reason) << '=' << rejected_by_reason[index];
  }
  return stream.str();
}

TerrainEdgeValidator::TerrainEdgeValidator(TerrainEdgeValidatorConfig config)
: config_(std::move(config))
{
  configuration_valid_ = validateConfiguration(config_);
}

bool TerrainEdgeValidator::validateConfiguration(
  const TerrainEdgeValidatorConfig & config,
  std::string * error)
{
  if (!policyConfigIsValid(config.policy)) {
    return setError(error, "terrain capability contains an invalid value");
  }
  if (!finiteNonnegative(config.support_sample_spacing_m) ||
    !finiteNonnegative(config.support_search_radius_m) ||
    !finiteNonnegative(config.support_vertical_tolerance_m) ||
    !finiteNonnegative(config.continuous_height_residual_m) ||
    !finiteNonnegative(config.ground_index_alignment_tolerance_m) ||
    config.support_sample_spacing_m <= 0.0F || config.support_search_radius_m <= 0.0F ||
    config.ground_index_alignment_tolerance_m <= 0.0F)
  {
    return setError(error, "terrain support sampling geometry is invalid");
  }
  if (!config.policy.enabled) {
    return true;
  }
  if (!config.policy.fail_closed) {
    return setError(error, "enabled terrain validation must be fail-closed");
  }
  if (config.expected_map_hash.empty()) {
    return setError(error, "enabled terrain validation requires terrain.map_hash");
  }
  if (config.policy.max_support_sample_spacing_m <= 0.0F ||
    config.support_sample_spacing_m > config.policy.max_support_sample_spacing_m +
    kGeometryEpsilon)
  {
    return setError(error, "support sampling spacing exceeds the policy coverage limit");
  }
  // continuous_height_residual_m classifies whether an edge agrees with the
  // fitted continuous surface.  It is a geometry/noise tolerance, not a
  // discrete-step capability.  Generic step limits may therefore remain
  // exactly zero while continuous ramps are enabled.
  if (config.policy.distance_cost_weight < 1.0F) {
    return setError(error, "terrain distance_cost_weight must preserve the A* heuristic");
  }
  return true;
}

TerrainEdgeSearchContext TerrainEdgeValidator::beginSearch(
  const std::shared_ptr<perception_3d::SharedData> & shared_data) const
{
  TerrainEdgeSearchContext context;
  context.statistics_.terrain_enabled = config_.policy.enabled;
  if (!config_.policy.enabled) {
    context.ready_ = true;
    return context;
  }

  auto fail = [&](perception_3d::TerrainRejectionReason reason) {
      context.statistics_.record(rejectedResult(reason));
      return context;
    };
  if (!configuration_valid_) {
    return fail(perception_3d::TerrainRejectionReason::INVALID_CAPABILITY);
  }
  if (!shared_data) {
    return fail(perception_3d::TerrainRejectionReason::MISSING_SNAPSHOT);
  }
  context.snapshot_ = shared_data->getTerrainSnapshot();
  if (!context.snapshot_) {
    return fail(perception_3d::TerrainRejectionReason::MISSING_SNAPSHOT);
  }
  context.statistics_.map_hash = context.snapshot_->mapHash();
  context.statistics_.snapshot_version = context.snapshot_->version();
  if (!context.snapshot_->valid()) {
    return fail(perception_3d::TerrainRejectionReason::INVALID_SNAPSHOT);
  }
  if (context.snapshot_->mapHash() != config_.expected_map_hash) {
    return fail(perception_3d::TerrainRejectionReason::MAP_MISMATCH);
  }
  if (config_.required_snapshot_version != 0U &&
    context.snapshot_->version() != config_.required_snapshot_version)
  {
    return fail(perception_3d::TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH);
  }

  context.ground_cloud_ = shared_data->pcl_ground_;
  context.ground_kdtree_ = shared_data->kdtree_ground_;
  if (!context.ground_cloud_ || context.ground_cloud_->empty() || !context.ground_kdtree_ ||
    context.snapshot_->nodes().size() != context.ground_cloud_->points.size())
  {
    return fail(perception_3d::TerrainRejectionReason::INDEX_OUT_OF_RANGE);
  }
  const auto kdtree_input = context.ground_kdtree_->getInputCloud();
  if (!kdtree_input || kdtree_input.get() != context.ground_cloud_.get()) {
    return fail(perception_3d::TerrainRejectionReason::MAP_MISMATCH);
  }
  context.ready_ = true;
  return context;
}

perception_3d::TerrainEdgeMode TerrainEdgeValidator::classifyMode(
  const perception_3d::TerrainSnapshot & snapshot,
  std::uint32_t from_index,
  std::uint32_t to_index,
  const Eigen::Vector3f & from_position,
  const Eigen::Vector3f & to_position) const
{
  const auto * from_node = snapshot.nodeAt(from_index);
  const auto * to_node = snapshot.nodeAt(to_index);
  if (from_node == nullptr || to_node == nullptr) {
    return perception_3d::TerrainEdgeMode::UNKNOWN;
  }
  const bool has_stair_endpoint =
    from_node->terrain_class == perception_3d::TerrainClass::STAIR_TREAD ||
    to_node->terrain_class == perception_3d::TerrainClass::STAIR_TREAD;
  if (has_stair_endpoint &&
    (from_node->staircase_id != to_node->staircase_id ||
    from_node->step_index != to_node->step_index))
  {
    return perception_3d::TerrainEdgeMode::STAIR_TRANSITION;
  }

  Eigen::Vector3f average_normal = from_node->normal.normalized() + to_node->normal.normalized();
  if (!average_normal.allFinite() || average_normal.norm() <= kGeometryEpsilon) {
    return perception_3d::TerrainEdgeMode::UNKNOWN;
  }
  average_normal.normalize();
  if (average_normal.z() < 0.0F) {
    average_normal = -average_normal;
  }
  if (std::abs(average_normal.z()) <= kGeometryEpsilon) {
    return perception_3d::TerrainEdgeMode::UNKNOWN;
  }

  const Eigen::Vector3f difference = to_position - from_position;
  const float predicted_height_change =
    -(average_normal.x() * difference.x() + average_normal.y() * difference.y()) /
    average_normal.z();
  const float height_residual = std::abs(difference.z() - predicted_height_change);
  if (height_residual > config_.continuous_height_residual_m + kGeometryEpsilon) {
    return perception_3d::TerrainEdgeMode::GENERIC_STEP;
  }
  return perception_3d::TerrainEdgeMode::CONTINUOUS_SURFACE;
}

bool TerrainEdgeValidator::buildSupportSamples(
  const TerrainEdgeSearchContext & context,
  const perception_3d::TerrainEdgeRequest & request,
  std::vector<perception_3d::TerrainSupportSample> * samples) const
{
  samples->clear();
  const auto * from_node = context.snapshot_->nodeAt(request.from_index);
  const auto * to_node = context.snapshot_->nodeAt(request.to_index);
  if (from_node == nullptr || to_node == nullptr) {
    return false;
  }
  const Eigen::Vector3f difference = request.to_position - request.from_position;
  const float edge_length = difference.norm();
  if (!std::isfinite(edge_length) || edge_length <= kGeometryEpsilon) {
    return false;
  }
  const double requested_segments = std::ceil(
    static_cast<double>(edge_length) /
    static_cast<double>(config_.support_sample_spacing_m));
  if (!std::isfinite(requested_segments) || requested_segments < 1.0 ||
    requested_segments > static_cast<double>(kMaximumSupportSegments))
  {
    return false;
  }
  const std::size_t segment_count = static_cast<std::size_t>(requested_segments);
  samples->reserve(segment_count + 1U);
  const float kdtree_radius = std::hypot(
    config_.support_search_radius_m, config_.support_vertical_tolerance_m);

  for (std::size_t sample_index = 0; sample_index <= segment_count; ++sample_index) {
    const float fraction = static_cast<float>(sample_index) /
      static_cast<float>(segment_count);
    if (sample_index == 0U) {
      samples->push_back(sampleFromNode(fraction, *from_node));
      continue;
    }
    if (sample_index == segment_count) {
      samples->push_back(sampleFromNode(fraction, *to_node));
      continue;
    }

    const Eigen::Vector3f expected = request.from_position + difference * fraction;
    pcl::PointXYZI query;
    query.x = expected.x();
    query.y = expected.y();
    query.z = expected.z();
    std::vector<int> candidate_indices;
    std::vector<float> candidate_distances;
    context.ground_kdtree_->radiusSearch(
      query, kdtree_radius, candidate_indices, candidate_distances);

    int best_index = -1;
    float best_distance = std::numeric_limits<float>::infinity();
    std::unordered_set<int> seen;
    for (const int candidate_index : candidate_indices) {
      if (candidate_index < 0 || !seen.insert(candidate_index).second ||
        static_cast<std::size_t>(candidate_index) >= context.ground_cloud_->points.size())
      {
        continue;
      }
      const auto & point = context.ground_cloud_->points[candidate_index];
      const auto * node = context.snapshot_->nodeAt(static_cast<std::size_t>(candidate_index));
      if (!finitePoint(point) || node == nullptr) {
        continue;
      }
      const float horizontal_distance = std::hypot(point.x - query.x, point.y - query.y);
      const float vertical_distance = std::abs(point.z - query.z);
      if (horizontal_distance > config_.support_search_radius_m ||
        vertical_distance > config_.support_vertical_tolerance_m)
      {
        continue;
      }
      if ((request.mode == perception_3d::TerrainEdgeMode::CONTINUOUS_SURFACE ||
        request.mode == perception_3d::TerrainEdgeMode::GENERIC_STEP) &&
        node->surface_id != from_node->surface_id)
      {
        continue;
      }
      if (request.mode == perception_3d::TerrainEdgeMode::STAIR_TRANSITION) {
        const bool is_tread =
          node->terrain_class == perception_3d::TerrainClass::STAIR_TREAD;
        const bool is_landing =
          (node->flags & perception_3d::TERRAIN_NODE_LANDING) != 0U &&
          (node->terrain_class == perception_3d::TerrainClass::FLAT ||
          node->terrain_class == perception_3d::TerrainClass::RAMP);
        const bool belongs_to_endpoint_step =
          node->step_index == from_node->step_index ||
          node->step_index == to_node->step_index;
        if (node->staircase_id != from_node->staircase_id ||
          (!is_tread && !is_landing) || !belongs_to_endpoint_step)
        {
          // A riser may be geometrically closest to the diagonal between two
          // treads, but it is never robot support.  Keep searching for a tread
          // or the explicitly typed landing inside the configured envelope.
          continue;
        }
      }
      const float distance = (eigenPoint(point) - expected).squaredNorm();
      if (distance < best_distance) {
        best_distance = distance;
        best_index = candidate_index;
      }
    }
    if (best_index < 0) {
      return false;
    }
    samples->push_back(sampleFromNode(
        fraction, *context.snapshot_->nodeAt(static_cast<std::size_t>(best_index))));
  }
  return samples->size() >= 2U;
}

bool TerrainEdgeValidator::validateEndpoint(
  TerrainEdgeSearchContext & context,
  std::uint32_t index,
  const pcl::PointXYZI & position) const
{
  if (!config_.policy.enabled) {
    return true;
  }
  auto reject_endpoint = [&](perception_3d::TerrainRejectionReason reason) {
      context.statistics_.record(rejectedResult(reason));
      return false;
    };
  if (!context.ready_ || !context.snapshot_ || !context.ground_cloud_) {
    return reject_endpoint(perception_3d::TerrainRejectionReason::MISSING_SNAPSHOT);
  }
  const auto * terrain_node = context.snapshot_->nodeAt(index);
  if (terrain_node == nullptr || index >= context.ground_cloud_->points.size() ||
    !finitePoint(position))
  {
    return reject_endpoint(perception_3d::TerrainRejectionReason::INDEX_OUT_OF_RANGE);
  }
  const float alignment_error =
    (eigenPoint(position) - eigenPoint(context.ground_cloud_->points[index])).norm();
  if (!std::isfinite(alignment_error) ||
    alignment_error > config_.ground_index_alignment_tolerance_m)
  {
    return reject_endpoint(perception_3d::TerrainRejectionReason::MAP_MISMATCH);
  }
  switch (terrain_node->terrain_class) {
    case perception_3d::TerrainClass::UNKNOWN:
      return reject_endpoint(perception_3d::TerrainRejectionReason::UNKNOWN);
    case perception_3d::TerrainClass::EDGE:
    case perception_3d::TerrainClass::DROP:
      return reject_endpoint(perception_3d::TerrainRejectionReason::DROP);
    case perception_3d::TerrainClass::STAIR_RISER:
      return reject_endpoint(perception_3d::TerrainRejectionReason::NO_SUPPORT);
    case perception_3d::TerrainClass::FLAT:
    case perception_3d::TerrainClass::RAMP:
    case perception_3d::TerrainClass::STAIR_TREAD:
      break;
  }
  if (terrain_node->confidence < config_.policy.min_confidence) {
    return reject_endpoint(perception_3d::TerrainRejectionReason::LOW_CONFIDENCE);
  }
  if (terrain_node->support_ratio + kGeometryEpsilon < config_.policy.min_support_ratio) {
    return reject_endpoint(perception_3d::TerrainRejectionReason::NO_SUPPORT);
  }
  return true;
}

TerrainEdgeValidationResult TerrainEdgeValidator::evaluate(
  TerrainEdgeSearchContext & context,
  std::uint32_t from_index,
  std::uint32_t to_index,
  const pcl::PointXYZI & from_position,
  const pcl::PointXYZI & to_position,
  bool dynamic_obstacle) const
{
  TerrainEdgeValidationResult validation;
  if (!config_.policy.enabled) {
    validation.policy_result.accepted = true;
    validation.policy_result.reason = perception_3d::TerrainRejectionReason::NONE;
    validation.policy_result.edge_length_m =
      (eigenPoint(to_position) - eigenPoint(from_position)).norm();
    validation.policy_result.traversal_cost = validation.policy_result.edge_length_m;
    return validation;
  }
  if (!context.ready_ || !context.snapshot_) {
    validation.policy_result = rejectedResult(
      perception_3d::TerrainRejectionReason::MISSING_SNAPSHOT);
    context.statistics_.record(validation.policy_result);
    return validation;
  }
  if (!finitePoint(from_position) || !finitePoint(to_position) ||
    context.snapshot_->nodeAt(from_index) == nullptr ||
    context.snapshot_->nodeAt(to_index) == nullptr ||
    from_index >= context.ground_cloud_->points.size() ||
    to_index >= context.ground_cloud_->points.size())
  {
    validation.policy_result = rejectedResult(
      perception_3d::TerrainRejectionReason::INDEX_OUT_OF_RANGE);
    context.statistics_.record(validation.policy_result);
    return validation;
  }
  const float from_alignment_error =
    (eigenPoint(from_position) - eigenPoint(context.ground_cloud_->points[from_index])).norm();
  const float to_alignment_error =
    (eigenPoint(to_position) - eigenPoint(context.ground_cloud_->points[to_index])).norm();
  if (!std::isfinite(from_alignment_error) || !std::isfinite(to_alignment_error) ||
    from_alignment_error > config_.ground_index_alignment_tolerance_m ||
    to_alignment_error > config_.ground_index_alignment_tolerance_m)
  {
    validation.policy_result = rejectedResult(
      perception_3d::TerrainRejectionReason::MAP_MISMATCH);
    context.statistics_.record(validation.policy_result);
    return validation;
  }

  perception_3d::TerrainEdgeRequest request;
  request.from_index = from_index;
  request.to_index = to_index;
  request.from_position = eigenPoint(from_position);
  request.to_position = eigenPoint(to_position);
  validation.mode = classifyMode(
    *context.snapshot_, from_index, to_index, request.from_position, request.to_position);
  request.mode = validation.mode;
  request.expected_map_hash = context.snapshot_->mapHash();
  request.expected_snapshot_version = context.snapshot_->version();
  request.dynamic_obstacle = dynamic_obstacle;
  const auto * from_node = context.snapshot_->nodeAt(from_index);
  const auto * to_node = context.snapshot_->nodeAt(to_index);
  request.inside_manual_corridor =
    (from_node->flags & perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR) != 0U &&
    (to_node->flags & perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR) != 0U;
  request.online_confirmation =
    (from_node->flags & perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED) != 0U &&
    (to_node->flags & perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED) != 0U;

  if (dynamic_obstacle) {
    // TerrainEdgePolicy checks dynamic obstacles before support samples.  Keep
    // that precedence here as well so diagnostics report the actual blocker
    // and an obstructed edge does not trigger needless KD-tree work.
    validation.policy_result = perception_3d::TerrainEdgePolicy::evaluate(
      context.snapshot_, request, config_.policy);
  } else if (!buildSupportSamples(context, request, &request.support_samples)) {
    validation.policy_result = rejectedResult(
      perception_3d::TerrainRejectionReason::NO_SUPPORT);
  } else {
    validation.policy_result = perception_3d::TerrainEdgePolicy::evaluate(
      context.snapshot_, request, config_.policy);
  }
  context.statistics_.record(validation.policy_result);
  return validation;
}

}  // namespace global_planner
