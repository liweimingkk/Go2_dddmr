#include <gtest/gtest.h>

#include <global_planner/terrain_edge_validator.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr float kPi = 3.14159265358979323846F;

pcl::PointXYZI point(float x, float y, float z)
{
  pcl::PointXYZI result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = 0.0F;
  return result;
}

perception_3d::TerrainNode node(
  std::uint32_t index,
  perception_3d::TerrainClass terrain_class = perception_3d::TerrainClass::FLAT,
  std::int32_t surface_id = 1)
{
  perception_3d::TerrainNode result;
  result.ground_index = index;
  result.normal = Eigen::Vector3f::UnitZ();
  result.slope_rad = 0.0F;
  result.roughness_m = 0.0F;
  result.support_ratio = 1.0F;
  result.confidence = 1.0F;
  result.surface_id = surface_id;
  result.terrain_class = terrain_class;
  result.flags = perception_3d::TERRAIN_NODE_OBSERVED;
  return result;
}

perception_3d::TerrainSnapshotConstPtr snapshot(
  std::vector<perception_3d::TerrainNode> nodes,
  std::vector<perception_3d::StaircaseModel> staircases = {},
  std::uint64_t version = 7U)
{
  return std::make_shared<const perception_3d::TerrainSnapshot>(
    "synthetic-map", version, 1000, std::move(nodes), std::move(staircases));
}

std::shared_ptr<perception_3d::SharedData> sharedData(
  const std::vector<pcl::PointXYZI> & points,
  const perception_3d::TerrainSnapshotConstPtr & terrain_snapshot)
{
  auto shared = std::make_shared<perception_3d::SharedData>();
  shared->pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto & item : points) {
    shared->pcl_ground_->push_back(item);
  }
  shared->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared->kdtree_ground_->setInputCloud(shared->pcl_ground_);
  shared->setTerrainSnapshot(terrain_snapshot);
  return shared;
}

global_planner::TerrainEdgeValidatorConfig enabledConfig()
{
  global_planner::TerrainEdgeValidatorConfig config;
  config.expected_map_hash = "synthetic-map";
  config.support_sample_spacing_m = 0.10F;
  config.support_search_radius_m = 0.16F;
  config.support_vertical_tolerance_m = 0.12F;
  config.continuous_height_residual_m = 0.04F;
  auto & policy = config.policy;
  policy.enabled = true;
  policy.fail_closed = true;
  policy.max_up_slope_rad = 15.0F * kPi / 180.0F;
  policy.max_down_slope_rad = 15.0F * kPi / 180.0F;
  policy.max_cross_slope_rad = 8.0F * kPi / 180.0F;
  policy.max_roughness_m = 0.05F;
  policy.max_normal_change_rad = 10.0F * kPi / 180.0F;
  policy.max_step_up_m = 0.10F;
  policy.max_step_down_m = 0.10F;
  policy.min_support_ratio = 1.0F;
  policy.max_unknown_ratio = 0.0F;
  policy.min_confidence = 1.0F;
  policy.max_support_sample_spacing_m = 0.11F;
  policy.distance_cost_weight = 1.0F;
  policy.slope_cost_weight = 1.0F;
  policy.cross_slope_cost_weight = 1.0F;
  policy.roughness_cost_weight = 1.0F;
  policy.risk_cost_weight = 1.0F;
  return config;
}

perception_3d::StaircaseModel staircaseModel()
{
  perception_3d::StaircaseModel staircase;
  staircase.id = 3;
  staircase.map_hash = "synthetic-map";
  staircase.up_axis = Eigen::Vector3f::UnitX();
  staircase.lower_landing_center = Eigen::Vector3f(-0.3F, 0.0F, 0.0F);
  staircase.upper_landing_center = Eigen::Vector3f(1.2F, 0.0F, 0.72F);
  staircase.first_riser_center = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  staircase.corridor_polygon_xy = {
    Eigen::Vector2f(-0.1F, -0.6F), Eigen::Vector2f(1.0F, -0.6F),
    Eigen::Vector2f(1.0F, 0.6F), Eigen::Vector2f(-0.1F, 0.6F)};
  staircase.lower_landing_polygon_xy = {
    Eigen::Vector2f(-0.6F, -0.6F), Eigen::Vector2f(0.0F, -0.6F),
    Eigen::Vector2f(0.0F, 0.6F), Eigen::Vector2f(-0.6F, 0.6F)};
  staircase.upper_landing_polygon_xy = {
    Eigen::Vector2f(0.9F, -0.6F), Eigen::Vector2f(1.5F, -0.6F),
    Eigen::Vector2f(1.5F, 0.6F), Eigen::Vector2f(0.9F, 0.6F)};
  staircase.width_m = 1.2F;
  staircase.riser_height_m = 0.18F;
  staircase.tread_depth_m = 0.30F;
  staircase.step_count = 4;
  staircase.confidence = 1.0F;
  staircase.allow_up = true;
  staircase.allow_down = true;
  return staircase;
}

global_planner::TerrainEdgeValidatorConfig stairConfig(float sample_spacing)
{
  auto config = enabledConfig();
  config.support_sample_spacing_m = sample_spacing;
  config.support_search_radius_m = 0.35F;
  config.support_vertical_tolerance_m = 0.40F;
  config.policy.max_support_sample_spacing_m = sample_spacing + 0.01F;
  config.policy.stair_enabled = true;
  config.policy.allow_stair_up = true;
  config.policy.allow_stair_down = true;
  config.policy.require_manual_corridor = true;
  config.policy.require_online_confirmation = true;
  config.policy.max_step_index_delta = 1;
  config.policy.max_stair_riser_height_m = 0.20F;
  config.policy.min_stair_tread_depth_m = 0.25F;
  config.policy.max_stair_tread_depth_m = 0.35F;
  config.policy.max_stair_riser_deviation_m = 0.01F;
  config.policy.max_stair_tread_deviation_m = 0.01F;
  config.policy.max_stair_heading_error_rad = 5.0F * kPi / 180.0F;
  config.policy.stair_transition_cost = 2.0F;
  return config;
}

std::vector<perception_3d::TerrainNode> stairNodes(std::int32_t to_step)
{
  auto from = node(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  from.staircase_id = 3;
  from.step_index = 0;
  from.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  auto to = node(1U, perception_3d::TerrainClass::STAIR_TREAD, 11);
  to.staircase_id = 3;
  to.step_index = to_step;
  to.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  return {from, to};
}

TEST(TerrainEdgeValidator, DisabledModePreservesLegacyBehavior)
{
  global_planner::TerrainEdgeValidator validator;
  auto shared = std::make_shared<perception_3d::SharedData>();
  auto context = validator.beginSearch(shared);
  EXPECT_TRUE(context.ready());
  EXPECT_FALSE(context.terrainEnabled());

  const auto result = validator.evaluate(
    context, 100U, 200U, point(0.0F, 0.0F, 0.0F), point(0.5F, 0.0F, 0.0F));
  EXPECT_TRUE(result.policy_result.accepted);
}

TEST(TerrainEdgeValidator, ValidatesFailClosedCapabilityConfiguration)
{
  std::string error;
  EXPECT_TRUE(global_planner::TerrainEdgeValidator::validateConfiguration(
      global_planner::TerrainEdgeValidatorConfig{}, &error));

  auto config = enabledConfig();
  EXPECT_TRUE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config.expected_map_hash.clear();
  EXPECT_FALSE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config = enabledConfig();
  config.policy.fail_closed = false;
  EXPECT_FALSE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config = enabledConfig();
  config.support_sample_spacing_m = 0.2F;
  EXPECT_FALSE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config = enabledConfig();
  config.policy.distance_cost_weight = 0.5F;
  EXPECT_FALSE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config = enabledConfig();
  config.policy.max_up_slope_rad = 15.0F;
  EXPECT_FALSE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config = enabledConfig();
  config.policy.max_stair_heading_error_rad = 4.0F;
  EXPECT_FALSE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
  config = enabledConfig();
  config.policy.max_step_up_m = 0.0F;
  config.policy.max_step_down_m = 0.0F;
  EXPECT_TRUE(global_planner::TerrainEdgeValidator::validateConfiguration(config, &error));
}

TEST(TerrainEdgeValidator, EnabledModeRejectsAMissingSnapshot)
{
  const global_planner::TerrainEdgeValidator validator(enabledConfig());
  auto context = validator.beginSearch(std::make_shared<perception_3d::SharedData>());
  EXPECT_FALSE(context.ready());
  ASSERT_EQ(context.statistics().evaluated, 1U);
  const auto reason_index = static_cast<std::size_t>(
    perception_3d::TerrainRejectionReason::MISSING_SNAPSHOT);
  EXPECT_EQ(context.statistics().rejected_by_reason[reason_index], 1U);
}

TEST(TerrainEdgeValidator, CapturesStaticAndTerrainIdentityAsOneBinding)
{
  auto shared = std::make_shared<perception_3d::SharedData>();
  shared->is_static_layer_ready_ = true;

  auto binding = global_planner::capturePlanningDataBinding(shared, false);
  EXPECT_FALSE(binding.valid);

  const std::uint64_t generation = shared->invalidateTerrainForStaticGroundUpdate();
  binding = global_planner::capturePlanningDataBinding(shared, false);
  EXPECT_TRUE(binding.valid);
  EXPECT_FALSE(binding.terrain_enabled);
  EXPECT_EQ(binding.static_ground_generation, generation);

  EXPECT_TRUE(shared->setTerrainSnapshotForStaticGroundGeneration(
      snapshot({node(0U)}, {}, 23U), generation));
  binding = global_planner::capturePlanningDataBinding(shared, true);
  EXPECT_TRUE(binding.valid);
  EXPECT_TRUE(binding.terrain_enabled);
  EXPECT_EQ(binding.static_ground_generation, generation);
  EXPECT_EQ(binding.map_hash, "synthetic-map");
  EXPECT_EQ(binding.terrain_snapshot_version, 23U);

  shared->invalidateTerrainForStaticGroundUpdate();
  binding = global_planner::capturePlanningDataBinding(shared, true);
  EXPECT_FALSE(binding.valid);
}

TEST(TerrainEdgeValidator, RejectsAnUnboundedSupportSampleCountWithoutAllocating)
{
  auto config = enabledConfig();
  config.support_sample_spacing_m = std::numeric_limits<float>::min();
  config.policy.max_support_sample_spacing_m = std::numeric_limits<float>::min();
  global_planner::TerrainEdgeValidator validator(config);
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F), point(0.50F, 0.0F, 0.0F)};
  auto shared = sharedData(points, snapshot({node(0U), node(1U)}));
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(context, 0U, 1U, points[0], points[1]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::NO_SUPPORT);
}

TEST(TerrainEdgeValidator, RejectsUnsupportedSearchEndpoint)
{
  auto unsupported = node(0U);
  unsupported.support_ratio = 0.0F;
  const std::vector<pcl::PointXYZI> points = {point(0.0F, 0.0F, 0.0F)};
  global_planner::TerrainEdgeValidator validator(enabledConfig());
  auto shared = sharedData(points, snapshot({unsupported}));
  auto context = validator.beginSearch(shared);
  EXPECT_FALSE(validator.validateEndpoint(context, 0U, points[0]));
  const auto reason_index = static_cast<std::size_t>(
    perception_3d::TerrainRejectionReason::NO_SUPPORT);
  EXPECT_EQ(context.statistics().rejected_by_reason[reason_index], 1U);
}

TEST(TerrainEdgeValidator, AcceptsRampAndRejectsExcessSlope)
{
  constexpr float slope = 10.0F * kPi / 180.0F;
  std::vector<pcl::PointXYZI> points;
  std::vector<perception_3d::TerrainNode> nodes;
  const Eigen::Vector3f normal = Eigen::Vector3f(-std::tan(slope), 0.0F, 1.0F).normalized();
  for (std::uint32_t index = 0U; index <= 5U; ++index) {
    const float x = 0.1F * static_cast<float>(index);
    points.push_back(point(x, 0.0F, std::tan(slope) * x));
    auto terrain_node = node(index, perception_3d::TerrainClass::RAMP);
    terrain_node.normal = normal;
    terrain_node.slope_rad = slope;
    nodes.push_back(terrain_node);
  }
  auto shared = sharedData(points, snapshot(nodes));
  auto config = enabledConfig();
  global_planner::TerrainEdgeValidator validator(config);
  auto context = validator.beginSearch(shared);
  auto result = validator.evaluate(context, 0U, 5U, points[0], points[5]);
  EXPECT_TRUE(result.policy_result.accepted);
  EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::CONTINUOUS_SURFACE);

  config.policy.max_up_slope_rad = 5.0F * kPi / 180.0F;
  global_planner::TerrainEdgeValidator strict_validator(config);
  auto strict_context = strict_validator.beginSearch(shared);
  result = strict_validator.evaluate(strict_context, 0U, 5U, points[0], points[5]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::SLOPE);
}

TEST(TerrainEdgeValidator, RejectsCrossSlope)
{
  std::vector<pcl::PointXYZI> points;
  std::vector<perception_3d::TerrainNode> nodes;
  const float cross_slope = 12.0F * kPi / 180.0F;
  const Eigen::Vector3f normal = Eigen::AngleAxisf(
    cross_slope, Eigen::Vector3f::UnitX()) * Eigen::Vector3f::UnitZ();
  for (std::uint32_t index = 0U; index <= 5U; ++index) {
    points.push_back(point(0.1F * index, 0.0F, 0.0F));
    auto terrain_node = node(index);
    terrain_node.normal = normal;
    terrain_node.slope_rad = cross_slope;
    nodes.push_back(terrain_node);
  }
  auto shared = sharedData(points, snapshot(nodes));
  global_planner::TerrainEdgeValidator validator(enabledConfig());
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(context, 0U, 5U, points[0], points[5]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(
    result.policy_result.reason, perception_3d::TerrainRejectionReason::CROSS_SLOPE);
}

TEST(TerrainEdgeValidator, RejectsUnsafeTerrainBetweenSafeEdgeEndpoints)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F),
    point(0.25F, 0.0F, 0.0F),
    point(0.50F, 0.0F, 0.0F)};
  auto evaluate_middle = [&points](
      global_planner::TerrainEdgeValidatorConfig config,
      perception_3d::TerrainNode middle)
    {
      config.support_sample_spacing_m = 0.25F;
      config.support_search_radius_m = 0.05F;
      config.support_vertical_tolerance_m = 0.05F;
      config.policy.max_support_sample_spacing_m = 0.25F;
      global_planner::TerrainEdgeValidator validator(config);
      auto shared = sharedData(points, snapshot({node(0U), middle, node(2U)}));
      auto context = validator.beginSearch(shared);
      return validator.evaluate(context, 0U, 2U, points[0], points[2]);
    };

  auto middle = node(1U, perception_3d::TerrainClass::RAMP);
  constexpr float cross_slope = 12.0F * kPi / 180.0F;
  middle.normal = Eigen::AngleAxisf(
    cross_slope, Eigen::Vector3f::UnitX()) * Eigen::Vector3f::UnitZ();
  middle.slope_rad = cross_slope;
  auto result = evaluate_middle(enabledConfig(), middle);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(
    result.policy_result.reason, perception_3d::TerrainRejectionReason::CROSS_SLOPE);

  middle = node(1U);
  middle.roughness_m = 0.10F;
  result = evaluate_middle(enabledConfig(), middle);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::ROUGHNESS);

  middle = node(1U, perception_3d::TerrainClass::RAMP);
  constexpr float normal_change = 8.0F * kPi / 180.0F;
  middle.normal = Eigen::AngleAxisf(
    -normal_change, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitZ();
  middle.slope_rad = normal_change;
  auto normal_config = enabledConfig();
  normal_config.policy.max_normal_change_rad = 5.0F * kPi / 180.0F;
  result = evaluate_middle(normal_config, middle);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(
    result.policy_result.reason, perception_3d::TerrainRejectionReason::NORMAL_CHANGE);
}

TEST(TerrainEdgeValidator, ClassifiesAndRejectsGenericStepAndDrop)
{
  const std::vector<pcl::PointXYZI> up_points = {
    point(0.0F, 0.0F, 0.0F), point(0.30F, 0.0F, 0.15F)};
  const auto terrain_nodes = std::vector<perception_3d::TerrainNode>{node(0U), node(1U)};
  auto config = enabledConfig();
  config.support_sample_spacing_m = 0.40F;
  config.support_search_radius_m = 0.35F;
  config.support_vertical_tolerance_m = 0.20F;
  config.policy.max_support_sample_spacing_m = 0.40F;
  global_planner::TerrainEdgeValidator validator(config);

  auto shared = sharedData(up_points, snapshot(terrain_nodes));
  auto context = validator.beginSearch(shared);
  auto result = validator.evaluate(context, 0U, 1U, up_points[0], up_points[1]);
  EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::GENERIC_STEP);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::STEP_UP);

  const std::vector<pcl::PointXYZI> down_points = {
    point(0.0F, 0.0F, 0.15F), point(0.30F, 0.0F, 0.0F)};
  shared = sharedData(down_points, snapshot(terrain_nodes));
  context = validator.beginSearch(shared);
  result = validator.evaluate(context, 0U, 1U, down_points[0], down_points[1]);
  EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::GENERIC_STEP);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::STEP_DOWN);

  auto drop_nodes = terrain_nodes;
  drop_nodes[1].terrain_class = perception_3d::TerrainClass::DROP;
  shared = sharedData(down_points, snapshot(drop_nodes));
  context = validator.beginSearch(shared);
  result = validator.evaluate(context, 0U, 1U, down_points[0], down_points[1]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::DROP);
}

TEST(TerrainEdgeValidator, RejectsCrossLayerConnection)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F), point(0.50F, 0.0F, 0.0F)};
  auto first = node(0U, perception_3d::TerrainClass::FLAT, 1);
  auto second = node(1U, perception_3d::TerrainClass::FLAT, 2);
  auto config = enabledConfig();
  config.support_sample_spacing_m = 0.60F;
  config.policy.max_support_sample_spacing_m = 0.60F;
  global_planner::TerrainEdgeValidator validator(config);
  auto shared = sharedData(points, snapshot({first, second}));
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(context, 0U, 1U, points[0], points[1]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(
    result.policy_result.reason, perception_3d::TerrainRejectionReason::LAYER_MISMATCH);
}

TEST(TerrainEdgeValidator, GenericStepCannotBorrowSupportFromAnotherSurface)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F),
    point(0.30F, 0.0F, 0.08F),
    point(0.075F, 0.0F, 0.02F),
    point(0.15F, 0.0F, 0.04F),
    point(0.225F, 0.0F, 0.06F)};
  auto config = enabledConfig();
  config.support_search_radius_m = 0.03F;
  config.support_vertical_tolerance_m = 0.03F;
  global_planner::TerrainEdgeValidator validator(config);
  auto shared = sharedData(
    points, snapshot(
      {node(0U), node(1U), node(2U, perception_3d::TerrainClass::FLAT, 2),
        node(3U, perception_3d::TerrainClass::FLAT, 2),
        node(4U, perception_3d::TerrainClass::FLAT, 2)}));
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(context, 0U, 1U, points[0], points[1]);
  EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::GENERIC_STEP);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::NO_SUPPORT);
}

TEST(TerrainEdgeValidator, RejectsMapgroundIndexMisalignment)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F), point(0.20F, 0.0F, 0.0F)};
  global_planner::TerrainEdgeValidator validator(enabledConfig());
  auto shared = sharedData(points, snapshot({node(0U), node(1U)}));
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(
    context, 0U, 1U, point(0.02F, 0.0F, 0.0F), points[1]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::MAP_MISMATCH);
}

TEST(TerrainEdgeValidator, ReportsDynamicObstacleBeforeMissingIntermediateSupport)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F), point(0.50F, 0.0F, 0.0F)};
  global_planner::TerrainEdgeValidator validator(enabledConfig());
  auto shared = sharedData(points, snapshot({node(0U), node(1U)}));
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(
    context, 0U, 1U, points[0], points[1], true);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(
    result.policy_result.reason,
    perception_3d::TerrainRejectionReason::DYNAMIC_OBSTACLE);
}

TEST(TerrainEdgeValidator, AcceptsAdjacentStairAndRejectsSkippedStep)
{
  const std::vector<pcl::PointXYZI> adjacent_points = {
    point(0.0F, 0.0F, 0.0F), point(0.30F, 0.0F, 0.18F)};
  auto config = stairConfig(0.40F);
  global_planner::TerrainEdgeValidator validator(config);
  auto shared = sharedData(
    adjacent_points, snapshot(stairNodes(1), {staircaseModel()}));
  auto context = validator.beginSearch(shared);
  auto result = validator.evaluate(
    context, 0U, 1U, adjacent_points[0], adjacent_points[1]);
  EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::STAIR_TRANSITION);
  ASSERT_TRUE(result.policy_result.accepted)
    << perception_3d::toString(result.policy_result.reason);

  const std::vector<pcl::PointXYZI> skipped_points = {
    point(0.0F, 0.0F, 0.0F), point(0.60F, 0.0F, 0.36F)};
  config = stairConfig(0.80F);
  global_planner::TerrainEdgeValidator skip_validator(config);
  shared = sharedData(skipped_points, snapshot(stairNodes(2), {staircaseModel()}));
  context = skip_validator.beginSearch(shared);
  result = skip_validator.evaluate(context, 0U, 1U, skipped_points[0], skipped_points[1]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(result.policy_result.reason, perception_3d::TerrainRejectionReason::SKIP_STEP);
}

TEST(TerrainEdgeValidator, IgnoresRiserWhenSamplingAdjacentTreadSupport)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F),
    point(0.15F, 0.0F, 0.09F),
    point(0.14F, 0.0F, 0.0F),
    point(0.30F, 0.0F, 0.18F)};

  auto first_tread = node(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  first_tread.staircase_id = 3;
  first_tread.step_index = 0;
  first_tread.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  auto riser = node(1U, perception_3d::TerrainClass::STAIR_RISER, 10);
  riser.staircase_id = 3;
  riser.step_index = 0;
  riser.support_ratio = 0.0F;
  riser.confidence = 0.0F;
  auto support_tread = node(2U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  support_tread.staircase_id = 3;
  support_tread.step_index = 0;
  support_tread.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  auto second_tread = node(3U, perception_3d::TerrainClass::STAIR_TREAD, 11);
  second_tread.staircase_id = 3;
  second_tread.step_index = 1;
  second_tread.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;

  auto config = stairConfig(0.20F);
  config.support_search_radius_m = 0.08F;
  config.support_vertical_tolerance_m = 0.10F;
  global_planner::TerrainEdgeValidator validator(config);
  auto shared = sharedData(
    points, snapshot(
      {first_tread, riser, support_tread, second_tread}, {staircaseModel()}));
  auto context = validator.beginSearch(shared);
  const auto result = validator.evaluate(context, 0U, 3U, points[0], points[3]);
  ASSERT_TRUE(result.policy_result.accepted)
    << perception_3d::toString(result.policy_result.reason);
  EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::STAIR_TRANSITION);
}

TEST(TerrainEdgeValidator, ConnectsBothStairLandingsInBothDirections)
{
  const auto config = stairConfig(0.40F);
  global_planner::TerrainEdgeValidator validator(config);

  auto verify_both_directions = [&validator](
      const std::vector<pcl::PointXYZI> & points,
      const std::vector<perception_3d::TerrainNode> & nodes)
    {
      auto shared = sharedData(points, snapshot(nodes, {staircaseModel()}));
      auto context = validator.beginSearch(shared);
      ASSERT_TRUE(context.ready());

      auto result = validator.evaluate(context, 0U, 1U, points[0], points[1]);
      ASSERT_TRUE(result.policy_result.accepted)
        << perception_3d::toString(result.policy_result.reason);
      EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::STAIR_TRANSITION);

      result = validator.evaluate(context, 1U, 0U, points[1], points[0]);
      ASSERT_TRUE(result.policy_result.accepted)
        << perception_3d::toString(result.policy_result.reason);
      EXPECT_EQ(result.mode, perception_3d::TerrainEdgeMode::STAIR_TRANSITION);
    };

  auto lower_landing = node(0U, perception_3d::TerrainClass::FLAT, 7);
  lower_landing.staircase_id = 3;
  lower_landing.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED |
    perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_LOWER_LANDING;
  auto first_tread = node(1U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  first_tread.staircase_id = 3;
  first_tread.step_index = 0;
  first_tread.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  verify_both_directions(
    {point(-0.30F, 0.0F, 0.0F), point(0.0F, 0.0F, 0.18F)},
    {lower_landing, first_tread});

  auto highest_tread = node(0U, perception_3d::TerrainClass::STAIR_TREAD, 13);
  highest_tread.staircase_id = 3;
  highest_tread.step_index = 3;
  highest_tread.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED;
  auto upper_landing = node(1U, perception_3d::TerrainClass::FLAT, 7);
  upper_landing.staircase_id = 3;
  upper_landing.flags |= perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED |
    perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_UPPER_LANDING;
  verify_both_directions(
    {point(0.90F, 0.0F, 0.72F), point(1.20F, 0.0F, 0.72F)},
    {highest_tread, upper_landing});
}

TEST(TerrainEdgeValidator, SearchRetainsOneSnapshotVersionAndReportsReasons)
{
  const std::vector<pcl::PointXYZI> points = {
    point(0.0F, 0.0F, 0.0F), point(0.30F, 0.0F, 0.15F)};
  const auto nodes = std::vector<perception_3d::TerrainNode>{node(0U), node(1U)};
  auto config = enabledConfig();
  config.support_sample_spacing_m = 0.40F;
  config.support_search_radius_m = 0.35F;
  config.support_vertical_tolerance_m = 0.20F;
  config.policy.max_support_sample_spacing_m = 0.40F;
  global_planner::TerrainEdgeValidator validator(config);
  auto shared = sharedData(points, snapshot(nodes, {}, 7U));
  auto context = validator.beginSearch(shared);
  shared->setTerrainSnapshot(snapshot(nodes, {}, 8U));

  const auto result = validator.evaluate(context, 0U, 1U, points[0], points[1]);
  EXPECT_FALSE(result.policy_result.accepted);
  EXPECT_EQ(context.statistics().snapshot_version, 7U);
  EXPECT_EQ(context.statistics().evaluated, 1U);
  EXPECT_NE(
    context.statistics().toStructuredString().find("reason.STEP_UP=1"),
    std::string::npos);
}

}  // namespace
