#include <gtest/gtest.h>

#include "perception_3d/shared_data.h"
#include "perception_3d/terrain_edge_policy.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace perception_3d
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;

TerrainNode makeNode(
  std::uint32_t index,
  TerrainClass terrain_class = TerrainClass::FLAT,
  std::int32_t surface_id = 7)
{
  TerrainNode node;
  node.ground_index = index;
  node.normal = Eigen::Vector3f::UnitZ();
  node.slope_rad = 0.0F;
  node.roughness_m = 0.0F;
  node.support_ratio = 1.0F;
  node.confidence = 1.0F;
  node.surface_id = surface_id;
  node.terrain_class = terrain_class;
  node.flags = TERRAIN_NODE_OBSERVED;
  return node;
}

TerrainSnapshotConstPtr makeSnapshot(
  std::vector<TerrainNode> nodes,
  std::vector<StaircaseModel> staircases = {},
  const std::string & map_hash = "map-sha256",
  std::uint64_t version = 42U)
{
  return std::make_shared<const TerrainSnapshot>(
    map_hash, version, 123456789, std::move(nodes), std::move(staircases));
}

TerrainSupportSample sample(
  float fraction,
  TerrainClass terrain_class = TerrainClass::FLAT,
  std::int32_t surface_id = 7)
{
  TerrainSupportSample value;
  value.edge_fraction = fraction;
  value.support_ratio = 1.0F;
  value.unknown_ratio = 0.0F;
  value.confidence = 1.0F;
  value.terrain_class = terrain_class;
  value.surface_id = surface_id;
  return value;
}

TerrainEdgeRequest continuousRequest()
{
  TerrainEdgeRequest request;
  request.from_index = 0U;
  request.to_index = 1U;
  request.from_position = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
  request.to_position = Eigen::Vector3f(0.5F, 0.0F, 0.0F);
  request.mode = TerrainEdgeMode::CONTINUOUS_SURFACE;
  request.expected_map_hash = "map-sha256";
  request.expected_snapshot_version = 42U;
  request.support_samples = {sample(0.0F), sample(0.5F), sample(1.0F)};
  return request;
}

TerrainEdgePolicyConfig enabledConfig()
{
  TerrainEdgePolicyConfig config;
  config.enabled = true;
  config.max_up_slope_rad = 30.0F * kPi / 180.0F;
  config.max_down_slope_rad = 30.0F * kPi / 180.0F;
  config.max_cross_slope_rad = 20.0F * kPi / 180.0F;
  config.max_roughness_m = 0.05F;
  config.max_normal_change_rad = 10.0F * kPi / 180.0F;
  config.max_step_up_m = 0.20F;
  config.max_step_down_m = 0.20F;
  config.min_support_ratio = 0.80F;
  config.max_unknown_ratio = 0.0F;
  config.min_confidence = 0.90F;
  config.max_support_sample_spacing_m = 0.30F;
  return config;
}

std::pair<TerrainSnapshotConstPtr, TerrainEdgeRequest> stairFixture(
  std::int32_t to_step = 1)
{
  TerrainNode from = makeNode(0U, TerrainClass::STAIR_TREAD, 10);
  from.staircase_id = 3;
  from.step_index = 0;
  from.flags |= TERRAIN_NODE_MANUAL_CORRIDOR | TERRAIN_NODE_ONLINE_CONFIRMED;
  TerrainNode to = makeNode(1U, TerrainClass::STAIR_TREAD, 11);
  to.staircase_id = 3;
  to.step_index = to_step;
  to.flags |= TERRAIN_NODE_MANUAL_CORRIDOR | TERRAIN_NODE_ONLINE_CONFIRMED;

  StaircaseModel staircase;
  staircase.id = 3;
  staircase.map_hash = "map-sha256";
  staircase.up_axis = Eigen::Vector3f::UnitX();
  staircase.lower_landing_center = Eigen::Vector3f(-0.30F, 0.0F, 0.0F);
  staircase.upper_landing_center = Eigen::Vector3f(1.20F, 0.0F, 0.72F);
  staircase.first_riser_center = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  staircase.corridor_polygon_xy = {
    Eigen::Vector2f(-0.10F, -0.60F), Eigen::Vector2f(1.00F, -0.60F),
    Eigen::Vector2f(1.00F, 0.60F), Eigen::Vector2f(-0.10F, 0.60F)};
  staircase.lower_landing_polygon_xy = {
    Eigen::Vector2f(-0.60F, -0.60F), Eigen::Vector2f(0.0F, -0.60F),
    Eigen::Vector2f(0.0F, 0.60F), Eigen::Vector2f(-0.60F, 0.60F)};
  staircase.upper_landing_polygon_xy = {
    Eigen::Vector2f(0.90F, -0.60F), Eigen::Vector2f(1.50F, -0.60F),
    Eigen::Vector2f(1.50F, 0.60F), Eigen::Vector2f(0.90F, 0.60F)};
  staircase.width_m = 1.2F;
  staircase.riser_height_m = 0.18F;
  staircase.tread_depth_m = 0.30F;
  staircase.step_count = 4;
  staircase.confidence = 1.0F;
  staircase.allow_up = true;
  staircase.allow_down = true;

  TerrainEdgeRequest request;
  request.from_index = 0U;
  request.to_index = 1U;
  request.from_position = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
  request.to_position = Eigen::Vector3f(0.30F, 0.0F, 0.18F * to_step);
  request.mode = TerrainEdgeMode::STAIR_TRANSITION;
  request.expected_map_hash = "map-sha256";
  request.expected_snapshot_version = 42U;
  request.inside_manual_corridor = true;
  request.online_confirmation = true;
  auto start_sample = sample(0.0F, TerrainClass::STAIR_TREAD, 10);
  start_sample.staircase_id = 3;
  start_sample.step_index = 0;
  start_sample.flags = from.flags;
  auto end_sample = sample(1.0F, TerrainClass::STAIR_TREAD, 11);
  end_sample.staircase_id = 3;
  end_sample.step_index = to_step;
  end_sample.flags = to.flags;
  request.support_samples = {start_sample, end_sample};

  return {
    makeSnapshot({from, to}, {staircase}),
    request};
}

TerrainEdgePolicyConfig stairConfig()
{
  auto config = enabledConfig();
  config.max_support_sample_spacing_m = 0.60F;
  config.stair_enabled = true;
  config.allow_stair_up = true;
  config.allow_stair_down = true;
  config.max_step_index_delta = 1;
  config.max_stair_riser_height_m = 0.20F;
  config.min_stair_tread_depth_m = 0.25F;
  config.max_stair_tread_depth_m = 0.35F;
  config.max_stair_riser_deviation_m = 0.01F;
  config.max_stair_tread_deviation_m = 0.01F;
  config.max_stair_heading_error_rad = 5.0F * kPi / 180.0F;
  config.stair_transition_cost = 2.0F;
  return config;
}

TEST(TerrainSnapshot, ValidatesStrictGroundOrderingAndStairReferences)
{
  auto valid = makeSnapshot({makeNode(0U), makeNode(1U)});
  EXPECT_TRUE(valid->valid());
  EXPECT_EQ(valid->nodeAt(1U)->ground_index, 1U);
  EXPECT_EQ(valid->nodeAt(2U), nullptr);

  auto out_of_order = makeNode(4U);
  auto invalid = makeSnapshot({makeNode(0U), out_of_order});
  std::string error;
  EXPECT_FALSE(invalid->valid(&error));
  EXPECT_NE(error.find("ordering"), std::string::npos);
}

TEST(TerrainSnapshot, SharedDataPublishesImmutableVersion)
{
  static_assert(std::is_same_v<
    decltype(std::declval<const TerrainSnapshot &>().nodes()),
    const std::vector<TerrainNode> &>);

  SharedData shared_data;
  EXPECT_EQ(shared_data.getTerrainSnapshot(), nullptr);
  auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  shared_data.setTerrainSnapshot(snapshot);
  EXPECT_EQ(shared_data.getTerrainSnapshot(), snapshot);
  EXPECT_EQ(shared_data.getTerrainSnapshot()->version(), 42U);
  shared_data.setTerrainSnapshot(nullptr);
  EXPECT_EQ(shared_data.getTerrainSnapshot(), nullptr);
}

TEST(TerrainSnapshot, ValidatesStairCorridorAndLandingGeometry)
{
  auto [snapshot, request] = stairFixture();
  (void)request;
  ASSERT_TRUE(snapshot->valid());
  auto staircases = snapshot->staircases();
  staircases.front().lower_landing_polygon_xy.clear();
  const TerrainSnapshot missing_landing(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  std::string error;
  EXPECT_FALSE(missing_landing.valid(&error));
  EXPECT_NE(error.find("polygon"), std::string::npos);

  staircases = snapshot->staircases();
  staircases.front().upper_landing_center.x() = 4.0F;
  const TerrainSnapshot center_outside(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_FALSE(center_outside.valid(&error));
  EXPECT_NE(error.find("outside"), std::string::npos);

  staircases = snapshot->staircases();
  staircases.front().upper_landing_center.z() = 0.54F;
  const TerrainSnapshot inconsistent_rise(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_FALSE(inconsistent_rise.valid(&error));
  EXPECT_NE(error.find("step_count"), std::string::npos);

  staircases = snapshot->staircases();
  staircases.front().first_riser_center = Eigen::Vector3f(
    std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.09F);
  const TerrainSnapshot missing_first_riser(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_FALSE(missing_first_riser.valid(&error));
  EXPECT_NE(error.find("first riser"), std::string::npos);

  staircases = snapshot->staircases();
  staircases.front().first_riser_center.z() = 0.0F;
  const TerrainSnapshot wrong_first_riser_height(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_FALSE(wrong_first_riser_height.valid(&error));
  EXPECT_NE(error.find("half a riser"), std::string::npos);

  staircases = snapshot->staircases();
  staircases.front().first_riser_center.z() += 0.0199F;
  const TerrainSnapshot accepted_first_riser_height_tolerance(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_TRUE(accepted_first_riser_height_tolerance.valid(&error)) << error;

  staircases = snapshot->staircases();
  staircases.front().first_riser_center.z() += 0.0201F;
  const TerrainSnapshot rejected_first_riser_height_tolerance(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_FALSE(rejected_first_riser_height_tolerance.valid(&error));
  EXPECT_NE(error.find("half a riser"), std::string::npos);

  staircases = snapshot->staircases();
  staircases.front().first_riser_center.x() = -0.20F;
  const TerrainSnapshot outside_first_riser(
    snapshot->mapHash(), snapshot->version(), snapshot->stampNanoseconds(),
    snapshot->nodes(), staircases);
  EXPECT_FALSE(outside_first_riser.valid(&error));
  EXPECT_NE(error.find("outside the corridor"), std::string::npos);
}

TEST(TerrainEdgePolicy, DefaultsToFailClosed)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  const auto result = TerrainEdgePolicy::evaluate(
    snapshot, continuousRequest(), TerrainEdgePolicyConfig{});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::TERRAIN_DISABLED);

  auto fail_open = enabledConfig();
  fail_open.fail_closed = false;
  const auto fail_open_result = TerrainEdgePolicy::evaluate(
    snapshot, continuousRequest(), fail_open);
  EXPECT_FALSE(fail_open_result.accepted);
  EXPECT_EQ(fail_open_result.reason, TerrainRejectionReason::FAIL_OPEN_CONFIGURATION);
}

TEST(TerrainEdgePolicy, AcceptsOnlyExactlyFlatEdgeWhenSlopeCapabilitiesAreZero)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto config = enabledConfig();
  config.max_up_slope_rad = 0.0F;
  config.max_down_slope_rad = 0.0F;
  config.max_cross_slope_rad = 0.0F;
  config.max_roughness_m = 0.0F;
  config.max_normal_change_rad = 0.0F;

  const auto flat = TerrainEdgePolicy::evaluate(snapshot, continuousRequest(), config);
  ASSERT_TRUE(flat.accepted);
  EXPECT_EQ(flat.reason, TerrainRejectionReason::NONE);
  EXPECT_GE(flat.traversal_cost, 0.0F);

  auto ramp_request = continuousRequest();
  ramp_request.to_position.z() = 0.10F;
  const auto ramp = TerrainEdgePolicy::evaluate(snapshot, ramp_request, config);
  EXPECT_FALSE(ramp.accepted);
  EXPECT_EQ(ramp.reason, TerrainRejectionReason::SLOPE);

  ramp_request.to_position.z() = 1.0e-7F;
  const auto tiny_ramp = TerrainEdgePolicy::evaluate(snapshot, ramp_request, config);
  EXPECT_FALSE(tiny_ramp.accepted);
  EXPECT_EQ(tiny_ramp.reason, TerrainRejectionReason::SLOPE);
}

TEST(TerrainEdgePolicy, AppliesSignedUpAndDownLimits)
{
  auto ramp_from = makeNode(0U, TerrainClass::RAMP);
  auto ramp_to = makeNode(1U, TerrainClass::RAMP);
  const auto snapshot = makeSnapshot({ramp_from, ramp_to});
  auto config = enabledConfig();
  config.max_up_slope_rad = 5.0F * kPi / 180.0F;
  config.max_down_slope_rad = 20.0F * kPi / 180.0F;
  auto request = continuousRequest();
  request.to_position.z() = 0.10F;

  auto result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::SLOPE);

  std::swap(request.from_position, request.to_position);
  result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_TRUE(result.accepted);
}

TEST(TerrainEdgePolicy, RejectsCrossSlopeAndLayerMismatch)
{
  auto first = makeNode(0U);
  auto second = makeNode(1U);
  const float tilt = 12.0F * kPi / 180.0F;
  first.normal = Eigen::AngleAxisf(tilt, Eigen::Vector3f::UnitX()) *
    Eigen::Vector3f::UnitZ();
  second.normal = first.normal;
  auto config = enabledConfig();
  config.max_cross_slope_rad = 5.0F * kPi / 180.0F;
  auto result = TerrainEdgePolicy::evaluate(
    makeSnapshot({first, second}), continuousRequest(), config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::CROSS_SLOPE);

  second = makeNode(1U, TerrainClass::FLAT, 8);
  auto request = continuousRequest();
  request.support_samples.back().surface_id = 8;
  result = TerrainEdgePolicy::evaluate(makeSnapshot({makeNode(0U), second}), request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::LAYER_MISMATCH);
}

TEST(TerrainEdgePolicy, RequiresContinuousSupportCoverage)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto request = continuousRequest();
  request.support_samples = {sample(0.0F), sample(1.0F)};
  auto config = enabledConfig();
  config.max_support_sample_spacing_m = 0.20F;
  auto result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::NO_SUPPORT);

  request = continuousRequest();
  config.max_support_sample_spacing_m = 0.30F;
  request.support_samples[1].terrain_class = TerrainClass::UNKNOWN;
  result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::UNKNOWN);
}

TEST(TerrainEdgePolicy, RejectsUnsafeIntermediateNormalAndRoughness)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto request = continuousRequest();
  auto config = enabledConfig();

  request.support_samples[1].roughness_m = config.max_roughness_m + 0.01F;
  auto result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::ROUGHNESS);

  request = continuousRequest();
  request.support_samples[1].normal =
    Eigen::AngleAxisf(10.0F * kPi / 180.0F, Eigen::Vector3f::UnitX()) *
    Eigen::Vector3f::UnitZ();
  config.max_cross_slope_rad = 5.0F * kPi / 180.0F;
  result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::CROSS_SLOPE);
}

TEST(TerrainEdgePolicy, ZeroGenericStepCapabilityRejectsHeightChange)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto request = continuousRequest();
  request.mode = TerrainEdgeMode::GENERIC_STEP;
  request.to_position = Eigen::Vector3f(0.30F, 0.0F, 0.08F);
  request.support_samples = {sample(0.0F), sample(1.0F)};
  auto config = enabledConfig();
  config.max_support_sample_spacing_m = 0.40F;
  config.max_step_up_m = 0.0F;
  const auto result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::STEP_UP);
}

TEST(TerrainEdgePolicy, GenericStepCannotBorrowSupportFromAnotherSurface)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto request = continuousRequest();
  request.mode = TerrainEdgeMode::GENERIC_STEP;
  request.to_position = Eigen::Vector3f(0.30F, 0.0F, 0.08F);
  request.support_samples = {sample(0.0F), sample(1.0F)};
  request.support_samples[1].surface_id = 99;
  auto config = enabledConfig();
  config.max_support_sample_spacing_m = 0.40F;
  const auto result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::LAYER_MISMATCH);
}

TEST(TerrainEdgePolicy, AcceptsOnlyAdjacentConfirmedStairTransition)
{
  auto [snapshot, request] = stairFixture();
  const auto config = stairConfig();
  const auto first = TerrainEdgePolicy::evaluate(snapshot, request, config);
  ASSERT_TRUE(first.accepted);
  EXPECT_EQ(first.reason, TerrainRejectionReason::NONE);
  EXPECT_GT(first.traversal_cost, config.stair_transition_cost);

  for (int iteration = 0; iteration < 20; ++iteration) {
    const auto repeated = TerrainEdgePolicy::evaluate(snapshot, request, config);
    EXPECT_EQ(repeated.accepted, first.accepted);
    EXPECT_EQ(repeated.reason, first.reason);
    EXPECT_FLOAT_EQ(repeated.traversal_cost, first.traversal_cost);
  }
}

TEST(TerrainEdgePolicy, ConnectsLowerLandingAndFirstTreadInBothDirections)
{
  auto [base_snapshot, base_request] = stairFixture();
  (void)base_request;
  TerrainNode landing = makeNode(0U, TerrainClass::FLAT, 7);
  landing.staircase_id = 3;
  landing.flags |= TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED | TERRAIN_NODE_LANDING |
    TERRAIN_NODE_LOWER_LANDING;
  TerrainNode first_tread = makeNode(1U, TerrainClass::STAIR_TREAD, 10);
  first_tread.staircase_id = 3;
  first_tread.step_index = 0;
  first_tread.flags |= TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED;
  const auto snapshot = makeSnapshot(
    {landing, first_tread}, base_snapshot->staircases());

  TerrainEdgeRequest request;
  request.from_index = 0U;
  request.to_index = 1U;
  request.from_position = Eigen::Vector3f(-0.30F, 0.0F, 0.0F);
  request.to_position = Eigen::Vector3f(0.0F, 0.0F, 0.18F);
  request.mode = TerrainEdgeMode::STAIR_TRANSITION;
  request.expected_map_hash = "map-sha256";
  request.expected_snapshot_version = 42U;
  request.inside_manual_corridor = true;
  request.online_confirmation = true;
  auto landing_sample = sample(0.0F, TerrainClass::FLAT, 7);
  landing_sample.staircase_id = 3;
  landing_sample.flags = landing.flags;
  auto tread_sample = sample(1.0F, TerrainClass::STAIR_TREAD, 10);
  tread_sample.staircase_id = 3;
  tread_sample.step_index = 0;
  tread_sample.flags = first_tread.flags;
  request.support_samples = {landing_sample, tread_sample};

  const auto config = stairConfig();
  EXPECT_TRUE(TerrainEdgePolicy::evaluate(snapshot, request, config).accepted);

  std::swap(request.from_index, request.to_index);
  std::swap(request.from_position, request.to_position);
  std::reverse(request.support_samples.begin(), request.support_samples.end());
  request.support_samples.front().edge_fraction = 0.0F;
  request.support_samples.back().edge_fraction = 1.0F;
  EXPECT_TRUE(TerrainEdgePolicy::evaluate(snapshot, request, config).accepted);

  TerrainNode untyped_landing = landing;
  untyped_landing.flags &= ~TERRAIN_NODE_LOWER_LANDING;
  request.support_samples.back().flags = untyped_landing.flags;
  const auto invalid_snapshot = makeSnapshot(
    {untyped_landing, first_tread}, base_snapshot->staircases());
  const auto missing_landing_side =
    TerrainEdgePolicy::evaluate(invalid_snapshot, request, config);
  EXPECT_FALSE(missing_landing_side.accepted);
  EXPECT_EQ(missing_landing_side.reason, TerrainRejectionReason::NO_LANDING);
}

TEST(TerrainEdgePolicy, ConnectsHighestTreadAndUpperLandingInBothDirections)
{
  auto [base_snapshot, base_request] = stairFixture();
  (void)base_request;
  TerrainNode highest_tread = makeNode(0U, TerrainClass::STAIR_TREAD, 13);
  highest_tread.staircase_id = 3;
  highest_tread.step_index = 3;
  highest_tread.flags |= TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED;
  TerrainNode upper_landing = makeNode(1U, TerrainClass::FLAT, 7);
  upper_landing.staircase_id = 3;
  upper_landing.flags |= TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED | TERRAIN_NODE_LANDING |
    TERRAIN_NODE_UPPER_LANDING;
  const auto snapshot = makeSnapshot(
    {highest_tread, upper_landing}, base_snapshot->staircases());

  TerrainEdgeRequest request;
  request.from_index = 0U;
  request.to_index = 1U;
  request.from_position = Eigen::Vector3f(0.90F, 0.0F, 0.72F);
  request.to_position = Eigen::Vector3f(1.20F, 0.0F, 0.72F);
  request.mode = TerrainEdgeMode::STAIR_TRANSITION;
  request.expected_map_hash = "map-sha256";
  request.expected_snapshot_version = 42U;
  request.inside_manual_corridor = true;
  request.online_confirmation = true;
  auto tread_sample = sample(0.0F, TerrainClass::STAIR_TREAD, 13);
  tread_sample.staircase_id = 3;
  tread_sample.step_index = 3;
  tread_sample.flags = highest_tread.flags;
  auto landing_sample = sample(1.0F, TerrainClass::FLAT, 7);
  landing_sample.staircase_id = 3;
  landing_sample.flags = upper_landing.flags;
  request.support_samples = {tread_sample, landing_sample};

  const auto config = stairConfig();
  EXPECT_TRUE(TerrainEdgePolicy::evaluate(snapshot, request, config).accepted);

  std::swap(request.from_index, request.to_index);
  std::swap(request.from_position, request.to_position);
  std::reverse(request.support_samples.begin(), request.support_samples.end());
  request.support_samples.front().edge_fraction = 0.0F;
  request.support_samples.back().edge_fraction = 1.0F;
  EXPECT_TRUE(TerrainEdgePolicy::evaluate(snapshot, request, config).accepted);
}

TEST(TerrainEdgePolicy, RejectsStairSkipOutsideCorridorAndZeroHeightCapability)
{
  auto [skip_snapshot, skip_request] = stairFixture(2);
  auto config = stairConfig();
  auto result = TerrainEdgePolicy::evaluate(skip_snapshot, skip_request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::SKIP_STEP);

  auto [snapshot, request] = stairFixture();
  request.inside_manual_corridor = false;
  result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::OUTSIDE_CORRIDOR);

  request.inside_manual_corridor = true;
  config.max_stair_riser_height_m = 0.0F;
  result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::STEP_UP);
}

TEST(TerrainEdgePolicy, BindsEveryDecisionToMapAndSnapshotVersion)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto request = continuousRequest();
  const auto config = enabledConfig();
  request.expected_map_hash = "different-map";
  auto result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::MAP_MISMATCH);

  request.expected_map_hash = "map-sha256";
  request.expected_snapshot_version = 43U;
  result = TerrainEdgePolicy::evaluate(snapshot, request, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH);
}

TEST(TerrainEdgePolicy, RejectsDegreeValuesMistakenForRadians)
{
  const auto snapshot = makeSnapshot({makeNode(0U), makeNode(1U)});
  auto config = enabledConfig();
  config.max_up_slope_rad = 15.0F;
  const auto result = TerrainEdgePolicy::evaluate(snapshot, continuousRequest(), config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, TerrainRejectionReason::INVALID_CAPABILITY);
}

}  // namespace
}  // namespace perception_3d
