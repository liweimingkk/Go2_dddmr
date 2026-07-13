#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <global_planner/global_planner.h>
#include <global_planner/planner_safety.h>

namespace
{

pcl::PointXYZI point(float x, float y, float z)
{
  pcl::PointXYZI result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = 0.0F;
  return result;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = z;
  result.pose.orientation.w = 1.0;
  return result;
}

nav_msgs::msg::Path pathFromPoints(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const std::string & frame = "map")
{
  nav_msgs::msg::Path result;
  result.header.frame_id = frame;
  result.poses = poses;
  for (auto & item : result.poses) {
    item.header = result.header;
  }
  return result;
}

perception_3d::TerrainNode terrainNode(
  std::uint32_t index,
  perception_3d::TerrainClass terrain_class,
  std::int32_t surface_id)
{
  perception_3d::TerrainNode result;
  result.ground_index = index;
  result.normal = Eigen::Vector3f::UnitZ();
  result.support_ratio = 1.0F;
  result.confidence = 1.0F;
  result.surface_id = surface_id;
  result.terrain_class = terrain_class;
  result.flags = perception_3d::TERRAIN_NODE_OBSERVED;
  return result;
}

perception_3d::StaircaseModel terrainPathStaircase()
{
  perception_3d::StaircaseModel staircase;
  staircase.id = 3;
  staircase.map_hash = "terrain-path-map";
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
  staircase.width_m = 1.20F;
  staircase.riser_height_m = 0.18F;
  staircase.tread_depth_m = 0.30F;
  staircase.step_count = 4;
  staircase.confidence = 1.0F;
  staircase.allow_up = true;
  staircase.allow_down = true;
  return staircase;
}

perception_3d::TerrainSnapshotConstPtr terrainPathSnapshot(
  std::vector<perception_3d::TerrainNode> nodes,
  bool include_staircase = true)
{
  std::vector<perception_3d::StaircaseModel> staircases;
  if (include_staircase) {
    staircases.push_back(terrainPathStaircase());
  }
  return std::make_shared<const perception_3d::TerrainSnapshot>(
    "terrain-path-map", 42U, 1000, std::move(nodes), std::move(staircases));
}

global_planner::planner_safety::PlanningDataBinding terrainPathBinding()
{
  global_planner::planner_safety::PlanningDataBinding binding;
  binding.valid = true;
  binding.terrain_enabled = true;
  binding.static_ground_generation = 7U;
  binding.map_hash = "terrain-path-map";
  binding.terrain_snapshot_version = 42U;
  return binding;
}

TEST(PlannerSafety, SelectsAUniqueGroundLayer)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(0.2F, 0.0F, 0.02F));
  pcl::PointXYZI query = point(0.02F, 0.0F, 0.2F);

  global_planner::planner_safety::GroundCandidateCriteria criteria;
  const auto result = global_planner::planner_safety::selectGroundCandidate(
    cloud, query, {0, 1}, criteria);

  EXPECT_EQ(
    result.status,
    global_planner::planner_safety::GroundCandidateStatus::SUCCESS);
  EXPECT_EQ(result.index, 0U);
}

TEST(PlannerSafety, RejectsOverlappingGroundLayers)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(0.02F, 0.01F, 0.45F));
  pcl::PointXYZI query = point(0.0F, 0.0F, 0.2F);

  global_planner::planner_safety::GroundCandidateCriteria criteria;
  criteria.layer_separation = 0.3;
  const auto result = global_planner::planner_safety::selectGroundCandidate(
    cloud, query, {0, 1}, criteria);

  EXPECT_EQ(
    result.status,
    global_planner::planner_safety::GroundCandidateStatus::AMBIGUOUS_LAYER);
}

TEST(PlannerSafety, BoundedProjectionRejectsADeepFloor)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, -1.0F));
  pcl::PointXYZI query = point(0.0F, 0.0F, 0.0F);

  global_planner::planner_safety::GroundCandidateCriteria criteria;
  criteria.max_vertical_distance = 0.5;
  const auto result = global_planner::planner_safety::selectGroundCandidate(
    cloud, query, {0}, criteria);

  EXPECT_EQ(
    result.status,
    global_planner::planner_safety::GroundCandidateStatus::NO_CANDIDATE);
}

TEST(PlannerSafety, GoalSupportMustSurroundTheGoal)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.2F, 0.2F, 0.0F));
  cloud.push_back(point(0.2F, -0.2F, 0.0F));
  cloud.push_back(point(-0.2F, 0.2F, 0.0F));
  cloud.push_back(point(-0.2F, -0.2F, 0.0F));
  geometry_msgs::msg::Point goal;

  global_planner::planner_safety::GoalSupportCriteria criteria;
  EXPECT_TRUE(global_planner::planner_safety::hasGoalSupport(
      cloud, {0, 1, 2, 3}, goal, 0.0, criteria));
  EXPECT_FALSE(global_planner::planner_safety::hasGoalSupport(
      cloud, {0, 1}, goal, 0.0, criteria));
}

TEST(PlannerSafety, PlanningBindingAlwaysRejectsAStaticGroundGenerationChange)
{
  global_planner::planner_safety::PlanningDataBinding expected;
  expected.valid = true;
  expected.static_ground_generation = 7U;
  auto observed = expected;
  observed.static_ground_generation = 8U;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::planningBindingsMatch(
      expected, observed, &reason));
  EXPECT_EQ(reason, "STATIC_GROUND_GENERATION_MISMATCH");
}

TEST(PlannerSafety, TerrainBindingRejectsMapAndSnapshotChanges)
{
  global_planner::planner_safety::PlanningDataBinding expected;
  expected.valid = true;
  expected.terrain_enabled = true;
  expected.static_ground_generation = 4U;
  expected.map_hash = "site-map";
  expected.terrain_snapshot_version = 12U;
  auto observed = expected;
  std::string reason;

  observed.map_hash = "replacement-map";
  EXPECT_FALSE(global_planner::planner_safety::planningBindingsMatch(
      expected, observed, &reason));
  EXPECT_EQ(reason, "TERRAIN_MAP_HASH_MISMATCH");

  observed = expected;
  observed.terrain_snapshot_version = 13U;
  EXPECT_FALSE(global_planner::planner_safety::planningBindingsMatch(
      expected, observed, &reason));
  EXPECT_EQ(reason, "TERRAIN_SNAPSHOT_VERSION_MISMATCH");
}

TEST(PlannerSafety, TerrainDisabledBindingDoesNotRequireASnapshot)
{
  global_planner::planner_safety::PlanningDataBinding expected;
  expected.valid = true;
  expected.static_ground_generation = 9U;
  auto observed = expected;
  observed.map_hash = "ignored-legacy-snapshot";
  observed.terrain_snapshot_version = 55U;
  std::string reason;

  EXPECT_TRUE(global_planner::planner_safety::planningBindingsMatch(
      expected, observed, &reason));
  EXPECT_TRUE(reason.empty());
}

TEST(PlannerSafety, InvalidBindingFailsClosed)
{
  global_planner::planner_safety::PlanningDataBinding invalid;
  auto observed = invalid;
  observed.valid = true;
  observed.static_ground_generation = 1U;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::planningBindingsMatch(
      invalid, observed, &reason));
  EXPECT_EQ(reason, "INVALID_EXPECTED_BINDING");
}

TEST(PlannerSafety, EmptyAndInvalidIndexPathsFailClosed)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  std_msgs::msg::Header header;
  header.frame_id = "map";
  nav_msgs::msg::Path output;

  EXPECT_FALSE(global_planner::planner_safety::buildInterpolatedPath(
      cloud, {}, header, 0.1, output));
  EXPECT_TRUE(output.poses.empty());
  EXPECT_FALSE(global_planner::planner_safety::buildInterpolatedPath(
      cloud, {1}, header, 0.1, output));
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, SingleNodePathHasAValidIdentityOrientation)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(1.0F, 2.0F, 3.0F));
  std_msgs::msg::Header header;
  header.frame_id = "map";
  nav_msgs::msg::Path output;

  ASSERT_TRUE(global_planner::planner_safety::buildInterpolatedPath(
      cloud, {0}, header, 0.1, output));
  ASSERT_EQ(output.poses.size(), 1U);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.orientation.w, 1.0);
}

TEST(PlannerSafety, LastPathPoseInheritsTheFinalSegmentOrientation)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(0.0F, 1.0F, 0.2F));
  std_msgs::msg::Header header;
  header.frame_id = "map";
  nav_msgs::msg::Path output;

  ASSERT_TRUE(global_planner::planner_safety::buildInterpolatedPath(
      cloud, {0, 1}, header, 0.1, output));
  ASSERT_GT(output.poses.size(), 2U);
  const auto & first = output.poses.front().pose.orientation;
  const auto & last = output.poses.back().pose.orientation;
  EXPECT_NEAR(last.x, first.x, 1e-12);
  EXPECT_NEAR(last.y, first.y, 1e-12);
  EXPECT_NEAR(last.z, first.z, 1e-12);
  EXPECT_NEAR(last.w, first.w, 1e-12);
  EXPECT_NEAR(
    std::sqrt(last.x * last.x + last.y * last.y + last.z * last.z + last.w * last.w),
    1.0, 1e-12);
}

TEST(PlannerSafety, TerrainAwarePathInterpolatesAContinuousRamp)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(1.0F, 0.0F, 0.2F));
  auto from = terrainNode(0U, perception_3d::TerrainClass::RAMP, 5);
  auto to = terrainNode(1U, perception_3d::TerrainClass::RAMP, 5);
  const auto snapshot = terrainPathSnapshot({from, to}, false);
  std_msgs::msg::Header header;
  header.frame_id = "map";
  nav_msgs::msg::Path output;
  std::string reason;

  ASSERT_TRUE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U, 1U}, snapshot, terrainPathBinding(), header, 0.25, output, &reason));
  EXPECT_GT(output.poses.size(), 2U);
  EXPECT_GT(output.poses[1].pose.position.z, 0.0);
  EXPECT_LT(output.poses[1].pose.position.z, 0.2);
  EXPECT_TRUE(reason.empty());
}

TEST(PlannerSafety, TerrainAwarePathInterpolatesWithinOneStairTread)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.10F, 0.0F, 0.18F));
  cloud.push_back(point(0.25F, 0.0F, 0.18F));
  auto from = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  from.staircase_id = 3;
  from.step_index = 0;
  auto to = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  to.staircase_id = 3;
  to.step_index = 0;
  const auto snapshot = terrainPathSnapshot({from, to});
  nav_msgs::msg::Path output;

  ASSERT_TRUE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U, 1U}, snapshot, terrainPathBinding(), std_msgs::msg::Header(),
      0.05, output));
  EXPECT_GT(output.poses.size(), 2U);
}

TEST(PlannerSafety, TerrainAwarePathDoesNotInterpolateAcrossAdjacentStairTreads)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.15F, 0.0F, 0.18F));
  cloud.push_back(point(0.45F, 0.0F, 0.36F));
  auto from = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  from.staircase_id = 3;
  from.step_index = 0;
  auto to = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 11);
  to.staircase_id = 3;
  to.step_index = 1;
  const auto snapshot = terrainPathSnapshot({from, to});
  nav_msgs::msg::Path output;

  ASSERT_TRUE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U, 1U}, snapshot, terrainPathBinding(), std_msgs::msg::Header(),
      0.02, output));
  ASSERT_EQ(output.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.z, 0.18F);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.position.z, 0.36F);
}

TEST(PlannerSafety, TerrainAwarePathDoesNotInterpolateLandingTransitions)
{
  auto lower_landing = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  lower_landing.staircase_id = 3;
  lower_landing.flags |= perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_LOWER_LANDING;
  auto first_tread = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  first_tread.staircase_id = 3;
  first_tread.step_index = 0;
  pcl::PointCloud<pcl::PointXYZI> lower_cloud;
  lower_cloud.push_back(point(-0.15F, 0.0F, 0.0F));
  lower_cloud.push_back(point(0.15F, 0.0F, 0.18F));
  nav_msgs::msg::Path output;

  ASSERT_TRUE(global_planner::planner_safety::buildTerrainAwarePath(
      lower_cloud, {0U, 1U}, terrainPathSnapshot({lower_landing, first_tread}),
      terrainPathBinding(), std_msgs::msg::Header(), 0.02, output));
  EXPECT_EQ(output.poses.size(), 2U);

  auto last_tread = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 13);
  last_tread.staircase_id = 3;
  last_tread.step_index = 3;
  auto upper_landing = terrainNode(1U, perception_3d::TerrainClass::FLAT, 2);
  upper_landing.staircase_id = 3;
  upper_landing.flags |= perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_UPPER_LANDING;
  pcl::PointCloud<pcl::PointXYZI> upper_cloud;
  upper_cloud.push_back(point(0.90F, 0.0F, 0.72F));
  upper_cloud.push_back(point(1.20F, 0.0F, 0.72F));

  ASSERT_TRUE(global_planner::planner_safety::buildTerrainAwarePath(
      upper_cloud, {0U, 1U}, terrainPathSnapshot({last_tread, upper_landing}),
      terrainPathBinding(), std_msgs::msg::Header(), 0.02, output));
  EXPECT_EQ(output.poses.size(), 2U);
}

TEST(PlannerSafety, TerrainAwarePathRejectsSkippedStairMetadata)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.15F, 0.0F, 0.18F));
  cloud.push_back(point(0.75F, 0.0F, 0.54F));
  auto from = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  from.staircase_id = 3;
  from.step_index = 0;
  auto to = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 12);
  to.staircase_id = 3;
  to.step_index = 2;
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U, 1U}, terrainPathSnapshot({from, to}), terrainPathBinding(),
      std_msgs::msg::Header(), 0.02, output, &reason));
  EXPECT_EQ(reason, "STAIR_STEP_SKIP");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainAwarePathRejectsAStairDirectionMismatch)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.45F, 0.0F, 0.18F));
  cloud.push_back(point(0.15F, 0.0F, 0.36F));
  auto from = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  from.staircase_id = 3;
  from.step_index = 0;
  auto to = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 11);
  to.staircase_id = 3;
  to.step_index = 1;
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U, 1U}, terrainPathSnapshot({from, to}), terrainPathBinding(),
      std_msgs::msg::Header(), 0.02, output, &reason));
  EXPECT_EQ(reason, "STAIR_TRANSITION_DIRECTION_MISMATCH");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainAwarePathRejectsInconsistentLandingFlags)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(-0.15F, 0.0F, 0.0F));
  auto landing = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  landing.staircase_id = 3;
  landing.flags |= perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_LOWER_LANDING |
    perception_3d::TERRAIN_NODE_UPPER_LANDING;
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U}, terrainPathSnapshot({landing}), terrainPathBinding(),
      std_msgs::msg::Header(), 0.1, output, &reason));
  EXPECT_EQ(reason, "INVALID_LANDING_METADATA");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainAwarePathRejectsSnapshotBindingMismatch)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  auto flat = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  auto binding = terrainPathBinding();
  binding.terrain_snapshot_version = 43U;
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::buildTerrainAwarePath(
      cloud, {0U}, terrainPathSnapshot({flat}, false), binding,
      std_msgs::msg::Header(), 0.1, output, &reason));
  EXPECT_EQ(reason, "TERRAIN_PATH_SNAPSHOT_VERSION_MISMATCH");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, EmptyDwaPrefixCannotReuseTheOldTail)
{
  const auto original = pathFromPoints({pose(0.0, 0.0, 0.0), pose(0.1, 0.0, 0.0)});
  nav_msgs::msg::Path empty_prefix;
  empty_prefix.header.frame_id = "map";
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::splicePathsFailClosed(
      empty_prefix, original, 0, 0.25, output, &reason));
  EXPECT_EQ(reason, "EMPTY_REPLANNED_PREFIX");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, DwaSpliceRejectsAGap)
{
  const auto prefix = pathFromPoints({pose(0.0, 0.0, 0.0), pose(0.1, 0.0, 0.0)});
  const auto original = pathFromPoints({pose(1.0, 0.0, 0.0), pose(1.1, 0.0, 0.0)});
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::splicePathsFailClosed(
      prefix, original, 0, 0.25, output, &reason));
  EXPECT_EQ(reason, "SPLICE_GAP");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, DwaSpliceKeepsAContinuousTailWithoutDuplicatingThePivot)
{
  const auto prefix = pathFromPoints({pose(0.0, 0.0, 0.0), pose(0.1, 0.0, 0.0)});
  const auto original = pathFromPoints(
    {pose(0.0, 0.0, 0.0), pose(0.1, 0.0, 0.0), pose(0.2, 0.0, 0.0)});
  nav_msgs::msg::Path output;
  std::string reason;

  ASSERT_TRUE(global_planner::planner_safety::splicePathsFailClosed(
      prefix, original, 1, 0.25, output, &reason));
  ASSERT_EQ(output.poses.size(), 3U);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.position.x, 0.2);
  EXPECT_TRUE(reason.empty());
}

TEST(PlannerSafety, TerrainDwaSpliceAllowsOnlyABoundAdjacentStairGap)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.15F, 0.0F, 0.18F));
  cloud.push_back(point(0.45F, 0.0F, 0.36F));
  auto first = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  first.staircase_id = 3;
  first.step_index = 0;
  auto second = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 11);
  second.staircase_id = 3;
  second.step_index = 1;
  const auto snapshot = terrainPathSnapshot({first, second});
  const auto prefix = pathFromPoints({pose(0.15, 0.0, 0.18)});
  const auto original = pathFromPoints(
    {pose(0.15, 0.0, 0.18), pose(0.45, 0.0, 0.36)});
  nav_msgs::msg::Path output;
  std::string reason;

  ASSERT_TRUE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      prefix, original, 0U, 0.25, cloud, snapshot, terrainPathBinding(),
      0.001, output, &reason));
  EXPECT_EQ(output.poses.size(), 2U);
  EXPECT_TRUE(reason.empty());
}

TEST(PlannerSafety, TerrainDwaSpliceAllowsABoundLandingTransition)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(-0.15F, 0.0F, 0.0F));
  cloud.push_back(point(0.15F, 0.0F, 0.18F));
  auto landing = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  landing.staircase_id = 3;
  landing.flags |= perception_3d::TERRAIN_NODE_LANDING |
    perception_3d::TERRAIN_NODE_LOWER_LANDING;
  auto tread = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  tread.staircase_id = 3;
  tread.step_index = 0;
  const auto prefix = pathFromPoints({pose(-0.15, 0.0, 0.0)});
  const auto original = pathFromPoints(
    {pose(-0.15, 0.0, 0.0), pose(0.15, 0.0, 0.18)});
  nav_msgs::msg::Path output;

  ASSERT_TRUE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      prefix, original, 0U, 0.25, cloud, terrainPathSnapshot({landing, tread}),
      terrainPathBinding(), 0.001, output));
  EXPECT_EQ(output.poses.size(), 2U);
}

TEST(PlannerSafety, TerrainDwaSpliceRejectsAnOrdinaryOversizedGap)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(1.0F, 0.0F, 0.0F));
  const auto first = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  const auto second = terrainNode(1U, perception_3d::TerrainClass::FLAT, 1);
  const auto prefix = pathFromPoints({pose(0.0, 0.0, 0.0)});
  const auto original = pathFromPoints(
    {pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)});
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      prefix, original, 0U, 0.25, cloud, terrainPathSnapshot({first, second}, false),
      terrainPathBinding(), 0.001, output, &reason));
  EXPECT_EQ(reason, "OVERSIZED_NON_STAIR_SEGMENT");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainDwaSpliceRejectsASkippedStepGap)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.15F, 0.0F, 0.18F));
  cloud.push_back(point(0.75F, 0.0F, 0.54F));
  auto first = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  first.staircase_id = 3;
  first.step_index = 0;
  auto third = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 12);
  third.staircase_id = 3;
  third.step_index = 2;
  const auto prefix = pathFromPoints({pose(0.15, 0.0, 0.18)});
  const auto original = pathFromPoints(
    {pose(0.15, 0.0, 0.18), pose(0.75, 0.0, 0.54)});
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      prefix, original, 0U, 0.25, cloud, terrainPathSnapshot({first, third}),
      terrainPathBinding(), 0.001, output, &reason));
  EXPECT_EQ(reason, "STAIR_STEP_SKIP");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainDwaSpliceRejectsAnAmbiguousWaypointMapping)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.15F, 0.0F, 0.18F));
  cloud.push_back(point(0.1505F, 0.0F, 0.18F));
  cloud.push_back(point(0.45F, 0.0F, 0.36F));
  auto first = terrainNode(0U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  first.staircase_id = 3;
  first.step_index = 0;
  auto duplicate = terrainNode(1U, perception_3d::TerrainClass::STAIR_TREAD, 10);
  duplicate.staircase_id = 3;
  duplicate.step_index = 0;
  auto second = terrainNode(2U, perception_3d::TerrainClass::STAIR_TREAD, 11);
  second.staircase_id = 3;
  second.step_index = 1;
  const auto prefix = pathFromPoints({pose(0.15, 0.0, 0.18)});
  const auto original = pathFromPoints(
    {pose(0.15, 0.0, 0.18), pose(0.45, 0.0, 0.36)});
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      prefix, original, 0U, 0.25, cloud,
      terrainPathSnapshot({first, duplicate, second}), terrainPathBinding(),
      0.001, output, &reason));
  EXPECT_EQ(reason, "AMBIGUOUS_TERRAIN_WAYPOINT");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainDwaSpliceRejectsAStaleSnapshotBinding)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  const auto flat = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  const auto single = pathFromPoints({pose(0.0, 0.0, 0.0)});
  auto binding = terrainPathBinding();
  binding.terrain_snapshot_version = 99U;
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      single, single, 0U, 0.25, cloud, terrainPathSnapshot({flat}, false),
      binding, 0.001, output, &reason));
  EXPECT_EQ(reason, "TERRAIN_PATH_SNAPSHOT_VERSION_MISMATCH");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, TerrainDwaSpliceRejectsAnOverbroadMappingTolerance)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  const auto flat = terrainNode(0U, perception_3d::TerrainClass::FLAT, 1);
  const auto single = pathFromPoints({pose(0.0, 0.0, 0.0)});
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(global_planner::planner_safety::spliceTerrainPathsFailClosed(
      single, single, 0U, 0.25, cloud, terrainPathSnapshot({flat}, false),
      terrainPathBinding(), 0.02, output, &reason));
  EXPECT_EQ(reason, "INVALID_TERRAIN_PATH_CONTINUITY_INPUT");
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, NonFinitePathIsRejected)
{
  auto prefix = pathFromPoints({pose(0.0, 0.0, 0.0)});
  prefix.poses.front().pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto original = pathFromPoints({pose(0.0, 0.0, 0.0)});
  nav_msgs::msg::Path output;

  EXPECT_FALSE(global_planner::planner_safety::splicePathsFailClosed(
      prefix, original, 0, 0.25, output));
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, InvalidPathQuaternionIsRejected)
{
  auto prefix = pathFromPoints({pose(0.0, 0.0, 0.0)});
  prefix.poses.front().pose.orientation.w = 0.0;
  const auto original = pathFromPoints({pose(0.0, 0.0, 0.0)});
  nav_msgs::msg::Path output;

  EXPECT_FALSE(global_planner::planner_safety::splicePathsFailClosed(
      prefix, original, 0, 0.25, output));
  EXPECT_TRUE(output.poses.empty());
}

TEST(PlannerSafety, AstarListGraphUpdateReplacesThePointCloudAndKdTree)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr first(new pcl::PointCloud<pcl::PointXYZI>());
  first->push_back(point(0.0F, 0.0F, 0.0F));
  AstarList list(first);

  pcl::PointCloud<pcl::PointXYZI>::Ptr second(new pcl::PointCloud<pcl::PointXYZI>());
  second->push_back(point(1.0F, 0.0F, 0.0F));
  second->push_back(point(1.1F, 0.0F, 0.0F));
  list.updateGraph(second);

  ASSERT_EQ(list.pc_original_z_up_, second);
  std::vector<int> indices;
  std::vector<float> squared_distances;
  EXPECT_EQ(
    list.kdtree_ground_->radiusSearch(second->front(), 0.2, indices, squared_distances), 2);
}

TEST(PlannerSafety, AstarListSafelyDrainsOnlyStaleClosedEntries)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->push_back(point(0.0F, 0.0F, 0.0F));
  AstarList list(cloud);
  list.Initial();
  Node_t node{0U, 0.0F, 0.0F, 0.0F, 0U, false, true};
  list.updateNode(node);
  list.closeNode(node);

  const auto drained = list.getNode_wi_MinimumF();
  EXPECT_EQ(drained.self_index, std::numeric_limits<unsigned int>::max());
  EXPECT_TRUE(list.isFrontierEmpty());
}

TEST(PlannerSafety, ActionResultCarriesStructuredPlanningIdentityAndStatistics)
{
  global_planner::BoundGlobalPlan plan;
  plan.path = pathFromPoints({pose(0.0, 0.0, 0.0), pose(0.2, 0.0, 0.0)});
  plan.binding.valid = true;
  plan.binding.terrain_enabled = true;
  plan.binding.static_ground_generation = 17U;
  plan.binding.map_hash = "site-map";
  plan.binding.terrain_snapshot_version = 23U;
  plan.status_code = global_planner::PlanStatusCode::NO_PATH;
  plan.status_reason = "NO_PATH:TEST";
  plan.terrain_statistics.terrain_enabled = true;
  plan.terrain_statistics.map_hash = "site-map";
  plan.terrain_statistics.snapshot_version = 23U;
  plan.terrain_statistics.evaluated = 8U;
  plan.terrain_statistics.accepted = 3U;

  dddmr_sys_core::action::GetPlan::Result result;
  global_planner::populateGetPlanResult(plan, result);

  EXPECT_EQ(
    result.status_code,
    dddmr_sys_core::action::GetPlan::Result::STATUS_NO_PATH);
  EXPECT_EQ(result.status_reason, "NO_PATH:TEST");
  EXPECT_TRUE(result.terrain_enabled);
  EXPECT_EQ(result.static_ground_generation, 17U);
  EXPECT_EQ(result.map_hash, "site-map");
  EXPECT_EQ(result.terrain_snapshot_version, 23U);
  EXPECT_EQ(result.evaluated_edge_count, 8U);
  EXPECT_EQ(result.accepted_edge_count, 3U);
  EXPECT_EQ(result.rejected_edge_count, 5U);
  EXPECT_NE(result.rejection_statistics.find("evaluated=8"), std::string::npos);
}

}  // namespace
