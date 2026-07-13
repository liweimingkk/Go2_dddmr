#include <gtest/gtest.h>

#include <perception_3d/cluster_marking.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace perception_3d
{
namespace
{

StaircaseModel staircase()
{
  StaircaseModel value;
  value.id = 3;
  value.map_hash = "map-a";
  value.up_axis = Eigen::Vector3f::UnitX();
  value.lower_landing_center = Eigen::Vector3f(-0.30F, 0.0F, 0.0F);
  value.upper_landing_center = Eigen::Vector3f(1.20F, 0.0F, 0.72F);
  value.first_riser_center = Eigen::Vector3f(0.0F, 0.0F, 0.09F);
  value.corridor_polygon_xy = {
    {-0.10F, -0.60F}, {1.00F, -0.60F}, {1.00F, 0.60F}, {-0.10F, 0.60F}};
  value.lower_landing_polygon_xy = {
    {-0.60F, -0.60F}, {0.0F, -0.60F}, {0.0F, 0.60F}, {-0.60F, 0.60F}};
  value.upper_landing_polygon_xy = {
    {0.90F, -0.60F}, {1.50F, -0.60F}, {1.50F, 0.60F}, {0.90F, 0.60F}};
  value.width_m = 1.20F;
  value.riser_height_m = 0.18F;
  value.tread_depth_m = 0.30F;
  value.step_count = 4;
  value.confidence = 0.98F;
  value.allow_up = true;
  value.allow_down = true;
  return value;
}

TerrainNode riserNode()
{
  TerrainNode node;
  node.ground_index = 0U;
  node.normal = -Eigen::Vector3f::UnitX();
  node.slope_rad = 1.57079632679F;
  node.surface_id = 30;
  node.staircase_id = 3;
  node.step_index = 0;
  node.terrain_class = TerrainClass::STAIR_RISER;
  node.flags = TERRAIN_NODE_STATIC_MAP | TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED;
  return node;
}

TerrainNode treadAnchorNode()
{
  TerrainNode node;
  node.ground_index = 0U;
  node.normal = Eigen::Vector3f::UnitZ();
  node.slope_rad = 0.0F;
  node.surface_id = 31;
  node.staircase_id = 3;
  node.step_index = 0;
  node.terrain_class = TerrainClass::STAIR_TREAD;
  node.flags = TERRAIN_NODE_STATIC_MAP | TERRAIN_NODE_MANUAL_CORRIDOR |
    TERRAIN_NODE_ONLINE_CONFIRMED;
  return node;
}

StairRiserSemanticsConfig enabledConfig()
{
  StairRiserSemanticsConfig config;
  config.enabled = true;
  config.fail_closed = true;
  config.expected_map_hash = "map-a";
  config.max_snapshot_age_nanoseconds = 200'000'000LL;
  config.minimum_stair_confidence = 0.90F;
  config.max_node_match_distance_m = 0.04F;
  config.riser_plane_tolerance_m = 0.02F;
  config.riser_lateral_tolerance_m = 0.01F;
  config.riser_vertical_tolerance_m = 0.01F;
  return config;
}

struct FixtureData
{
  std::shared_ptr<SharedData> shared{std::make_shared<SharedData>()};
  std::shared_ptr<rclcpp::Clock> clock{
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};
  DynamicGraph graph;

  explicit FixtureData(
    const TerrainNode & terrain_node = riserNode(),
    const Eigen::Vector3f & ground_position = Eigen::Vector3f(0.0F, 0.0F, 0.09F))
  {
    shared->pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI ground;
    ground.x = ground_position.x();
    ground.y = ground_position.y();
    ground.z = ground_position.z();
    shared->pcl_ground_->push_back(ground);
    shared->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
    shared->kdtree_ground_->setInputCloud(shared->pcl_ground_);
    shared->setTerrainSnapshot(std::make_shared<const TerrainSnapshot>(
      "map-a", 6U, clock->now().nanoseconds() - 10'000'000LL,
      std::vector<TerrainNode>{terrain_node},
      std::vector<StaircaseModel>{staircase()}));
    graph.initial(1U, 10.0);
  }
};

pcl::ModelCoefficients::Ptr horizontalPlane()
{
  auto plane = std::make_shared<pcl::ModelCoefficients>();
  plane->values = {0.0F, 0.0F, 1.0F, 0.0F};
  return plane;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAt(float z)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI point;
  point.x = 0.0F;
  point.y = 0.0F;
  point.z = z;
  cloud->push_back(point);
  return cloud;
}

TEST(ClusterMarkingStairSemantics, ExpectedRiserSkipsOnlyGraphProjectionAndKeepsRawPoint)
{
  FixtureData fixture;
  Marking marking(
    "cluster_marking_test", &fixture.graph, 0.20, 0.50, fixture.shared,
    0.10, 0.10, enabledConfig(), 0.03, fixture.clock);
  auto raw_cluster = cloudAt(0.09F);
  std::unordered_map<int, float> distances;
  marking.computeMinDistanceFromObstacle2GroundNodes(
    raw_cluster, horizontalPlane(), distances);
  EXPECT_TRUE(distances.empty());
  ASSERT_EQ(raw_cluster->size(), 1U);
  EXPECT_FLOAT_EQ(raw_cluster->front().z, 0.09F);
}

TEST(ClusterMarkingStairSemantics, GroundWithoutRiserUsesAdjacentTreadAnchor)
{
  FixtureData fixture(
    treadAnchorNode(), Eigen::Vector3f(0.02F, 0.0F, 0.18F));
  auto limits = enabledConfig();
  limits.max_node_match_distance_m = 0.12F;
  Marking marking(
    "cluster_marking_test", &fixture.graph, 0.20, 0.50, fixture.shared,
    0.10, 0.10, limits, 0.03, fixture.clock);
  auto raw_cluster = cloudAt(0.09F);
  std::unordered_map<int, float> distances;
  marking.computeMinDistanceFromObstacle2GroundNodes(
    raw_cluster, horizontalPlane(), distances);
  EXPECT_TRUE(distances.empty());
  ASSERT_EQ(raw_cluster->size(), 1U);
  EXPECT_FLOAT_EQ(raw_cluster->front().z, 0.09F);
}

TEST(ClusterMarkingStairSemantics, DisabledModePreservesLegacyRiserMarking)
{
  FixtureData fixture;
  Marking marking(
    "cluster_marking_test", &fixture.graph, 0.20, 0.50, fixture.shared,
    0.10, 0.10);
  auto raw_cluster = cloudAt(0.09F);
  std::unordered_map<int, float> distances;
  marking.computeMinDistanceFromObstacle2GroundNodes(
    raw_cluster, horizontalPlane(), distances);
  EXPECT_FALSE(distances.empty());
  EXPECT_EQ(raw_cluster->size(), 1U);
}

TEST(ClusterMarkingStairSemantics, GeometryMismatchRemainsAnObstacle)
{
  FixtureData fixture;
  Marking marking(
    "cluster_marking_test", &fixture.graph, 0.20, 0.50, fixture.shared,
    0.10, 0.10, enabledConfig(), 0.03, fixture.clock);
  auto unexpected_cluster = cloudAt(0.40F);
  std::unordered_map<int, float> distances;
  marking.computeMinDistanceFromObstacle2GroundNodes(
    unexpected_cluster, horizontalPlane(), distances);
  EXPECT_FALSE(distances.empty());
  EXPECT_EQ(unexpected_cluster->size(), 1U);
}

}  // namespace
}  // namespace perception_3d
