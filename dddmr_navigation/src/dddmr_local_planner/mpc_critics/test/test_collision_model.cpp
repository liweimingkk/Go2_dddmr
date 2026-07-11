#include <gtest/gtest.h>

#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mpc_critics/collision_model.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace mpc_critics
{
namespace
{

base_trajectory::Trajectory makeTrajectory(float trajectory_origin_z = 0.0F)
{
  base_trajectory::Trajectory trajectory(0.3, 0.0, 0.0, 0.1, 1);
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = trajectory_origin_z;
  pose.pose.orientation.w = 1.0;

  pcl::PointCloud<pcl::PointXYZ> cuboid;
  const auto add_vertex = [&cuboid](float x, float y, float z) {
    pcl::PointXYZ point;
    point.x = x;
    point.y = y;
    point.z = z;
    cuboid.push_back(point);
  };

  // Vertex order matches the trajectory generators: blb, brb, blt, flb,
  // brt, frt, flt, frb.
  add_vertex(-0.35F,  0.30F, -0.20F);
  add_vertex(-0.35F, -0.30F, -0.20F);
  add_vertex(-0.35F,  0.30F,  0.60F);
  add_vertex( 0.42F,  0.30F, -0.20F);
  add_vertex(-0.35F, -0.30F,  0.60F);
  add_vertex( 0.42F, -0.30F,  0.60F);
  add_vertex( 0.42F,  0.30F,  0.60F);
  add_vertex( 0.42F, -0.30F, -0.20F);

  base_trajectory::cuboid_min_max_t bounds;
  bounds.first.x = -0.35F;
  bounds.first.y = -0.30F;
  bounds.first.z = -0.20F;
  bounds.second.x = 0.42F;
  bounds.second.y = 0.30F;
  bounds.second.z = 0.60F;
  trajectory.addPoint(pose, cuboid, bounds);
  return trajectory;
}

class CollisionModelTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if(!rclcpp::ok()){
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if(rclcpp::ok()){
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("collision_model_test");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    shared_data_ = std::make_shared<ModelSharedData>(buffer_);
    model_.setSharedData(shared_data_);
    model_.initialize("collision", node_);
  }

  void setObstacle(float x, float y, float z)
  {
    shared_data_->pcl_perception_->clear();
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    shared_data_->pcl_perception_->push_back(point);
    shared_data_->updateData();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<ModelSharedData> shared_data_;
  CollisionModel model_;
};

TEST_F(CollisionModelTest, SinglePointInsideCuboidIsRejected)
{
  setObstacle(0.20F, 0.0F, 0.10F);
  auto trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(CollisionModelTest, SingleLowPointInsideLegEnvelopeIsRejected)
{
  setObstacle(0.20F, 0.0F, -0.15F);
  auto trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(CollisionModelTest, SinglePointOutsideCuboidIsAccepted)
{
  setObstacle(0.80F, 0.0F, 0.10F);
  auto trajectory = makeTrajectory();
  EXPECT_DOUBLE_EQ(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(CollisionModelTest, EmptyFreshObservationIsAccepted)
{
  shared_data_->pcl_perception_->clear();
  shared_data_->updateData();
  auto trajectory = makeTrajectory();
  EXPECT_DOUBLE_EQ(model_.scoreTrajectory(trajectory), 0.0);
}

class StairSemanticCollisionModelTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if(!rclcpp::ok()){
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if(rclcpp::ok()){
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("collision.stair_semantics.enabled", true),
      rclcpp::Parameter("collision.stair_semantics.fail_closed", true),
      rclcpp::Parameter("collision.stair_semantics.expected_map_hash", "map-a"),
      rclcpp::Parameter("collision.stair_semantics.max_snapshot_age_sec", 0.20),
      rclcpp::Parameter("collision.stair_semantics.minimum_stair_confidence", 0.90),
      rclcpp::Parameter("collision.stair_semantics.max_node_match_distance_m", 0.04),
      rclcpp::Parameter("collision.stair_semantics.riser_plane_tolerance_m", 0.02),
      rclcpp::Parameter("collision.stair_semantics.riser_lateral_tolerance_m", 0.01),
      rclcpp::Parameter("collision.stair_semantics.riser_vertical_tolerance_m", 0.01),
      rclcpp::Parameter("collision.stair_semantics.leg_envelope_min_x_m", -0.35),
      rclcpp::Parameter("collision.stair_semantics.leg_envelope_min_y_m", -0.30),
      rclcpp::Parameter("collision.stair_semantics.leg_envelope_min_z_m", -0.20),
      rclcpp::Parameter("collision.stair_semantics.leg_envelope_max_x_m", 0.42),
      rclcpp::Parameter("collision.stair_semantics.leg_envelope_max_y_m", 0.30),
      rclcpp::Parameter("collision.stair_semantics.leg_envelope_max_z_m", -0.02),
      rclcpp::Parameter("collision.stair_semantics.max_support_xy_distance_m", 0.30),
      rclcpp::Parameter("collision.stair_semantics.min_body_clearance_m", 0.10),
      rclcpp::Parameter("collision.stair_semantics.max_body_clearance_m", 0.50)});
    node_ = std::make_shared<rclcpp::Node>("stair_semantic_collision_model_test", options);
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    shared_data_ = std::make_shared<ModelSharedData>(buffer_);
    model_.setSharedData(shared_data_);
    model_.initialize("collision", node_);
  }

  perception_3d::StaircaseModel staircase() const
  {
    perception_3d::StaircaseModel value;
    value.id = 5;
    value.map_hash = "map-a";
    value.up_axis = Eigen::Vector3f::UnitX();
    value.lower_landing_center = Eigen::Vector3f(-0.10F, 0.0F, -0.24F);
    value.upper_landing_center = Eigen::Vector3f(1.10F, 0.0F, 0.48F);
    value.first_riser_center = Eigen::Vector3f(0.20F, 0.0F, -0.15F);
    value.corridor_polygon_xy = {
      {0.10F, -0.15F}, {0.90F, -0.15F}, {0.90F, 0.15F}, {0.10F, 0.15F}};
    value.lower_landing_polygon_xy = {
      {-0.30F, -0.30F}, {0.20F, -0.30F}, {0.20F, 0.30F}, {-0.30F, 0.30F}};
    value.upper_landing_polygon_xy = {
      {0.80F, -0.30F}, {1.30F, -0.30F}, {1.30F, 0.30F}, {0.80F, 0.30F}};
    value.width_m = 0.30F;
    value.riser_height_m = 0.18F;
    value.tread_depth_m = 0.20F;
    value.step_count = 4;
    value.confidence = 0.98F;
    value.allow_up = true;
    value.allow_down = true;
    return value;
  }

  void setSemanticObstacle(
    float x, float y, float z,
    std::uint32_t flags = perception_3d::TERRAIN_NODE_STATIC_MAP |
      perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
      perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED,
    std::uint64_t ground_version = 9U)
  {
    shared_data_->pcl_perception_->clear();
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    shared_data_->pcl_perception_->push_back(point);
    shared_data_->updateData();

    perception_3d::TerrainNode node;
    node.ground_index = 0U;
    node.normal = -Eigen::Vector3f::UnitX();
    node.slope_rad = 1.57079632679F;
    node.surface_id = 50;
    node.staircase_id = 5;
    node.step_index = 0;
    node.terrain_class = perception_3d::TerrainClass::STAIR_RISER;
    node.flags = flags;
    perception_3d::TerrainNode support_node;
    support_node.ground_index = 1U;
    support_node.normal = Eigen::Vector3f::UnitZ();
    support_node.slope_rad = 0.0F;
    support_node.support_ratio = 0.95F;
    support_node.confidence = 0.95F;
    support_node.surface_id = 1;
    support_node.staircase_id = 5;
    support_node.step_index = -1;
    support_node.terrain_class = perception_3d::TerrainClass::FLAT;
    support_node.flags = perception_3d::TERRAIN_NODE_STATIC_MAP |
      perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
      perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED |
      perception_3d::TERRAIN_NODE_LANDING |
      perception_3d::TERRAIN_NODE_LOWER_LANDING;
    const auto snapshot = std::make_shared<const perception_3d::TerrainSnapshot>(
      "map-a", 9U, node_->now().nanoseconds() - 50'000'000LL,
      std::vector<perception_3d::TerrainNode>{node, support_node},
      std::vector<perception_3d::StaircaseModel>{staircase()});
    ASSERT_TRUE(snapshot->valid());
    auto ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    ground->push_back(point);
    pcl::PointXYZI support_point;
    support_point.x = 0.0F;
    support_point.y = 0.0F;
    support_point.z = -0.24F;
    ground->push_back(support_point);
    shared_data_->updateTerrainData(snapshot, ground, ground_version);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<ModelSharedData> shared_data_;
  CollisionModel model_;
};

TEST_F(StairSemanticCollisionModelTest, ExpectedRiserPassesOnlyInsideLegEnvelope)
{
  setSemanticObstacle(0.20F, 0.0F, -0.15F);
  auto trajectory = makeTrajectory();
  EXPECT_DOUBLE_EQ(model_.scoreTrajectory(trajectory), 0.0);

  trajectory = makeTrajectory(-0.20F);
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(StairSemanticCollisionModelTest, SideWallAndHandrailRemainLethal)
{
  setSemanticObstacle(0.20F, 0.25F, -0.15F);
  auto trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);

  setSemanticObstacle(0.20F, 0.0F, 0.40F);
  trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(StairSemanticCollisionModelTest, MissingConfirmationOrVersionMismatchIsLethal)
{
  const std::uint32_t no_online = perception_3d::TERRAIN_NODE_STATIC_MAP |
    perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR;
  setSemanticObstacle(0.20F, 0.0F, -0.15F, no_online);
  auto trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);

  setSemanticObstacle(
    0.20F, 0.0F, -0.15F,
    perception_3d::TERRAIN_NODE_STATIC_MAP |
    perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR |
    perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED,
    8U);
  trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(StairSemanticCollisionModelTest, UnexpectedPointInSameCuboidStillRejectsTrajectory)
{
  setSemanticObstacle(0.20F, 0.0F, -0.15F);
  pcl::PointXYZI unexpected;
  unexpected.x = 0.25F;
  unexpected.y = 0.25F;
  unexpected.z = 0.10F;
  shared_data_->pcl_perception_->push_back(unexpected);
  shared_data_->updateData();
  auto trajectory = makeTrajectory();
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

}  // namespace
}  // namespace mpc_critics
