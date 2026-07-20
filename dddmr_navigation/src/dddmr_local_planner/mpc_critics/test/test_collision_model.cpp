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

base_trajectory::Trajectory makeTrajectory()
{
  base_trajectory::Trajectory trajectory(0.3, 0.0, 0.0, 0.1, 1);
  geometry_msgs::msg::PoseStamped pose;
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

}  // namespace
}  // namespace mpc_critics
