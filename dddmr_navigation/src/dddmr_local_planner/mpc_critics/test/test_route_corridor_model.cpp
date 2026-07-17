#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mpc_critics/route_corridor_model.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace mpc_critics
{
namespace
{

base_trajectory::Trajectory makeTrajectory(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  base_trajectory::Trajectory trajectory(0.3, 0.0, 0.0, 0.1, points.size());
  const pcl::PointCloud<pcl::PointXYZ> cuboid;
  const base_trajectory::cuboid_min_max_t bounds;
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    trajectory.addPoint(pose, cuboid, bounds);
  }
  return trajectory;
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point output;
  output.x = x;
  output.y = y;
  output.z = z;
  return output;
}

class RouteCorridorModelTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("route_corridor_model_test");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    shared_data_ = std::make_shared<ModelSharedData>(buffer_);
    model_.setSharedData(shared_data_);
    model_.initialize("route_corridor", node_);
  }

  void setRoute(const std::vector<geometry_msgs::msg::Point> & points)
  {
    shared_data_->prune_plan_.poses.clear();
    for (const auto & value : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = value;
      pose.pose.orientation.w = 1.0;
      shared_data_->prune_plan_.poses.push_back(pose);
    }
    shared_data_->updateData();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<ModelSharedData> shared_data_;
  RouteCorridorModel model_;
};

TEST_F(RouteCorridorModelTest, AcceptsTrajectoryInsideHorizontalAndVerticalBounds)
{
  setRoute({point(0.0, 0.0, 0.0), point(1.0, 0.0, 0.2)});
  auto trajectory = makeTrajectory({point(0.2, 0.4, 0.1), point(0.9, 0.5, 0.3)});
  EXPECT_DOUBLE_EQ(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(RouteCorridorModelTest, RejectsHorizontalDeparture)
{
  setRoute({point(0.0, 0.0, 0.0), point(1.0, 0.0, 0.0)});
  auto trajectory = makeTrajectory({point(0.5, 0.61, 0.0)});
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(RouteCorridorModelTest, RejectsVerticalDeparture)
{
  setRoute({point(0.0, 0.0, 0.0), point(1.0, 0.0, 0.0)});
  auto trajectory = makeTrajectory({point(0.5, 0.0, 0.36)});
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

TEST_F(RouteCorridorModelTest, RejectsWhenReferenceRouteIsMissing)
{
  shared_data_->updateData();
  auto trajectory = makeTrajectory({point(0.0, 0.0, 0.0)});
  EXPECT_LT(model_.scoreTrajectory(trajectory), 0.0);
}

}  // namespace
}  // namespace mpc_critics
