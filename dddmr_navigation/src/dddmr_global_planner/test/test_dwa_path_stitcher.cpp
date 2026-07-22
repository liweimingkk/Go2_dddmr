#include <gtest/gtest.h>

#include <string>

#include "global_planner/dwa_path_stitcher.h"

namespace global_planner
{
namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = z;
  result.pose.orientation.w = 1.0;
  return result;
}

TEST(DwaPathStitcher, RejectsMissingConnectorInsteadOfReturningRemoteTail)
{
  nav_msgs::msg::Path connector;
  nav_msgs::msg::Path reference;
  reference.poses = {pose(0.0, 0.0, 0.0), pose(3.0, 0.0, 0.0)};
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(stitchDwaPath(
      connector, reference, 1U, 0.15, 0.35, output, &reason));
  EXPECT_EQ(reason, "empty_connector");
  EXPECT_TRUE(output.poses.empty());
}

TEST(DwaPathStitcher, RejectsDiscontinuousConnector)
{
  nav_msgs::msg::Path connector;
  connector.poses = {pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)};
  nav_msgs::msg::Path reference;
  reference.poses = {pose(0.0, 0.0, 0.0), pose(1.3, 0.0, 0.0)};
  nav_msgs::msg::Path output;
  std::string reason;

  EXPECT_FALSE(stitchDwaPath(
      connector, reference, 1U, 0.15, 0.35, output, &reason));
  EXPECT_EQ(reason, "discontinuous_join");
  EXPECT_TRUE(output.poses.empty());
}

TEST(DwaPathStitcher, AppendsReferenceTailOnce)
{
  nav_msgs::msg::Path connector;
  connector.poses = {pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)};
  nav_msgs::msg::Path reference;
  reference.poses = {
    pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0),
    pose(2.0, 0.0, 0.0), pose(3.0, 0.0, 0.0)};
  nav_msgs::msg::Path output;

  ASSERT_TRUE(stitchDwaPath(
      connector, reference, 1U, 0.15, 0.35, output));
  ASSERT_EQ(output.poses.size(), 4U);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(output.poses[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.poses[2].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.position.x, 3.0);
}

TEST(DwaPathStitcher, AllowsBoundedProjectedJoin)
{
  nav_msgs::msg::Path connector;
  connector.poses = {pose(0.0, 0.0, 0.0), pose(0.91, 0.02, 0.10)};
  nav_msgs::msg::Path reference;
  reference.poses = {pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.20)};
  nav_msgs::msg::Path output;

  ASSERT_TRUE(stitchDwaPath(
      connector, reference, 1U, 0.15, 0.35, output));
  ASSERT_EQ(output.poses.size(), 3U);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.position.x, 1.0);
}

}  // namespace
}  // namespace global_planner
