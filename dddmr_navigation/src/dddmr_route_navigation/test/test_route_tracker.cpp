#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "dddmr_route_navigation/route_tracker.hpp"

namespace dddmr_route_navigation
{
namespace
{

geometry_msgs::msg::Point point(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point output;
  output.x = x;
  output.y = y;
  output.z = z;
  return output;
}

nav_msgs::msg::Path makePath(const std::vector<geometry_msgs::msg::Point> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & value : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position = value;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(RouteTrackerTest, RejectsInvalidAndGappedRoutes)
{
  RouteTracker tracker;
  std::string error;
  EXPECT_FALSE(tracker.setRoute(makePath({point(0.0, 0.0), point(1.0, 0.0)}), &error));
  EXPECT_NE(error.find("at least three"), std::string::npos);

  EXPECT_FALSE(
    tracker.setRoute(
      makePath({point(0.0, 0.0), point(1.0, 0.0), point(4.0, 0.0)}), &error));
  EXPECT_NE(error.find("exceeding"), std::string::npos);
}

TEST(RouteTrackerTest, RemovesStationaryDuplicatesAndResamplesInThreeDimensions)
{
  RouteTracker tracker;
  std::string error;
  ASSERT_TRUE(
    tracker.setRoute(
      makePath({
        point(0.0, 0.0, 0.0), point(0.005, 0.0, 0.0),
        point(0.5, 0.0, 0.5), point(1.0, 0.0, 1.0)}),
      &error)) << error;
  EXPECT_NEAR(tracker.length(), std::sqrt(2.0), 1.0e-6);
  ASSERT_GT(tracker.route().poses.size(), 10U);
  EXPECT_NEAR(tracker.route().poses.back().pose.position.z, 1.0, 1.0e-9);
}

TEST(RouteTrackerTest, RequiresStartEnvelopeByDefault)
{
  RouteTracker tracker;
  std::string error;
  ASSERT_TRUE(
    tracker.setRoute(
      makePath({point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)}), &error));
  EXPECT_FALSE(tracker.initialize(point(1.0, 0.0), false, &error));
  EXPECT_NE(error.find("start envelope"), std::string::npos);
  EXPECT_TRUE(tracker.initialize(point(0.2, 0.0), false, &error)) << error;
}

TEST(RouteTrackerTest, ProgressIsMonotonicAndDoesNotJumpAtDistantCrossing)
{
  RouteTrackerConfig config;
  config.progress_search_forward = 1.0;
  RouteTracker tracker(config);
  std::string error;
  ASSERT_TRUE(
    tracker.setRoute(
      makePath({
        point(0.0, 0.0), point(2.0, 0.0), point(0.0, 0.0),
        point(0.0, 2.0)}),
      &error)) << error;
  ASSERT_TRUE(tracker.initialize(point(0.0, 0.0), false, &error)) << error;

  ASSERT_TRUE(tracker.update(point(0.8, 0.0)).valid);
  const double forward_progress = tracker.progress();
  EXPECT_NEAR(forward_progress, 0.8, 0.11);

  ASSERT_TRUE(tracker.update(point(0.3, 0.0)).valid);
  EXPECT_GE(tracker.progress(), forward_progress);

  ASSERT_TRUE(tracker.update(point(0.0, 0.0)).valid);
  EXPECT_LT(tracker.progress(), 2.0);
}

TEST(RouteTrackerTest, ChecksHorizontalAndVerticalCorridorSeparately)
{
  RouteTracker tracker;
  std::string error;
  ASSERT_TRUE(
    tracker.setRoute(
      makePath({point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)}), &error));
  ASSERT_TRUE(tracker.initialize(point(0.0, 0.0), false, &error));

  EXPECT_TRUE(tracker.insideCorridor(tracker.update(point(0.5, 0.50, 0.30))));
  EXPECT_FALSE(tracker.insideCorridor(tracker.update(point(0.6, 0.61, 0.0))));
  EXPECT_FALSE(tracker.insideCorridor(tracker.update(point(0.7, 0.0, 0.36))));
}

TEST(RouteTrackerTest, LocalPlanEndsAtTrueRouteGoalNearCompletion)
{
  RouteTracker tracker;
  std::string error;
  ASSERT_TRUE(
    tracker.setRoute(
      makePath({point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)}), &error));
  ASSERT_TRUE(tracker.initialize(point(0.0, 0.0), false, &error));
  ASSERT_TRUE(tracker.update(point(1.8, 0.0)).valid);
  const auto local_plan = tracker.localPlan();
  ASSERT_GE(local_plan.poses.size(), 3U);
  EXPECT_NEAR(local_plan.poses.back().pose.position.x, 2.0, 1.0e-9);
  EXPECT_TRUE(tracker.goalReached(point(1.8, 0.0)));
  EXPECT_FALSE(tracker.goalReached(point(1.8, 0.0, 0.30)));
}

}  // namespace
}  // namespace dddmr_route_navigation
