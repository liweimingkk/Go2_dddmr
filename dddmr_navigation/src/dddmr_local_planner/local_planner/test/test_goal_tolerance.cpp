#include "local_planner/goal_tolerance.h"
#include "local_planner/plan_validation.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{

perception_3d::TerrainNode makeTerrainNode(
  perception_3d::TerrainClass terrain_class,
  std::int32_t surface_id = -1)
{
  perception_3d::TerrainNode node;
  node.terrain_class = terrain_class;
  node.surface_id = surface_id;
  return node;
}

void markLanding(
  perception_3d::TerrainNode * node,
  std::int32_t staircase_id,
  perception_3d::TerrainNodeFlag side)
{
  ASSERT_NE(node, nullptr);
  node->staircase_id = staircase_id;
  node->flags = static_cast<std::uint32_t>(perception_3d::TERRAIN_NODE_LANDING) |
    static_cast<std::uint32_t>(side);
}

geometry_msgs::msg::PoseStamped planPose(
  double x, double y, double z, const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

TEST(GoalTolerance, SeparatesHorizontalAndVerticalError)
{
  EXPECT_TRUE(local_planner::withinGoalTolerance(0.2, 0.1, 0.1, 0.3, 0.2));
  EXPECT_FALSE(local_planner::withinGoalTolerance(0.2, 0.1, 0.3, 0.3, 0.2));
  EXPECT_FALSE(local_planner::withinGoalTolerance(0.4, 0.0, 0.1, 0.3, 0.2));
}

TEST(GoalTolerance, IncludesExactBoundary)
{
  EXPECT_TRUE(local_planner::withinGoalTolerance(0.3, 0.0, -0.2, 0.3, 0.2));
}

TEST(GoalTolerance, RejectsNonfiniteAndInvalidLimits)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(local_planner::withinGoalTolerance(nan, 0.0, 0.0, 0.3, 0.2));
  EXPECT_FALSE(local_planner::withinGoalTolerance(0.0, 0.0, 0.0, -0.3, 0.2));
  EXPECT_FALSE(local_planner::withinGoalTolerance(0.0, 0.0, 0.0, 0.3, -0.2));
}

TEST(GoalTolerance, TerrainGoalRequiresSafeSupportAndMatchingFlatSurface)
{
  auto current = makeTerrainNode(perception_3d::TerrainClass::FLAT, 7);
  auto goal = makeTerrainNode(perception_3d::TerrainClass::RAMP, 7);
  EXPECT_TRUE(local_planner::terrainGoalMatches(&current, &goal));

  goal.surface_id = 8;
  EXPECT_FALSE(local_planner::terrainGoalMatches(&current, &goal));
  goal.terrain_class = perception_3d::TerrainClass::DROP;
  EXPECT_FALSE(local_planner::terrainGoalMatches(&current, &goal));
  EXPECT_FALSE(local_planner::terrainGoalMatches(nullptr, &goal));
  EXPECT_FALSE(local_planner::terrainGoalMatches(&current, nullptr));
}

TEST(GoalTolerance, TerrainGoalDistinguishesLowerAndUpperLanding)
{
  auto lower = makeTerrainNode(perception_3d::TerrainClass::FLAT, 10);
  auto same_lower = makeTerrainNode(perception_3d::TerrainClass::FLAT, 11);
  auto upper = makeTerrainNode(perception_3d::TerrainClass::FLAT, 10);
  markLanding(&lower, 4, perception_3d::TERRAIN_NODE_LOWER_LANDING);
  markLanding(&same_lower, 4, perception_3d::TERRAIN_NODE_LOWER_LANDING);
  markLanding(&upper, 4, perception_3d::TERRAIN_NODE_UPPER_LANDING);

  EXPECT_TRUE(local_planner::terrainGoalMatches(&same_lower, &lower));
  EXPECT_FALSE(local_planner::terrainGoalMatches(&upper, &lower));
  same_lower.staircase_id = 5;
  EXPECT_FALSE(local_planner::terrainGoalMatches(&same_lower, &lower));

  lower.flags |= static_cast<std::uint32_t>(perception_3d::TERRAIN_NODE_UPPER_LANDING);
  EXPECT_FALSE(local_planner::terrainGoalMatches(&same_lower, &lower));
}

TEST(GoalTolerance, TerrainGoalRequiresExactStairTread)
{
  auto current = makeTerrainNode(perception_3d::TerrainClass::STAIR_TREAD);
  auto goal = makeTerrainNode(perception_3d::TerrainClass::STAIR_TREAD);
  current.staircase_id = goal.staircase_id = 2;
  current.step_index = goal.step_index = 3;
  EXPECT_TRUE(local_planner::terrainGoalMatches(&current, &goal));

  current.step_index = 2;
  EXPECT_FALSE(local_planner::terrainGoalMatches(&current, &goal));
  current.step_index = 3;
  current.staircase_id = 6;
  EXPECT_FALSE(local_planner::terrainGoalMatches(&current, &goal));
}

TEST(GlobalPlanValidation, AcceptsTwoPoseStairTransition)
{
  const std::vector<geometry_msgs::msg::PoseStamped> plan{
    planPose(0.0, 0.0, 0.0), planPose(0.30, 0.0, 0.18)};
  std::string error;
  EXPECT_TRUE(local_planner::validGlobalPlan(plan, "map", 0.75, &error)) << error;
}

TEST(GlobalPlanValidation, RejectsShortStaleOrDiscontinuousReplacement)
{
  EXPECT_FALSE(local_planner::validGlobalPlan(
      {planPose(0.0, 0.0, 0.0)}, "map", 0.75));
  EXPECT_FALSE(local_planner::validGlobalPlan(
      {planPose(0.0, 0.0, 0.0), planPose(0.1, 0.0, 0.0, "odom")},
      "map", 0.75));
  EXPECT_FALSE(local_planner::validGlobalPlan(
      {planPose(0.0, 0.0, 0.0), planPose(1.0, 0.0, 0.0)}, "map", 0.75));
  EXPECT_FALSE(local_planner::validGlobalPlan(
      {planPose(0.0, 0.0, 0.0), planPose(0.0, 0.0, 0.0)}, "map", 0.75));

  auto invalid = planPose(0.1, 0.0, 0.0);
  invalid.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(local_planner::validGlobalPlan(
      {planPose(0.0, 0.0, 0.0), invalid}, "map", 0.75));
}
