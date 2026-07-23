#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "dddmr_scan_planner/command_guard_policy.hpp"

namespace dddmr_scan_planner
{
namespace
{

CommandGuardInput validInput()
{
  CommandGuardInput input;
  input.route_ready = true;
  input.raw_command_received = true;
  input.odom_received = true;
  input.cloud_received = true;
  input.trajectory_received = true;
  input.planner_heartbeat_received = true;
  input.raw_command_age = 0.01;
  input.odom_age = 0.01;
  input.cloud_age = 0.01;
  input.planner_heartbeat_age = 0.01;
  input.odom_header_age = 0.05;
  input.cloud_header_age = 0.10;
  input.raw_command = {0.30, 0.10, 0.20};
  return input;
}

TEST(CommandGuardPolicy, PassesFreshCommand)
{
  const CommandGuardPolicy policy(CommandGuardLimits{});
  const auto result = policy.evaluate(validInput());
  EXPECT_TRUE(result.allowed);
  EXPECT_TRUE(result.moving);
  EXPECT_EQ(result.reason, "ok");
  EXPECT_DOUBLE_EQ(result.command.x, 0.30);
  EXPECT_DOUBLE_EQ(result.command.y, 0.0);
  EXPECT_DOUBLE_EQ(result.command.yaw, 0.20);
}

TEST(CommandGuardPolicy, StopsUntilRouteIsReady)
{
  const CommandGuardPolicy policy(CommandGuardLimits{});
  auto input = validInput();
  input.route_ready = false;
  const auto result = policy.evaluate(input);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.reason, "route_not_ready");
  EXPECT_DOUBLE_EQ(result.command.x, 0.0);
}

TEST(CommandGuardPolicy, StopsForEveryStaleSafetyInput)
{
  const CommandGuardPolicy policy(CommandGuardLimits{});

  auto raw_stale = validInput();
  raw_stale.raw_command_age = 0.16;
  EXPECT_EQ(policy.evaluate(raw_stale).reason, "raw_command_stale");

  auto odom_stale = validInput();
  odom_stale.odom_age = 0.26;
  EXPECT_EQ(policy.evaluate(odom_stale).reason, "odom_stale");

  auto cloud_stale = validInput();
  cloud_stale.cloud_age = 0.36;
  EXPECT_EQ(policy.evaluate(cloud_stale).reason, "cloud_stale");

  auto planner_stale = validInput();
  planner_stale.planner_heartbeat_age = 0.51;
  EXPECT_EQ(policy.evaluate(planner_stale).reason, "planner_stale");

  auto trajectory_missing = validInput();
  trajectory_missing.trajectory_received = false;
  EXPECT_EQ(policy.evaluate(trajectory_missing).reason, "trajectory_missing");
}

TEST(CommandGuardPolicy, StopsForStaleOrFutureHeaders)
{
  const CommandGuardPolicy policy(CommandGuardLimits{});

  auto stale = validInput();
  stale.cloud_header_age = 0.36;
  EXPECT_EQ(policy.evaluate(stale).reason, "cloud_header_stale");

  auto future = validInput();
  future.odom_header_age = -0.051;
  EXPECT_EQ(policy.evaluate(future).reason, "odom_header_stale");
}

TEST(CommandGuardPolicy, RejectsNonfiniteCommand)
{
  const CommandGuardPolicy policy(CommandGuardLimits{});
  auto input = validInput();
  input.raw_command.x = std::numeric_limits<double>::quiet_NaN();
  const auto result = policy.evaluate(input);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.reason, "nonfinite_command");
}

TEST(CommandGuardPolicy, AppliesAxisAndTranslationCaps)
{
  CommandGuardLimits limits;
  limits.max_x = 0.50;
  limits.max_y = 0.20;
  limits.max_yaw = 0.50;
  limits.max_translation = 0.40;
  const CommandGuardPolicy policy(limits);
  auto input = validInput();
  input.raw_command = {1.0, -1.0, 2.0};
  const auto result = policy.evaluate(input);
  EXPECT_TRUE(result.allowed);
  EXPECT_NEAR(std::hypot(result.command.x, result.command.y), 0.40, 1e-12);
  EXPECT_GT(result.command.x, 0.0);
  EXPECT_LT(result.command.y, 0.0);
  EXPECT_DOUBLE_EQ(result.command.yaw, 0.50);
}

TEST(CommandGuardPolicy, RejectsInvalidConfiguration)
{
  CommandGuardLimits limits;
  limits.planner_timeout = 0.0;
  EXPECT_THROW(CommandGuardPolicy policy(limits), std::invalid_argument);
}

}  // namespace
}  // namespace dddmr_scan_planner
