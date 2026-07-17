#include <cmath>
#include <cstdint>
#include <deque>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "degeneracy_utils.h"
#include "odom_sync_utils.h"

namespace
{

TEST(DegeneracyProjection, ProjectsAscendingWeakEigenvectors)
{
  Eigen::Matrix<float, 6, 6> normal = Eigen::Matrix<float, 6, 6>::Zero();
  normal.diagonal() << 1.0F, 2.0F, 3.0F, 200.0F, 300.0F, 400.0F;
  Eigen::Matrix<float, 6, 1> thresholds;
  thresholds.setConstant(100.0F);

  const auto result = lego_loam_bor::computeDegeneracyProjection(normal, thresholds);

  EXPECT_TRUE(result.is_degenerate);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(result.degenerate_eigendirections[static_cast<std::size_t>(i)]);
    EXPECT_NEAR(result.projection(i, i), 0.0F, 1e-5F);
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_FALSE(result.degenerate_eigendirections[static_cast<std::size_t>(i)]);
    EXPECT_NEAR(result.projection(i, i), 1.0F, 1e-5F);
  }
  EXPECT_TRUE(result.projection.isApprox(result.projection.transpose(), 1e-5F));
  EXPECT_TRUE((result.projection * result.projection).isApprox(result.projection, 1e-5F));
}

TEST(DegeneracyProjection, UsesEigenvectorColumnsForRotatedSystem)
{
  constexpr float angle = 0.61F;
  Eigen::Matrix<float, 6, 6> eigenvectors = Eigen::Matrix<float, 6, 6>::Identity();
  eigenvectors(0, 0) = std::cos(angle);
  eigenvectors(3, 0) = std::sin(angle);
  eigenvectors(0, 3) = -std::sin(angle);
  eigenvectors(3, 3) = std::cos(angle);
  Eigen::Matrix<float, 6, 1> eigenvalues;
  eigenvalues << 1.0F, 200.0F, 250.0F, 300.0F, 350.0F, 400.0F;
  const Eigen::Matrix<float, 6, 6> normal =
      eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
  Eigen::Matrix<float, 6, 1> thresholds;
  thresholds.setConstant(100.0F);

  const auto result = lego_loam_bor::computeDegeneracyProjection(normal, thresholds);

  const Eigen::Matrix<float, 6, 1> weak_direction = result.eigenvectors.col(0);
  const Eigen::Matrix<float, 6, 1> strong_direction = result.eigenvectors.col(5);
  EXPECT_LT((result.projection * weak_direction).norm(), 1e-4F);
  EXPECT_TRUE((result.projection * strong_direction).isApprox(strong_direction, 1e-4F));
}

TEST(DegeneracyProjection, SupportsThreeDofFeatureAssociationSystems)
{
  Eigen::Matrix3f normal = Eigen::Matrix3f::Zero();
  normal.diagonal() << 1.0F, 20.0F, 30.0F;
  Eigen::Vector3f thresholds;
  thresholds.setConstant(10.0F);

  const auto result = lego_loam_bor::computeDegeneracyProjection(normal, thresholds);

  EXPECT_TRUE(result.is_degenerate);
  EXPECT_TRUE(result.degenerate_eigendirections[0]);
  EXPECT_FALSE(result.degenerate_eigendirections[1]);
  EXPECT_FALSE(result.degenerate_eigendirections[2]);
  EXPECT_NEAR(result.projection(0, 0), 0.0F, 1e-5F);
}

TEST(OdomInterpolation, InterpolatesPoseTwistAndCovarianceAtCloudStamp)
{
  nav_msgs::msg::Odometry before;
  nav_msgs::msg::Odometry after;
  before.header.frame_id = after.header.frame_id = "odom";
  before.child_frame_id = after.child_frame_id = "base_link";
  before.header.stamp.sec = 10;
  after.header.stamp.sec = 12;
  before.pose.pose.position.x = 1.0;
  after.pose.pose.position.x = 5.0;
  before.twist.twist.linear.x = 0.2;
  after.twist.twist.linear.x = 0.6;
  before.pose.covariance[0] = 2.0;
  after.pose.covariance[0] = 6.0;
  tf2::Quaternion q_before;
  tf2::Quaternion q_after;
  q_before.setRPY(0.0, 0.0, 0.0);
  q_after.setRPY(0.0, 0.0, M_PI_2);
  before.pose.pose.orientation = tf2::toMsg(q_before);
  after.pose.pose.orientation = tf2::toMsg(q_after);
  builtin_interfaces::msg::Time target;
  target.sec = 11;

  nav_msgs::msg::Odometry output;
  ASSERT_TRUE(lego_loam_bor::interpolateOdometry(before, after, target, output));
  EXPECT_EQ(output.header.stamp.sec, 11);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 4.0);

  tf2::Quaternion interpolated;
  tf2::fromMsg(output.pose.pose.orientation, interpolated);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(interpolated).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(yaw, M_PI_4, 1e-6);
}

TEST(OdomInterpolation, RejectsMismatchedFrames)
{
  nav_msgs::msg::Odometry before;
  nav_msgs::msg::Odometry after;
  before.header.frame_id = "odom_a";
  after.header.frame_id = "odom_b";
  before.child_frame_id = after.child_frame_id = "base_link";
  before.header.stamp.sec = 1;
  after.header.stamp.sec = 2;
  before.pose.pose.orientation.w = 1.0;
  after.pose.pose.orientation.w = 1.0;
  builtin_interfaces::msg::Time target;
  target.sec = 1;
  target.nanosec = 500000000;
  nav_msgs::msg::Odometry output;
  EXPECT_FALSE(lego_loam_bor::interpolateOdometry(before, after, target, output));
}

nav_msgs::msg::Odometry makeOdom(const double stamp_sec, const double x)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = lego_loam_bor::nanosecondsToStamp(
      static_cast<int64_t>(std::llround(stamp_sec * 1e9)));
  odom.pose.pose.position.x = x;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

TEST(OdomSelection, InterpolatesBracketAfterApplyingClockOffset)
{
  std::deque<nav_msgs::msg::Odometry> buffer{
      makeOdom(8.00, 0.0), makeOdom(8.02, 2.0)};
  const auto target = lego_loam_bor::nanosecondsToStamp(10010000000LL);

  const auto result = lego_loam_bor::selectTimeAlignedOdometry(
      buffer, target, 2.0, 0.02, 0.05, false);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.interpolated);
  EXPECT_NEAR(result.odometry.pose.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(result.bracket_span_sec, 0.02, 1e-9);
  EXPECT_NEAR(result.sync_error_sec, 0.01, 1e-9);
}

TEST(OdomSelection, RejectsOutsideTolerance)
{
  std::deque<nav_msgs::msg::Odometry> buffer{
      makeOdom(10.00, 0.0), makeOdom(10.02, 2.0)};
  const auto target = lego_loam_bor::nanosecondsToStamp(11000000000LL);

  const auto result = lego_loam_bor::selectTimeAlignedOdometry(
      buffer, target, 0.0, 0.05, 0.05, true);

  EXPECT_FALSE(result.valid);
}

TEST(OdomSelection, UsesNearestOnlyWhenExplicitlyAllowed)
{
  std::deque<nav_msgs::msg::Odometry> buffer{makeOdom(10.00, 4.0)};
  const auto target = lego_loam_bor::nanosecondsToStamp(10010000000LL);

  const auto before_timeout = lego_loam_bor::selectTimeAlignedOdometry(
      buffer, target, 0.0, 0.02, 0.05, false);
  const auto after_timeout = lego_loam_bor::selectTimeAlignedOdometry(
      buffer, target, 0.0, 0.02, 0.05, true);

  EXPECT_FALSE(before_timeout.valid);
  ASSERT_TRUE(after_timeout.valid);
  EXPECT_FALSE(after_timeout.interpolated);
  EXPECT_DOUBLE_EQ(after_timeout.odometry.pose.pose.position.x, 4.0);
}

}  // namespace
