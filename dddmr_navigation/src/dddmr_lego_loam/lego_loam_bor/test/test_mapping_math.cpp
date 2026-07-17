#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "axis_split_factor.h"
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

gtsam::SharedNoiseModel makeAxisSplitTestNoise()
{
  gtsam::Vector6 variances;
  variances << 0.0025, 0.0025, 0.0025, 0.01, 0.01, 0.01;
  return gtsam::noiseModel::Diagonal::Variances(variances);
}

TEST(AxisSplitFactor, SelectsOdometryRotationAndWorldXYButScanZ)
{
  const gtsam::Pose3 previous_external(
      gtsam::Rot3::RzRyRx(0.2, -0.35, 1.4),
      gtsam::Point3(10.0, 20.0, 100.0));
  const gtsam::Pose3 current_external(
      gtsam::Rot3::RzRyRx(-0.1, 0.25, 1.7),
      gtsam::Point3(12.0, 23.0, -50.0));
  const gtsam::Pose3 previous_scan(
      gtsam::Rot3::RzRyRx(-0.8, 0.7, -1.0),
      gtsam::Point3(-40.0, 60.0, 5.0));
  const gtsam::Pose3 current_scan(
      gtsam::Rot3::RzRyRx(0.9, -0.6, 0.3),
      gtsam::Point3(400.0, -600.0, 8.0));

  const auto measurement = lego_loam_bor::makeAxisSplitMeasurement(
      previous_external, current_external, previous_scan, current_scan);

  EXPECT_TRUE(measurement.relative_rotation.equals(
      previous_external.rotation().between(current_external.rotation()), 1e-12));
  EXPECT_TRUE(measurement.world_translation_delta.isApprox(
      (gtsam::Vector3() << 2.0, 3.0, 3.0).finished(), 1e-12));
}

TEST(AxisSplitFactor, UsesMapAlignedOdometryForWorldXY)
{
  const gtsam::Pose3 map_from_odom(
      gtsam::Rot3::Rz(M_PI_2), gtsam::Point3(10.0, -4.0, 0.0));
  const gtsam::Pose3 previous_raw_odom(
      gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
      gtsam::Point3(1.0, 2.0, 80.0));
  const gtsam::Pose3 current_raw_odom(
      gtsam::Rot3::RzRyRx(-0.1, 0.25, 0.5),
      gtsam::Point3(3.0, 2.0, -60.0));
  const gtsam::Pose3 previous_map_odom =
      map_from_odom.compose(previous_raw_odom);
  const gtsam::Pose3 current_map_odom =
      map_from_odom.compose(current_raw_odom);
  const gtsam::Pose3 previous_scan(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.4));
  const gtsam::Pose3 current_scan(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 2.4));

  const auto measurement = lego_loam_bor::makeAxisSplitMeasurement(
      previous_map_odom, current_map_odom, previous_scan, current_scan);
  const gtsam::Vector3 expected_world_delta =
      current_map_odom.translation() - previous_map_odom.translation();

  EXPECT_NEAR(measurement.world_translation_delta.x(), 0.0, 1e-12);
  EXPECT_NEAR(measurement.world_translation_delta.y(), 2.0, 1e-12);
  EXPECT_NEAR(measurement.world_translation_delta.z(), 2.0, 1e-12);
  EXPECT_NEAR(measurement.world_translation_delta.x(), expected_world_delta.x(), 1e-12);
  EXPECT_NEAR(measurement.world_translation_delta.y(), expected_world_delta.y(), 1e-12);
}

TEST(AxisSplitFactor, SolvesObservableConditionalScanZ)
{
  const auto update = lego_loam_bor::solveAxisSplitScalarUpdate(
      40.0, 2.0, 10.0, 0.15);

  EXPECT_TRUE(update.observable);
  EXPECT_FALSE(update.clamped);
  EXPECT_DOUBLE_EQ(update.information, 40.0);
  EXPECT_NEAR(update.delta, 0.05, 1e-12);
}

TEST(AxisSplitFactor, RejectsWeakScanZAndClampsLargeIterationSteps)
{
  const auto weak = lego_loam_bor::solveAxisSplitScalarUpdate(
      9.0, 2.0, 10.0, 0.15);
  const auto large = lego_loam_bor::solveAxisSplitScalarUpdate(
      40.0, 20.0, 10.0, 0.15);

  EXPECT_FALSE(weak.observable);
  EXPECT_TRUE(large.observable);
  EXPECT_TRUE(large.clamped);
  EXPECT_DOUBLE_EQ(large.delta, 0.15);
}

TEST(AxisSplitFactor, HasWorldAxisResidualWithTiltedPoses)
{
  const gtsam::Pose3 previous_external(
      gtsam::Rot3::RzRyRx(0.15, 0.35, M_PI_2),
      gtsam::Point3(0.0, 0.0, 90.0));
  const gtsam::Pose3 current_external(
      gtsam::Rot3::RzRyRx(-0.12, 0.28, M_PI_2 + 0.2),
      gtsam::Point3(1.5, -0.7, -30.0));
  const gtsam::Pose3 previous_scan(gtsam::Rot3(), gtsam::Point3(9.0, 8.0, 2.0));
  const gtsam::Pose3 current_scan(gtsam::Rot3(), gtsam::Point3(-9.0, -8.0, 5.0));
  const auto measurement = lego_loam_bor::makeAxisSplitMeasurement(
      previous_external, current_external, previous_scan, current_scan);
  const lego_loam_bor::AxisSplitFactor factor(
      0, 1, measurement, makeAxisSplitTestNoise());

  const gtsam::Pose3 previous_estimate(
      previous_external.rotation(), gtsam::Point3(3.0, 4.0, 2.0));
  const gtsam::Pose3 current_estimate(
      current_external.rotation(), gtsam::Point3(4.5, 3.3, 5.0));

  EXPECT_LT(factor.evaluateError(previous_estimate, current_estimate).norm(), 1e-10);
}

TEST(AxisSplitFactor, AnalyticJacobiansMatchNumericalJacobians)
{
  const gtsam::Pose3 previous_external(
      gtsam::Rot3::RzRyRx(0.2, -0.3, 1.1),
      gtsam::Point3(0.0, 0.0, 50.0));
  const gtsam::Pose3 current_external(
      gtsam::Rot3::RzRyRx(-0.1, 0.25, 1.4),
      gtsam::Point3(1.2, -0.8, -70.0));
  const gtsam::Pose3 previous_scan(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 1.0));
  const gtsam::Pose3 current_scan(gtsam::Rot3(), gtsam::Point3(-4.0, -5.0, 4.0));
  const lego_loam_bor::AxisSplitFactor factor(
      0, 1,
      lego_loam_bor::makeAxisSplitMeasurement(
          previous_external, current_external, previous_scan, current_scan),
      makeAxisSplitTestNoise());
  const gtsam::Pose3 previous_pose(
      gtsam::Rot3::RzRyRx(0.18, -0.27, 1.05),
      gtsam::Point3(2.0, -1.0, 0.7));
  const gtsam::Pose3 current_pose(
      gtsam::Rot3::RzRyRx(-0.07, 0.22, 1.38),
      gtsam::Point3(3.1, -1.9, 3.9));
  gtsam::Matrix analytic_previous;
  gtsam::Matrix analytic_current;
  factor.evaluateError(
      previous_pose, current_pose, analytic_previous, analytic_current);
  const std::function<gtsam::Vector6(
      const gtsam::Pose3 &, const gtsam::Pose3 &)> error =
      [&factor](const gtsam::Pose3 & first, const gtsam::Pose3 & second) {
        return factor.evaluateError(first, second);
      };

  const gtsam::Matrix numerical_previous =
      gtsam::numericalDerivative21<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3>(
      error, previous_pose, current_pose, 1e-6);
  const gtsam::Matrix numerical_current =
      gtsam::numericalDerivative22<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3>(
      error, previous_pose, current_pose, 1e-6);

  EXPECT_TRUE(analytic_previous.isApprox(numerical_previous, 1e-5));
  EXPECT_TRUE(analytic_current.isApprox(numerical_current, 1e-5));
}

TEST(AxisSplitFactor, OptimizesMeasuredScanRiseWithoutExternalZ)
{
  const gtsam::Pose3 previous_external(
      gtsam::Rot3::RzRyRx(0.05, 0.3, M_PI_2),
      gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 current_external(
      gtsam::Rot3::RzRyRx(-0.03, 0.2, M_PI_2 + 0.15),
      gtsam::Point3(2.0, -1.0, 0.0));
  const gtsam::Pose3 previous_scan(gtsam::Rot3(), gtsam::Point3(50.0, 60.0, 0.0));
  const gtsam::Pose3 current_scan(gtsam::Rot3(), gtsam::Point3(-50.0, -60.0, 2.0));
  const gtsam::Pose3 previous_pose(previous_external.rotation(), gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 expected_current(current_external.rotation(), gtsam::Point3(2.0, -1.0, 2.0));

  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      0, previous_pose, gtsam::noiseModel::Isotropic::Variance(6, 1e-9)));
  graph.add(lego_loam_bor::AxisSplitFactor(
      0, 1,
      lego_loam_bor::makeAxisSplitMeasurement(
          previous_external, current_external, previous_scan, current_scan),
      makeAxisSplitTestNoise()));
  gtsam::Values initial;
  initial.insert(0, previous_pose);
  initial.insert(
      1, gtsam::Pose3(gtsam::Rot3::RzRyRx(0.4, -0.2, 0.3),
                      gtsam::Point3(-2.0, 4.0, 0.0)));

  const gtsam::Values result = gtsam::GaussNewtonOptimizer(graph, initial).optimize();
  const gtsam::Pose3 optimized = result.at<gtsam::Pose3>(1);

  EXPECT_TRUE(optimized.rotation().equals(expected_current.rotation(), 1e-6));
  EXPECT_TRUE(optimized.translation().isApprox(expected_current.translation(), 1e-6));
}

TEST(AxisSplitFactor, RejectsNonFiniteMeasurements)
{
  lego_loam_bor::AxisSplitMeasurement measurement;
  measurement.world_translation_delta.z() =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      lego_loam_bor::AxisSplitFactor(0, 1, measurement, makeAxisSplitTestNoise()),
      std::invalid_argument);
}

}  // namespace
