/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include <trajectory_generators/terrain_following_dd_theory.h>

#include <trajectory_generators/dd_simple_trajectory_generator_theory.h>

#include <pluginlib/class_list_macros.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <tf2/utils.h>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(
  trajectory_generators::TerrainFollowingDDTheory,
  trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

void TerrainFollowingDDTheory::onInitialize()
{
  const auto getBool = [this](const std::string & key, bool default_value) {
      node_->declare_parameter(name_ + key, rclcpp::ParameterValue(default_value));
      bool value = default_value;
      node_->get_parameter(name_ + key, value);
      return value;
    };
  const auto getDouble = [this](const std::string & key, double default_value) {
      node_->declare_parameter(name_ + key, rclcpp::ParameterValue(default_value));
      double value = default_value;
      node_->get_parameter(name_ + key, value);
      return value;
    };

  enabled_ = getBool(".terrain_enabled", false);
  fail_closed_ = getBool(".terrain_fail_closed", true);
  nominal_body_clearance_m_ = getDouble(".terrain_body_clearance_m", 0.24);
  limits_.max_support_xy_distance_m = getDouble(
    ".terrain_max_support_xy_distance_m", limits_.max_support_xy_distance_m);
  limits_.max_support_vertical_distance_m = getDouble(
    ".terrain_max_support_vertical_distance_m", limits_.max_support_vertical_distance_m);
  limits_.min_support_ratio = getDouble(
    ".terrain_min_support_ratio", limits_.min_support_ratio);
  limits_.min_confidence = getDouble(
    ".terrain_min_confidence", limits_.min_confidence);
  limits_.min_upward_normal_z = getDouble(
    ".terrain_min_upward_normal_z", limits_.min_upward_normal_z);
  limits_.max_normal_change_rad = getDouble(
    ".terrain_max_normal_change_rad", limits_.max_normal_change_rad);
  limits_.stair_transition_waypoint_tolerance_m = getDouble(
    ".terrain_stair_transition_waypoint_tolerance_m",
    limits_.stair_transition_waypoint_tolerance_m);
  limits_.max_stair_transition_span_m = getDouble(
    ".terrain_max_stair_transition_span_m", limits_.max_stair_transition_span_m);
  limits_.max_stair_heading_error_rad = getDouble(
    ".terrain_max_stair_heading_error_rad", limits_.max_stair_heading_error_rad);
  limits_.stairs_enabled = getBool(".terrain_stairs_enabled", false);
  limits_.require_manual_corridor = getBool(
    ".terrain_require_manual_corridor", true);
  limits_.require_online_confirmation = getBool(
    ".terrain_require_online_confirmation", true);

  if ((enabled_ && !fail_closed_) ||
    !std::isfinite(nominal_body_clearance_m_) || nominal_body_clearance_m_ < 0.0 ||
    nominal_body_clearance_m_ > 2.0 ||
    !terrainTrajectoryProjectionLimitsValid(limits_))
  {
    throw std::invalid_argument(
            name_ + " terrain-following configuration is invalid or fail-open");
  }

  projector_ = std::make_unique<TerrainTrajectoryProjector>(limits_);
  planar_generator_ = std::make_shared<DDSimpleTrajectoryGeneratorTheory>();
  planar_generator_->setSharedData(shared_data_);
  // Reuse the exact DDSimple x/y/yaw and cuboid parameter surface.  Only the
  // additional terrain_* parameters above are new.
  planar_generator_->initialize(name_, node_);

  if (enabled_) {
    shared_data_->requestTerrainProjectionData();
    RCLCPP_WARN(
      node_->get_logger().get_child(name_),
      "TerrainFollowingDDTheory ENABLED; every candidate is leased to one "
      "TerrainSnapshot/static-ground generation and rejected as a whole on any mismatch");
  } else {
    RCLCPP_WARN(
      node_->get_logger().get_child(name_),
      "TerrainFollowingDDTheory is disabled and will generate no trajectories");
  }
}

void TerrainFollowingDDTheory::initialise()
{
  cycle_context_.reset();
  cycle_invalidated_ = false;
  cycle_rejection_ = enabled_ ?
    TerrainTrajectoryRejection::MISSING_SNAPSHOT :
    TerrainTrajectoryRejection::TERRAIN_DISABLED;
  if (!enabled_ || !fail_closed_ || !projector_ || !planar_generator_) {
    return;
  }

  cycle_context_ = shared_data_->terrainProjectionData();
  if (!cycle_context_) {
    return;
  }
  if (!cycle_context_->valid()) {
    cycle_rejection_ = TerrainTrajectoryRejection::SNAPSHOT_GROUND_MISMATCH;
    cycle_context_.reset();
    return;
  }
  cycle_rejection_ = TerrainTrajectoryRejection::NONE;
  planar_generator_->initialise();
}

bool TerrainFollowingDDTheory::contextStillCurrent() const noexcept
{
  return terrainProjectionContextIdentityMatches(
    cycle_context_, shared_data_ ? shared_data_->terrainProjectionData() : nullptr);
}

bool TerrainFollowingDDTheory::hasMoreTrajectories()
{
  if (!enabled_ || cycle_invalidated_ || cycle_rejection_ != TerrainTrajectoryRejection::NONE ||
    !planar_generator_ || !contextStillCurrent())
  {
    if (enabled_ && cycle_rejection_ == TerrainTrajectoryRejection::NONE &&
      !contextStillCurrent())
    {
      cycle_invalidated_ = true;
      cycle_rejection_ = TerrainTrajectoryRejection::SNAPSHOT_CHANGED;
    }
    return false;
  }
  return planar_generator_->hasMoreTrajectories();
}

void TerrainFollowingDDTheory::recordRejection(
  TerrainTrajectoryRejection rejection,
  std::size_t pose_index,
  base_trajectory::Trajectory * trajectory)
{
  if (trajectory != nullptr) {
    trajectory->resetPoints();
    trajectory->rejected_by_ = name_ + ":" + toString(rejection);
  }
  RCLCPP_WARN_THROTTLE(
    node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
    "terrain_trajectory_reject reason=%s pose_index=%zu snapshot_version=%lu "
    "static_ground_generation=%lu",
    toString(rejection), pose_index,
    cycle_context_ && cycle_context_->snapshot() ?
    static_cast<unsigned long>(cycle_context_->snapshot()->version()) : 0UL,
    cycle_context_ ?
    static_cast<unsigned long>(cycle_context_->staticGroundGeneration()) : 0UL);
}

bool TerrainFollowingDDTheory::nextTrajectory(base_trajectory::Trajectory & trajectory)
{
  if (!hasMoreTrajectories()) {
    recordRejection(cycle_rejection_, 0U, &trajectory);
    return false;
  }

  base_trajectory::Trajectory planar_trajectory;
  if (!planar_generator_->nextTrajectory(planar_trajectory)) {
    return false;
  }
  if (!contextStillCurrent()) {
    cycle_invalidated_ = true;
    cycle_rejection_ = TerrainTrajectoryRejection::SNAPSHOT_CHANGED;
    recordRejection(cycle_rejection_, 0U, &trajectory);
    return false;
  }

  TerrainBodyPose reference;
  reference.position = Eigen::Vector3d(
    shared_data_->robot_pose_.transform.translation.x,
    shared_data_->robot_pose_.transform.translation.y,
    shared_data_->robot_pose_.transform.translation.z);
  reference.yaw_rad = tf2::getYaw(shared_data_->robot_pose_.transform.rotation);

  std::vector<TerrainBodyPose> future_poses;
  future_poses.reserve(planar_trajectory.getPointsSize());
  for (unsigned int index = 0U; index < planar_trajectory.getPointsSize(); ++index) {
    const auto pose = planar_trajectory.getPoint(index);
    TerrainBodyPose future;
    future.position = Eigen::Vector3d(
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    future.yaw_rad = tf2::getYaw(pose.pose.orientation);
    future_poses.push_back(future);
  }

  const TerrainTrajectoryProjectionResult projection = projector_->project(
    cycle_context_, reference, nominal_body_clearance_m_, future_poses);
  if (!projection.ok()) {
    recordRejection(projection.rejection, projection.rejected_pose_index, &trajectory);
    return false;
  }
  if (!contextStillCurrent()) {
    cycle_invalidated_ = true;
    cycle_rejection_ = TerrainTrajectoryRejection::SNAPSHOT_CHANGED;
    recordRejection(cycle_rejection_, projection.poses.size(), &trajectory);
    return false;
  }
  if (!rebuildProjectedTrajectory(planar_trajectory, projection, &trajectory)) {
    recordRejection(TerrainTrajectoryRejection::INVALID_GEOMETRY, 0U, &trajectory);
    return false;
  }
  if (!contextStillCurrent()) {
    cycle_invalidated_ = true;
    cycle_rejection_ = TerrainTrajectoryRejection::SNAPSHOT_CHANGED;
    recordRejection(cycle_rejection_, projection.poses.size(), &trajectory);
    return false;
  }
  return true;
}

bool TerrainFollowingDDTheory::rebuildProjectedTrajectory(
  const base_trajectory::Trajectory & source,
  const TerrainTrajectoryProjectionResult & projection,
  base_trajectory::Trajectory * output) const
{
  if (output == nullptr || projection.poses.size() != source.getPointsSize() ||
    projection.poses.empty())
  {
    return false;
  }

  base_trajectory::Trajectory projected(
    source.xv_, source.yv_, source.thetav_, source.time_delta_, source.getPointsSize());
  projected.cost_ = source.cost_;
  projected.rejected_by_ = source.rejected_by_;

  for (unsigned int index = 0U; index < source.getPointsSize(); ++index) {
    const auto old_pose = source.getPoint(index);
    const auto & terrain_pose = projection.poses[index];
    Eigen::Quaterniond old_orientation(
      old_pose.pose.orientation.w,
      old_pose.pose.orientation.x,
      old_pose.pose.orientation.y,
      old_pose.pose.orientation.z);
    if (!old_orientation.coeffs().allFinite() || old_orientation.norm() <= 1e-9 ||
      !terrain_pose.position.allFinite() ||
      !terrain_pose.orientation.coeffs().allFinite())
    {
      return false;
    }
    old_orientation.normalize();

    Eigen::Affine3d old_transform = Eigen::Affine3d::Identity();
    old_transform.translation() = Eigen::Vector3d(
      old_pose.pose.position.x, old_pose.pose.position.y, old_pose.pose.position.z);
    old_transform.linear() = old_orientation.toRotationMatrix();
    Eigen::Affine3d new_transform = Eigen::Affine3d::Identity();
    new_transform.translation() = terrain_pose.position;
    new_transform.linear() = terrain_pose.orientation.normalized().toRotationMatrix();
    const Eigen::Affine3d delta = new_transform * old_transform.inverse();

    pcl::PointCloud<pcl::PointXYZ> projected_cuboid;
    const pcl::PointCloud<pcl::PointXYZ> source_cuboid = source.getCuboid(index);
    if (source_cuboid.empty()) {
      return false;
    }
    pcl::transformPointCloud(
      source_cuboid, projected_cuboid, delta.matrix().cast<float>());
    base_trajectory::cuboid_min_max_t min_max;
    pcl::getMinMax3D(projected_cuboid, min_max.first, min_max.second);

    geometry_msgs::msg::PoseStamped pose = old_pose;
    pose.pose.position.x = terrain_pose.position.x();
    pose.pose.position.y = terrain_pose.position.y();
    pose.pose.position.z = terrain_pose.position.z();
    pose.pose.orientation.x = terrain_pose.orientation.x();
    pose.pose.orientation.y = terrain_pose.orientation.y();
    pose.pose.orientation.z = terrain_pose.orientation.z();
    pose.pose.orientation.w = terrain_pose.orientation.w();
    if (!projected.addPoint(pose, projected_cuboid, min_max)) {
      return false;
    }
  }
  *output = std::move(projected);
  return true;
}

}  // namespace trajectory_generators
