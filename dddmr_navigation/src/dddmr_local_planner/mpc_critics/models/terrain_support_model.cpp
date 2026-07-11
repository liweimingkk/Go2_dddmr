/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <mpc_critics/terrain_support_model.h>

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mpc_critics::TerrainSupportModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

void TerrainSupportModel::onInitialize()
{
  node_->declare_parameter(name_ + ".enabled", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".enabled", enabled_);

  node_->declare_parameter(name_ + ".fail_closed", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".fail_closed", fail_closed_);

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);

  node_->declare_parameter(name_ + ".support_z_offset_m", rclcpp::ParameterValue(0.24));
  node_->get_parameter(name_ + ".support_z_offset_m", support_z_offset_m_);

  node_->declare_parameter(name_ + ".max_support_distance_m", rclcpp::ParameterValue(0.30));
  node_->get_parameter(name_ + ".max_support_distance_m", limits_.max_support_distance_m);

  node_->declare_parameter(name_ + ".max_z_gap_m", rclcpp::ParameterValue(0.12));
  node_->get_parameter(name_ + ".max_z_gap_m", limits_.max_z_gap_m);

  node_->declare_parameter(name_ + ".min_support_ratio", rclcpp::ParameterValue(0.80));
  node_->get_parameter(name_ + ".min_support_ratio", limits_.min_support_ratio);

  node_->declare_parameter(name_ + ".min_confidence", rclcpp::ParameterValue(0.90));
  node_->get_parameter(name_ + ".min_confidence", limits_.min_confidence);

  node_->declare_parameter(name_ + ".max_step_index_delta", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + ".max_step_index_delta", limits_.max_step_index_delta);

  node_->declare_parameter(name_ + ".stairs_enabled", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".stairs_enabled", limits_.stairs_enabled);

  node_->declare_parameter(name_ + ".require_manual_corridor", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".require_manual_corridor", limits_.require_manual_corridor);

  node_->declare_parameter(name_ + ".require_online_confirmation", rclcpp::ParameterValue(true));
  node_->get_parameter(
    name_ + ".require_online_confirmation", limits_.require_online_confirmation);

  if ((enabled_ && !fail_closed_) || !terrainSupportLimitsValid(limits_) ||
    !std::isfinite(weight_) || weight_ < 0.0 || !std::isfinite(support_z_offset_m_) ||
    support_z_offset_m_ < 0.0)
  {
    throw std::invalid_argument(
            "TerrainSupportModel requires fail_closed=true and finite non-negative limits");
  }
  if (enabled_ && shared_data_) {
    shared_data_->requestTerrainSupportData();
  }

  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "enabled=%d fail_closed=%d support_radius=%.3f z_offset=%.3f z_gap=%.3f "
    "min_support=%.2f min_confidence=%.2f stairs_enabled=%d",
    enabled_, fail_closed_, limits_.max_support_distance_m, support_z_offset_m_,
    limits_.max_z_gap_m, limits_.min_support_ratio, limits_.min_confidence,
    limits_.stairs_enabled);
}

double TerrainSupportModel::reject(
  perception_3d::TerrainRejectionReason reason,
  unsigned int step,
  unsigned int step_count) const
{
  RCLCPP_WARN_THROTTLE(
    node_->get_logger().get_child(name_), *node_->get_clock(), 1000,
    "terrain_support_reject reason=%s step=%u/%u",
    perception_3d::toString(reason), step, step_count);
  return -1.0;
}

double TerrainSupportModel::scoreTrajectory(base_trajectory::Trajectory & traj)
{
  using perception_3d::TerrainRejectionReason;

  if (!enabled_) {
    return 0.0;
  }
  if (!fail_closed_ || !shared_data_) {
    return reject(TerrainRejectionReason::FAIL_OPEN_CONFIGURATION, 0U, traj.getPointsSize());
  }

  const auto snapshot = shared_data_->terrain_snapshot_;
  const auto ground = shared_data_->terrain_ground_;
  const auto kdtree = shared_data_->terrain_ground_kdtree_;
  const std::uint64_t ground_version = shared_data_->terrain_ground_version_;
  if (validated_snapshot_ != snapshot || validated_ground_ != ground ||
    validated_kdtree_ != kdtree.get() || validated_ground_version_ != ground_version)
  {
    validation_result_ = validateTerrainSupportData(
      snapshot, ground_version, ground ? ground->points.size() : 0U,
      kdtree != nullptr && ground != nullptr && !ground->points.empty());
    validated_snapshot_ = snapshot;
    validated_ground_ = ground;
    validated_kdtree_ = kdtree.get();
    validated_ground_version_ = ground_version;
  }
  if (validation_result_ != TerrainRejectionReason::NONE) {
    return reject(validation_result_, 0U, traj.getPointsSize());
  }
  if (traj.getPointsSize() == 0U) {
    return reject(TerrainRejectionReason::INVALID_GEOMETRY, 0U, 0U);
  }

  const std::uint64_t scoring_version = snapshot->version();
  TerrainSupportObservation previous;
  bool has_previous = false;
  double risk_cost = 0.0;

  for (unsigned int step = 0U; step < traj.getPointsSize(); ++step) {
    if (shared_data_->terrain_snapshot_ != snapshot ||
      shared_data_->terrain_ground_version_ != scoring_version)
    {
      return reject(
        TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH, step + 1U,
        traj.getPointsSize());
    }

    const pcl::PointXYZI trajectory_point = traj.getPCLPoint(step);
    if (!std::isfinite(trajectory_point.x) || !std::isfinite(trajectory_point.y) ||
      !std::isfinite(trajectory_point.z))
    {
      return reject(TerrainRejectionReason::INVALID_GEOMETRY, step + 1U, traj.getPointsSize());
    }

    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    // The trajectory pose is base_link while terrain_ground contains support
    // surfaces. Query at the expected contact elevation; querying at body z
    // can select the next higher tread near a riser and manufacture a gap.
    pcl::PointXYZI support_query = trajectory_point;
    support_query.z -= static_cast<float>(support_z_offset_m_);
    if (kdtree->nearestKSearch(support_query, 1, indices, squared_distances) != 1 ||
      indices.front() < 0 ||
      static_cast<std::size_t>(indices.front()) >= ground->points.size())
    {
      return reject(TerrainRejectionReason::NO_SUPPORT, step + 1U, traj.getPointsSize());
    }

    const std::size_t ground_index = static_cast<std::size_t>(indices.front());
    const auto & support_point = ground->points[ground_index];
    const double dx = static_cast<double>(trajectory_point.x - support_point.x);
    const double dy = static_cast<double>(trajectory_point.y - support_point.y);
    const double support_clearance =
      static_cast<double>(trajectory_point.z - support_point.z);

    TerrainSupportObservation current;
    current.ground_index = ground_index;
    current.horizontal_distance_m = std::hypot(dx, dy);
    current.z_gap_m = std::fabs(support_clearance - support_z_offset_m_);
    current.node = snapshot->nodeAt(ground_index);
    if (current.node != nullptr && current.node->staircase_id >= 0) {
      current.staircase = snapshot->staircaseById(current.node->staircase_id);
    }

    const auto reason = evaluateTerrainSupport(
      current, has_previous ? &previous : nullptr, limits_);
    if (reason != TerrainRejectionReason::NONE) {
      return reject(reason, step + 1U, traj.getPointsSize());
    }

    risk_cost += weight_ *
      ((1.0 - static_cast<double>(current.node->support_ratio)) +
      (1.0 - static_cast<double>(current.node->confidence)));
    previous = current;
    has_previous = true;
  }

  if (shared_data_->terrain_snapshot_ != snapshot ||
    shared_data_->terrain_ground_version_ != scoring_version)
  {
    return reject(
      TerrainRejectionReason::SNAPSHOT_VERSION_MISMATCH, traj.getPointsSize(),
      traj.getPointsSize());
  }
  return risk_cost / static_cast<double>(traj.getPointsSize());
}

}  // namespace mpc_critics
