/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

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
#include <mpc_critics/collision_model.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

PLUGINLIB_EXPORT_CLASS(mpc_critics::CollisionModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

CollisionModel::CollisionModel(){
  return;
  
}

void CollisionModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  const std::string prefix = name_ + ".stair_semantics.";
  node_->declare_parameter(prefix + "enabled", rclcpp::ParameterValue(false));
  node_->get_parameter(prefix + "enabled", stair_collision_config_.riser_semantics.enabled);
  node_->declare_parameter(prefix + "fail_closed", rclcpp::ParameterValue(true));
  node_->get_parameter(
    prefix + "fail_closed", stair_collision_config_.riser_semantics.fail_closed);
  node_->declare_parameter(prefix + "expected_map_hash", rclcpp::ParameterValue(""));
  node_->get_parameter(
    prefix + "expected_map_hash", stair_collision_config_.riser_semantics.expected_map_hash);

  double max_snapshot_age_sec = 0.0;
  node_->declare_parameter(prefix + "max_snapshot_age_sec", rclcpp::ParameterValue(0.0));
  node_->get_parameter(prefix + "max_snapshot_age_sec", max_snapshot_age_sec);
  if(std::isfinite(max_snapshot_age_sec) && max_snapshot_age_sec > 0.0 &&
    max_snapshot_age_sec <=
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1.0e9)
  {
    stair_collision_config_.riser_semantics.max_snapshot_age_nanoseconds =
      static_cast<std::int64_t>(max_snapshot_age_sec * 1.0e9);
  }

  double minimum_stair_confidence = 0.0;
  double max_node_match_distance_m = 0.0;
  double riser_plane_tolerance_m = 0.0;
  double riser_lateral_tolerance_m = 0.0;
  double riser_vertical_tolerance_m = 0.0;
  node_->declare_parameter(prefix + "minimum_stair_confidence", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "max_node_match_distance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "riser_plane_tolerance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "riser_lateral_tolerance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "riser_vertical_tolerance_m", rclcpp::ParameterValue(0.0));
  node_->get_parameter(prefix + "minimum_stair_confidence", minimum_stair_confidence);
  node_->get_parameter(prefix + "max_node_match_distance_m", max_node_match_distance_m);
  node_->get_parameter(prefix + "riser_plane_tolerance_m", riser_plane_tolerance_m);
  node_->get_parameter(prefix + "riser_lateral_tolerance_m", riser_lateral_tolerance_m);
  node_->get_parameter(prefix + "riser_vertical_tolerance_m", riser_vertical_tolerance_m);
  stair_collision_config_.riser_semantics.minimum_stair_confidence =
    static_cast<float>(minimum_stair_confidence);
  stair_collision_config_.riser_semantics.max_node_match_distance_m =
    static_cast<float>(max_node_match_distance_m);
  stair_collision_config_.riser_semantics.riser_plane_tolerance_m =
    static_cast<float>(riser_plane_tolerance_m);
  stair_collision_config_.riser_semantics.riser_lateral_tolerance_m =
    static_cast<float>(riser_lateral_tolerance_m);
  stair_collision_config_.riser_semantics.riser_vertical_tolerance_m =
    static_cast<float>(riser_vertical_tolerance_m);

  double leg_min_x = 0.0;
  double leg_min_y = 0.0;
  double leg_min_z = 0.0;
  double leg_max_x = 0.0;
  double leg_max_y = 0.0;
  double leg_max_z = 0.0;
  node_->declare_parameter(prefix + "leg_envelope_min_x_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "leg_envelope_min_y_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "leg_envelope_min_z_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "leg_envelope_max_x_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "leg_envelope_max_y_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "leg_envelope_max_z_m", rclcpp::ParameterValue(0.0));
  double max_support_xy_distance_m = 0.0;
  double min_body_clearance_m = 0.0;
  double max_body_clearance_m = 0.0;
  node_->declare_parameter(prefix + "max_support_xy_distance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "min_body_clearance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(prefix + "max_body_clearance_m", rclcpp::ParameterValue(0.0));
  node_->get_parameter(prefix + "leg_envelope_min_x_m", leg_min_x);
  node_->get_parameter(prefix + "leg_envelope_min_y_m", leg_min_y);
  node_->get_parameter(prefix + "leg_envelope_min_z_m", leg_min_z);
  node_->get_parameter(prefix + "leg_envelope_max_x_m", leg_max_x);
  node_->get_parameter(prefix + "leg_envelope_max_y_m", leg_max_y);
  node_->get_parameter(prefix + "leg_envelope_max_z_m", leg_max_z);
  node_->get_parameter(prefix + "max_support_xy_distance_m", max_support_xy_distance_m);
  node_->get_parameter(prefix + "min_body_clearance_m", min_body_clearance_m);
  node_->get_parameter(prefix + "max_body_clearance_m", max_body_clearance_m);
  stair_collision_config_.leg_envelope_min = Eigen::Vector3f(
    static_cast<float>(leg_min_x), static_cast<float>(leg_min_y),
    static_cast<float>(leg_min_z));
  stair_collision_config_.leg_envelope_max = Eigen::Vector3f(
    static_cast<float>(leg_max_x), static_cast<float>(leg_max_y),
    static_cast<float>(leg_max_z));
  stair_collision_config_.max_support_xy_distance_m =
    static_cast<float>(max_support_xy_distance_m);
  stair_collision_config_.min_body_clearance_m =
    static_cast<float>(min_body_clearance_m);
  stair_collision_config_.max_body_clearance_m =
    static_cast<float>(max_body_clearance_m);

  if(stair_collision_config_.riser_semantics.enabled){
    shared_data_->requestTerrainSupportData();
    std::string validation_error;
    if(!StairCollisionPolicy::validConfig(stair_collision_config_, &validation_error)){
      RCLCPP_ERROR(
        node_->get_logger().get_child(name_),
        "Stair collision semantics are enabled but invalid; all obstacle overlaps "
        "remain lethal: %s", validation_error.c_str());
    }
  }
  RCLCPP_INFO(
    node_->get_logger().get_child(name_), "stair collision semantics: %s",
    stair_collision_config_.riser_semantics.enabled ? "enabled (fail closed)" :
    "disabled (legacy collision behavior)");

}

double CollisionModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(!shared_data_ || !shared_data_->pcl_perception_){
    RCLCPP_ERROR(node_->get_logger().get_child(name_),
      "Collision critic has no perception data object; rejecting trajectory.");
    return -1.0;
  }

  // An empty, fresh obstacle cloud represents clear space. A sparse non-empty
  // cloud must still be checked: a pole or the edge of a person can be only one
  // or two XT16 voxels after segmentation and downsampling.
  if(shared_data_->pcl_perception_->points.empty()){
    return 0.0;
  }

  if(!shared_data_->pcl_perception_kdtree_){
    RCLCPP_ERROR(node_->get_logger().get_child(name_),
      "Collision critic has obstacle points but no kd-tree; rejecting trajectory.");
    return -1.0;
  }

  const bool stair_semantics_enabled = stair_collision_config_.riser_semantics.enabled;
  const auto scoring_snapshot = stair_semantics_enabled ?
    shared_data_->terrain_snapshot_ : perception_3d::TerrainSnapshotConstPtr{};
  const std::uint64_t scoring_ground_version = stair_semantics_enabled ?
    shared_data_->terrain_ground_version_ : 0U;
  const auto scoring_ground = stair_semantics_enabled ?
    shared_data_->terrain_ground_ : pcl::PointCloud<pcl::PointXYZI>::Ptr{};
  const auto scoring_ground_kdtree = stair_semantics_enabled ?
    shared_data_->terrain_ground_kdtree_ : pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr{};
  bool used_stair_passthrough = false;

  for(unsigned int i=0;i<traj.getPointsSize();i++){
    const pcl::PointXYZI pcl_traj_pose = traj.getPCLPoint(i);
    const pcl::PointCloud<pcl::PointXYZ> cuboid = traj.getCuboid(i);
    if(cuboid.points.size() < 4){
      RCLCPP_ERROR(node_->get_logger().get_child(name_),
        "Trajectory step %u has only %zu cuboid vertices; rejecting trajectory.",
        i, cuboid.points.size());
      return -1.0;
    }

    pcl::PointXYZ cuboid_center;
    cuboid_center.x = 0.0F;
    cuboid_center.y = 0.0F;
    cuboid_center.z = 0.0F;
    for(const auto& point : cuboid.points){
      cuboid_center.x += point.x;
      cuboid_center.y += point.y;
      cuboid_center.z += point.z;
    }
    cuboid_center.x /= cuboid.points.size();
    cuboid_center.y /= cuboid.points.size();
    cuboid_center.z /= cuboid.points.size();

    // Vertex order is defined by the trajectory generators: 0=blb, 1=brb,
    // 2=blt, 3=flb. These three edges form the oriented cuboid axes.
    pcl::PointXYZ dx;
    dx.x = cuboid.points[3].x - cuboid.points[0].x;
    dx.y = cuboid.points[3].y - cuboid.points[0].y;
    dx.z = cuboid.points[3].z - cuboid.points[0].z;

    pcl::PointXYZ dy;
    dy.x = cuboid.points[1].x - cuboid.points[0].x;
    dy.y = cuboid.points[1].y - cuboid.points[0].y;
    dy.z = cuboid.points[1].z - cuboid.points[0].z;

    pcl::PointXYZ dz;
    dz.x = cuboid.points[2].x - cuboid.points[0].x;
    dz.y = cuboid.points[2].y - cuboid.points[0].y;
    dz.z = cuboid.points[2].z - cuboid.points[0].z;

    const double half_x = std::sqrt(dx.x*dx.x + dx.y*dx.y + dx.z*dx.z)/2.0;
    const double half_y = std::sqrt(dy.x*dy.x + dy.y*dy.y + dy.z*dy.z)/2.0;
    const double half_z = std::sqrt(dz.x*dz.x + dz.y*dz.y + dz.z*dz.z)/2.0;
    constexpr double kMinimumHalfExtent = 1e-6;
    if(!std::isfinite(half_x) || !std::isfinite(half_y) || !std::isfinite(half_z) ||
       half_x <= kMinimumHalfExtent || half_y <= kMinimumHalfExtent ||
       half_z <= kMinimumHalfExtent){
      RCLCPP_ERROR(node_->get_logger().get_child(name_),
        "Trajectory step %u has invalid cuboid half-extents (%.6f, %.6f, %.6f); "
        "rejecting trajectory.", i, half_x, half_y, half_z);
      return -1.0;
    }

    dx.x/=(2.0*half_x); dx.y/=(2.0*half_x); dx.z/=(2.0*half_x);
    dy.x/=(2.0*half_y); dy.y/=(2.0*half_y); dy.z/=(2.0*half_y);
    dz.x/=(2.0*half_z); dz.y/=(2.0*half_z); dz.z/=(2.0*half_z);

    Eigen::Matrix3f body_axes_world;
    body_axes_world.col(0) = Eigen::Vector3f(dx.x, dx.y, dx.z);
    body_axes_world.col(1) = Eigen::Vector3f(dy.x, dy.y, dy.z);
    body_axes_world.col(2) = Eigen::Vector3f(dz.x, dz.y, dz.z);

    std::int32_t active_support_staircase_id = -1;
    std::int32_t active_support_location = -2;
    if(stair_semantics_enabled && scoring_snapshot && scoring_ground &&
      scoring_ground_kdtree && !scoring_ground->empty())
    {
      const float support_search_radius = std::hypot(
        stair_collision_config_.max_support_xy_distance_m,
        stair_collision_config_.max_body_clearance_m);
      std::vector<int> support_indices;
      std::vector<float> support_squared_distances;
      if(std::isfinite(support_search_radius) && support_search_radius > 0.0F){
        scoring_ground_kdtree->radiusSearch(
          pcl_traj_pose, support_search_radius, support_indices,
          support_squared_distances);
      }
      float best_support_xy_squared = std::numeric_limits<float>::infinity();
      float best_support_clearance_error = std::numeric_limits<float>::infinity();
      for(const int support_index_value : support_indices){
        if(support_index_value < 0 ||
          static_cast<std::size_t>(support_index_value) >= scoring_ground->size())
        {
          continue;
        }
        const std::size_t support_index =
          static_cast<std::size_t>(support_index_value);
        const auto* support_node = scoring_snapshot->nodeAt(support_index);
        if(!support_node || support_node->staircase_id < 0 ||
          (support_node->flags & perception_3d::TERRAIN_NODE_STATIC_MAP) == 0U ||
          (support_node->flags & perception_3d::TERRAIN_NODE_MANUAL_CORRIDOR) == 0U ||
          (support_node->flags & perception_3d::TERRAIN_NODE_ONLINE_CONFIRMED) == 0U)
        {
          continue;
        }
        const auto* staircase = scoring_snapshot->staircaseById(
          support_node->staircase_id);
        if(!staircase){
          continue;
        }
        std::int32_t support_location = -2;
        if(support_node->terrain_class == perception_3d::TerrainClass::STAIR_TREAD &&
          support_node->step_index >= 0 &&
          support_node->step_index < staircase->step_count)
        {
          support_location = support_node->step_index;
        }else if((support_node->flags & perception_3d::TERRAIN_NODE_LANDING) != 0U){
          const bool lower =
            (support_node->flags & perception_3d::TERRAIN_NODE_LOWER_LANDING) != 0U;
          const bool upper =
            (support_node->flags & perception_3d::TERRAIN_NODE_UPPER_LANDING) != 0U;
          if(lower != upper){
            support_location = lower ? -1 : staircase->step_count;
          }
        }
        if(support_location < -1){
          continue;
        }
        const auto& support_point = scoring_ground->points[support_index];
        const float support_dx = support_point.x - pcl_traj_pose.x;
        const float support_dy = support_point.y - pcl_traj_pose.y;
        const float support_xy_squared = support_dx * support_dx + support_dy * support_dy;
        const float body_clearance = pcl_traj_pose.z - support_point.z;
        if(support_xy_squared >
          stair_collision_config_.max_support_xy_distance_m *
          stair_collision_config_.max_support_xy_distance_m ||
          body_clearance < stair_collision_config_.min_body_clearance_m ||
          body_clearance > stair_collision_config_.max_body_clearance_m)
        {
          continue;
        }
        const float midpoint_clearance = 0.5F *
          (stair_collision_config_.min_body_clearance_m +
          stair_collision_config_.max_body_clearance_m);
        const float clearance_error = std::abs(body_clearance - midpoint_clearance);
        if(support_xy_squared < best_support_xy_squared ||
          (support_xy_squared == best_support_xy_squared &&
          clearance_error < best_support_clearance_error))
        {
          best_support_xy_squared = support_xy_squared;
          best_support_clearance_error = clearance_error;
          active_support_staircase_id = support_node->staircase_id;
          active_support_location = support_location;
        }
      }
    }

    // Search the entire swept cuboid instead of assuming every footprint fits
    // inside the old fixed 1 m query radius.
    double search_radius = 0.0;
    for(const auto& vertex : cuboid.points){
      const double vx = static_cast<double>(vertex.x - pcl_traj_pose.x);
      const double vy = static_cast<double>(vertex.y - pcl_traj_pose.y);
      const double vz = static_cast<double>(vertex.z - pcl_traj_pose.z);
      search_radius = std::max(search_radius, std::sqrt(vx*vx + vy*vy + vz*vz));
    }
    if(!std::isfinite(search_radius) || search_radius <= 0.0){
      RCLCPP_ERROR(node_->get_logger().get_child(name_),
        "Trajectory step %u has an invalid cuboid search radius; rejecting trajectory.", i);
      return -1.0;
    }

    std::vector<int> id;
    std::vector<float> sqdist;
    shared_data_->pcl_perception_kdtree_->radiusSearch(
      pcl_traj_pose, search_radius + 1e-3, id, sqdist);

    for(const int point_index : id){
      const auto& pct_point = shared_data_->pcl_perception_->points[point_index];
      pcl::PointXYZ dp;
      dp.x = pct_point.x-cuboid_center.x;
      dp.y = pct_point.y-cuboid_center.y;
      dp.z = pct_point.z-cuboid_center.z;

      const double x_value = std::fabs(dp.x * dx.x + dp.y * dx.y + dp.z * dx.z);
      const double y_value = std::fabs(dp.x * dy.x + dp.y * dy.y + dp.z * dy.z);
      const double z_value = std::fabs(dp.x * dz.x + dp.y * dz.y + dp.z * dz.z);

      if(x_value<=half_x && y_value<=half_y && z_value<=half_z){
        StairCollisionResult stair_result;
        if(stair_semantics_enabled && scoring_snapshot && scoring_ground &&
          scoring_ground_kdtree && !scoring_ground->empty())
        {
          std::vector<int> terrain_indices(1);
          std::vector<float> terrain_squared_distances(1);
          if(scoring_ground_kdtree->nearestKSearch(
              pct_point, 1, terrain_indices, terrain_squared_distances) > 0 &&
            terrain_indices.front() >= 0 &&
            static_cast<std::size_t>(terrain_indices.front()) < scoring_ground->size())
          {
            const std::size_t terrain_index =
              static_cast<std::size_t>(terrain_indices.front());
            const auto& terrain_point = scoring_ground->points[terrain_index];
            StairCollisionQuery query;
            query.riser_observation.snapshot = scoring_snapshot;
            query.riser_observation.terrain_ground_version = scoring_ground_version;
            query.riser_observation.now_nanoseconds = node_->now().nanoseconds();
            query.riser_observation.terrain_node_index = terrain_index;
            query.riser_observation.terrain_node_position = Eigen::Vector3f(
              terrain_point.x, terrain_point.y, terrain_point.z);
            query.riser_observation.obstacle_position = Eigen::Vector3f(
              pct_point.x, pct_point.y, pct_point.z);
            // The aggregate legacy XYZI cloud carries no reliable temporal
            // dynamic label.  False therefore means "not confirmed dynamic",
            // not proof that the return is static.  Geometry outside the thin
            // surveyed riser remains lethal; a perfectly co-planar return is
            // indistinguishable until a tracker supplies explicit evidence.
            query.riser_observation.dynamic_obstacle_confirmed = false;
            query.trajectory_origin_world = Eigen::Vector3f(
              pcl_traj_pose.x, pcl_traj_pose.y, pcl_traj_pose.z);
            query.body_axes_world = body_axes_world;
            query.active_support_staircase_id = active_support_staircase_id;
            query.active_support_location = active_support_location;
            stair_result = StairCollisionPolicy::evaluate(
              stair_collision_config_, query);
          }
        }
        if(stair_result.passthrough){
          used_stair_passthrough = true;
          RCLCPP_DEBUG_THROTTLE(
            node_->get_logger().get_child(name_), *(node_->get_clock()), 1000,
            "stair_riser_passthrough staircase=%d step=%d obstacle=(%.3f,%.3f,%.3f)",
            stair_result.staircase_id, stair_result.step_index,
            pct_point.x, pct_point.y, pct_point.z);
          continue;
        }
        RCLCPP_WARN_THROTTLE(
          node_->get_logger().get_child(name_), *(node_->get_clock()), 1000,
          "collision_reject traj_v=(%.3f,%.3f) step=%u/%u traj_pose=(%.3f,%.3f,%.3f) "
          "obstacle=(%.3f,%.3f,%.3f) cuboid_center=(%.3f,%.3f,%.3f) "
          "half=(%.3f,%.3f,%.3f) value=(%.3f,%.3f,%.3f) perception_points=%zu "
          "stair_collision_reason=%s stair_riser_reason=%s",
          traj.xv_, traj.thetav_, i + 1, traj.getPointsSize(),
          pcl_traj_pose.x, pcl_traj_pose.y, pcl_traj_pose.z,
          pct_point.x, pct_point.y, pct_point.z,
          cuboid_center.x, cuboid_center.y, cuboid_center.z,
          half_x, half_y, half_z,
          x_value, y_value, z_value,
          shared_data_->pcl_perception_->points.size(),
          mpc_critics::toString(stair_result.reason),
          perception_3d::toString(stair_result.riser_reason));
        return -1.0;
      }
    }
  }

  if(used_stair_passthrough &&
    (shared_data_->terrain_snapshot_ != scoring_snapshot ||
    shared_data_->terrain_ground_version_ != scoring_ground_version))
  {
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "Terrain snapshot changed while applying stair collision semantics; "
      "rejecting trajectory.");
    return -1.0;
  }

  return 0.0;
}

}//end of name space
