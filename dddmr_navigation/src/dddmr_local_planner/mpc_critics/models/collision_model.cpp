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
        RCLCPP_WARN_THROTTLE(
          node_->get_logger().get_child(name_), *(node_->get_clock()), 1000,
          "collision_reject traj_v=(%.3f,%.3f) step=%u/%u traj_pose=(%.3f,%.3f,%.3f) "
          "obstacle=(%.3f,%.3f,%.3f) cuboid_center=(%.3f,%.3f,%.3f) "
          "half=(%.3f,%.3f,%.3f) value=(%.3f,%.3f,%.3f) perception_points=%zu",
          traj.xv_, traj.thetav_, i + 1, traj.getPointsSize(),
          pcl_traj_pose.x, pcl_traj_pose.y, pcl_traj_pose.z,
          pct_point.x, pct_point.y, pct_point.z,
          cuboid_center.x, cuboid_center.y, cuboid_center.z,
          half_x, half_y, half_z,
          x_value, y_value, z_value,
          shared_data_->pcl_perception_->points.size());
        return -1.0;
      }
    }
  }

  return 0.0;
}

}//end of name space
