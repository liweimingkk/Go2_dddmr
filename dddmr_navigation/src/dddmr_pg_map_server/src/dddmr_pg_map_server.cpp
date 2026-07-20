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

#include <dddmr_pg_map_server.h>

#include <cmath>
#include <stdexcept>

#include <pcl/common/point_tests.h>

namespace dddmr_pg_map_server
{
DDDMRPGMapServer::DDDMRPGMapServer(
    const std::string & name, const rclcpp::NodeOptions & options)
    : Node(name, options){

  clock_ = this->get_clock();

  declare_parameter("pose_graph_dir", rclcpp::ParameterValue(""));
  this->get_parameter("pose_graph_dir", pose_graph_dir_);
  RCLCPP_INFO(this->get_logger(), "pose_graph_dir: %s", pose_graph_dir_.c_str());

  declare_parameter("complete_map_voxel_size", rclcpp::ParameterValue(0.3f));
  this->get_parameter("complete_map_voxel_size", complete_map_voxel_size_);
  RCLCPP_INFO(this->get_logger(), "complete_map_voxel_size: %.2f", complete_map_voxel_size_);

  // Keep the historical behavior when the ground-specific parameter is not
  // configured, while allowing mapground to retain more geometric detail than
  // mapcloud when required.
  declare_parameter(
      "complete_ground_voxel_size",
      rclcpp::ParameterValue(complete_map_voxel_size_));
  this->get_parameter(
      "complete_ground_voxel_size", complete_ground_voxel_size_);
  RCLCPP_INFO(
      this->get_logger(), "complete_ground_voxel_size: %.2f",
      complete_ground_voxel_size_);

  declare_parameter(
      "merge_non_ground_surface_into_mapcloud",
      rclcpp::ParameterValue(false));
  this->get_parameter(
      "merge_non_ground_surface_into_mapcloud",
      merge_non_ground_surface_into_mapcloud_);
  RCLCPP_INFO(
      this->get_logger(), "merge_non_ground_surface_into_mapcloud: %s",
      merge_non_ground_surface_into_mapcloud_ ? "true" : "false");

  declare_parameter(
      "surface_ground_exclusion_radius", rclcpp::ParameterValue(0.1f));
  this->get_parameter(
      "surface_ground_exclusion_radius", surface_ground_exclusion_radius_);
  RCLCPP_INFO(
      this->get_logger(), "surface_ground_exclusion_radius: %.2f",
      surface_ground_exclusion_radius_);

  declare_parameter("planning_ground_overlay_path", rclcpp::ParameterValue(""));
  this->get_parameter(
    "planning_ground_overlay_path", planning_ground_overlay_path_);

  declare_parameter(
    "planning_ground_overlay_max_distance", rclcpp::ParameterValue(0.32));
  this->get_parameter(
    "planning_ground_overlay_max_distance",
    planning_ground_overlay_max_distance_);

  declare_parameter(
    "planning_ground_overlay_max_fraction", rclcpp::ParameterValue(0.05));
  this->get_parameter(
    "planning_ground_overlay_max_fraction",
    planning_ground_overlay_max_fraction_);

  declare_parameter(
    "planning_ground_voxel_size",
    rclcpp::ParameterValue(static_cast<double>(complete_ground_voxel_size_)));
  this->get_parameter(
    "planning_ground_voxel_size", planning_ground_voxel_size_);

  if (!std::isfinite(planning_ground_overlay_max_distance_) ||
    planning_ground_overlay_max_distance_ <= 0.0)
  {
    throw std::invalid_argument(
            "planning_ground_overlay_max_distance must be finite and positive");
  }
  if (!std::isfinite(planning_ground_overlay_max_fraction_) ||
    planning_ground_overlay_max_fraction_ < 0.0 ||
    planning_ground_overlay_max_fraction_ > 0.05)
  {
    throw std::invalid_argument(
            "planning_ground_overlay_max_fraction must be in [0.0, 0.05]");
  }
  if (!std::isfinite(planning_ground_voxel_size_) ||
    planning_ground_voxel_size_ <= 0.0)
  {
    throw std::invalid_argument(
            "planning_ground_voxel_size must be finite and positive");
  }
  RCLCPP_INFO(
    this->get_logger(),
    "planning ground overlay: path='%s', max_distance=%.3f m, "
    "max_fraction=%.3f, voxel_size=%.3f m",
    planning_ground_overlay_path_.c_str(),
    planning_ground_overlay_max_distance_,
    planning_ground_overlay_max_fraction_, planning_ground_voxel_size_);
  
  pub_key_pose_arr_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/key_poses",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());    

  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_surf_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/mapsurface",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_planning_ground_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/planning_ground",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  
  srv_get_key_frame_ = this->create_service<dddmr_sys_core::srv::GetKeyFrameCloud>("~/get_key_frame_cloud", std::bind(&DDDMRPGMapServer::getKeyFrameCloud, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);

  readPoseGraph();

  access_ = new sub_maps_mutex_t();

}

DDDMRPGMapServer::~DDDMRPGMapServer(){
  delete access_;
}

pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr
DDDMRPGMapServer::buildPlanningGround(
  const pcl::PointCloud<dddmr_pg_map_server::pcl_t>::ConstPtr & observed_ground)
{
  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr planning_ground(
    new pcl::PointCloud<dddmr_pg_map_server::pcl_t>(*observed_ground));

  if (planning_ground_overlay_path_.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Planning ground uses %zu observed mapground points; no overlay configured",
      observed_ground->size());
    return planning_ground;
  }

  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr overlay(
    new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
  if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t>(
      planning_ground_overlay_path_, *overlay) < 0)
  {
    const std::string error =
      "Failed to load planning ground overlay PCD: " +
      planning_ground_overlay_path_;
    RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
    throw std::runtime_error(error);
  }

  if (!overlay->empty() && observed_ground->empty()) {
    const std::string error =
      "Planning ground overlay is non-empty but observed mapground is empty";
    RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
    throw std::runtime_error(error);
  }

  const std::size_t allowed_overlay_points = static_cast<std::size_t>(
    std::floor(
      static_cast<double>(observed_ground->size()) *
      planning_ground_overlay_max_fraction_));
  if (overlay->size() > allowed_overlay_points) {
    const std::string error =
      "Planning ground overlay contains " + std::to_string(overlay->size()) +
      " points, exceeding the configured safe limit of " +
      std::to_string(allowed_overlay_points) + " (" +
      std::to_string(planning_ground_overlay_max_fraction_ * 100.0) +
      "% of " + std::to_string(observed_ground->size()) +
      " observed points)";
    RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
    throw std::runtime_error(error);
  }

  pcl::search::KdTree<dddmr_pg_map_server::pcl_t> observed_ground_kdtree;
  if (!overlay->empty()) {
    observed_ground_kdtree.setInputCloud(observed_ground);
  }
  const double max_squared_distance =
    planning_ground_overlay_max_distance_ *
    planning_ground_overlay_max_distance_;
  double largest_nearest_distance = 0.0;
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  for (std::size_t index = 0; index < overlay->size(); ++index) {
    const auto & point = overlay->points[index];
    if (!pcl::isFinite(point) || !std::isfinite(point.intensity)) {
      const std::string error =
        "Planning ground overlay contains a non-finite point at index " +
        std::to_string(index);
      RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
      throw std::runtime_error(error);
    }

    if (observed_ground_kdtree.nearestKSearch(
        point, 1, nearest_index, nearest_squared_distance) != 1 ||
      !std::isfinite(nearest_squared_distance.front()))
    {
      const std::string error =
        "Could not associate planning ground overlay point " +
        std::to_string(index) + " with observed mapground";
      RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
      throw std::runtime_error(error);
    }

    const double squared_distance = nearest_squared_distance.front();
    largest_nearest_distance = std::max(
      largest_nearest_distance, std::sqrt(squared_distance));
    if (squared_distance > max_squared_distance) {
      const std::string error =
        "Planning ground overlay point " + std::to_string(index) +
        " is " + std::to_string(std::sqrt(squared_distance)) +
        " m from observed mapground, exceeding " +
        std::to_string(planning_ground_overlay_max_distance_) + " m";
      RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
      throw std::runtime_error(error);
    }
  }

  *planning_ground += *overlay;
  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr filtered_planning_ground(
    new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
  pcl::VoxelGrid<dddmr_pg_map_server::pcl_t> planning_ground_voxel_filter;
  planning_ground_voxel_filter.setInputCloud(planning_ground);
  const float voxel_size = static_cast<float>(planning_ground_voxel_size_);
  planning_ground_voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  planning_ground_voxel_filter.filter(*filtered_planning_ground);
  filtered_planning_ground->is_dense = false;
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(
    *filtered_planning_ground, *filtered_planning_ground, finite_indices);

  RCLCPP_INFO(
    this->get_logger(),
    "Planning ground accepted %zu overlay points near %zu observed points "
    "(largest nearest distance %.3f m); merged/voxelized output has %zu points",
    overlay->size(), observed_ground->size(), largest_nearest_distance,
    filtered_planning_ground->size());
  return filtered_planning_ground;
}

void DDDMRPGMapServer::readPoseGraph(){

  /*
  pcd_poses_ is original data. It is map frame to base_link, each pcd is base_link frame
  */

  pcd_poses_.reset(new pcl::PointCloud<PointTypePose>());
  if (pcl::io::loadPCDFile<PointTypePose> (pose_graph_dir_ + "/poses.pcd", *pcd_poses_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read poses PCD file fail: %s", pose_graph_dir_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Poses read: %lu", pcd_poses_->points.size());
  
  for(unsigned int it=0; it<pcd_poses_->points.size(); it++){
    //@ something like 0_feature.pcd
    std::string feature_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_feature.pcd";
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_feature_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t> (feature_file_dir, *a_feature_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read feature PCD file fail: %s", feature_file_dir.c_str());
    }
    cornerCloudKeyFrames_baselink_.push_back(a_feature_pcd);

    std::string surface_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_surface.pcd";
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_surface_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t> (surface_file_dir, *a_surface_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read feature PCD file fail: %s", surface_file_dir.c_str());
    }
    surfCloudKeyFrames_baselink_.push_back(a_surface_pcd);

    std::string ground_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_ground.pcd";
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_ground_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t> (ground_file_dir, *a_ground_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read ground PCD file fail: %s", ground_file_dir.c_str());
    }
    groundCloudKeyFrames_baselink_.push_back(a_ground_pcd);
  }

  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr map_cloud (new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr map_surf (new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr map_ground (new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);  
  poses_ = *pcd_poses_;
  //@----- update keyframe poses ---
  cornerCloudKeyFrames_.clear();
  surfCloudKeyFrames_.clear();
  groundCloudKeyFrames_.clear();
  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr poses_pcl_t(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
  for(unsigned int it=0; it<poses_.points.size(); it++){

    geometry_msgs::msg::TransformStamped trans_m2b;
    Eigen::Affine3d trans_m2b_af3;
    trans_m2b.transform.translation.x = poses_.points[it].x;
    trans_m2b.transform.translation.y = poses_.points[it].y;
    trans_m2b.transform.translation.z = poses_.points[it].z;
    tf2::Quaternion q;
    q.setRPY( poses_.points[it].roll, poses_.points[it].pitch, poses_.points[it].yaw);
    trans_m2b.transform.rotation.x = q.x(); trans_m2b.transform.rotation.y = q.y();
    trans_m2b.transform.rotation.z = q.z(); trans_m2b.transform.rotation.w = q.w();
    trans_m2b_af3 = tf2::transformToEigen(trans_m2b);

    //@ push to pose array for publishing
    geometry_msgs::msg::Pose a_pose;
    a_pose.position.x = poses_.points[it].x;
    a_pose.position.y = poses_.points[it].y;
    a_pose.position.z = poses_.points[it].z;
    a_pose.orientation.x = trans_m2b.transform.rotation.x;
    a_pose.orientation.y = trans_m2b.transform.rotation.y;
    a_pose.orientation.z = trans_m2b.transform.rotation.z;
    a_pose.orientation.w = trans_m2b.transform.rotation.w;
    key_poses_.poses.push_back(a_pose);
    
    //@transform to map frame
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_feature_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::PointCloud<dddmr_pg_map_server::pcl_t> a_feature_pcd_baselink = *cornerCloudKeyFrames_baselink_[it];
    if(a_feature_pcd_baselink.points.size()>0){
      pcl::transformPointCloud(a_feature_pcd_baselink, *a_feature_pcd, trans_m2b_af3);
      *map_cloud += (*a_feature_pcd);
    }
    // Preserve one entry per pose even when a keyframe PCD is empty or could
    // not be read. get_key_frame_cloud uses the pose number as a direct index.
    cornerCloudKeyFrames_.push_back(a_feature_pcd);

    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_surf_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::PointCloud<dddmr_pg_map_server::pcl_t> a_surf_pcd_baselink = *surfCloudKeyFrames_baselink_[it];
    if(a_surf_pcd_baselink.points.size()>0){
      pcl::transformPointCloud(a_surf_pcd_baselink, *a_surf_pcd, trans_m2b_af3);
      *map_surf += (*a_surf_pcd);
    }
    surfCloudKeyFrames_.push_back(a_surf_pcd);

    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_ground_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::PointCloud<dddmr_pg_map_server::pcl_t> a_ground_pcd_baselink = *groundCloudKeyFrames_baselink_[it];
    if(a_ground_pcd_baselink.points.size()>0){
      pcl::transformPointCloud(a_ground_pcd_baselink, *a_ground_pcd, trans_m2b_af3);
      *map_ground += (*a_ground_pcd);
    }
    groundCloudKeyFrames_.push_back(a_ground_pcd);

    dddmr_pg_map_server::pcl_t pt;
    pt.x = poses_.points[it].x;
    pt.y = poses_.points[it].y;
    pt.z = poses_.points[it].z;
    poses_pcl_t->push_back(pt);
  }

  RCLCPP_INFO(this->get_logger(), "\033[1;32mPose graph is loaded.\033[0m ");
  RCLCPP_INFO(this->get_logger(), "Map pointcloud size: %lu", map_cloud->points.size());
  RCLCPP_INFO(this->get_logger(), "Surface pointcloud size: %lu", map_surf->points.size());

  if (merge_non_ground_surface_into_mapcloud_) {
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr finite_ground(
      new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
    std::vector<int> finite_ground_indices;
    pcl::removeNaNFromPointCloud(
      *map_ground, *finite_ground, finite_ground_indices);

    pcl::search::KdTree<dddmr_pg_map_server::pcl_t>::Ptr ground_kdtree;
    if (!finite_ground->empty()) {
      ground_kdtree.reset(
        new pcl::search::KdTree<dddmr_pg_map_server::pcl_t>);
      ground_kdtree->setInputCloud(finite_ground);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "mapground is empty; merging all finite mapsurface points into mapcloud");
    }

    std::size_t merged_surface_points = 0;
    std::size_t excluded_ground_surface_points = 0;
    std::size_t invalid_surface_points = 0;
    std::vector<int> ground_neighbors;
    std::vector<float> ground_squared_distances;
    for (const auto & point : map_surf->points) {
      if (!pcl::isFinite(point)) {
        ++invalid_surface_points;
        continue;
      }

      bool overlaps_ground = false;
      if (ground_kdtree) {
        ground_neighbors.clear();
        ground_squared_distances.clear();
        overlaps_ground = ground_kdtree->radiusSearch(
          point, surface_ground_exclusion_radius_, ground_neighbors,
          ground_squared_distances, 1) > 0;
      }

      if (overlaps_ground) {
        ++excluded_ground_surface_points;
        continue;
      }

      map_cloud->push_back(point);
      ++merged_surface_points;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Merged %zu non-ground surface points into mapcloud; excluded %zu "
      "ground-overlapping and %zu invalid surface points",
      merged_surface_points, excluded_ground_surface_points,
      invalid_surface_points);
  }
  
  //@ euc to filter noisy data
  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr map_cloud_after_euc (new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
  pcl::search::KdTree<dddmr_pg_map_server::pcl_t>::Ptr pc_kdtree (new pcl::search::KdTree<dddmr_pg_map_server::pcl_t>);
  pc_kdtree->setInputCloud (map_cloud);
  std::vector<pcl::PointIndices> cluster_indices_segmentation;
  pcl::EuclideanClusterExtraction<dddmr_pg_map_server::pcl_t> ec_segmentation;
  ec_segmentation.setClusterTolerance (0.2);
  ec_segmentation.setMinClusterSize (1);
  ec_segmentation.setMaxClusterSize (map_cloud->points.size());
  ec_segmentation.setSearchMethod (pc_kdtree);
  ec_segmentation.setInputCloud (map_cloud);
  ec_segmentation.extract (cluster_indices_segmentation);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
  {
    if(it->indices.size()<10)
      continue;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      map_cloud_after_euc->push_back(map_cloud->points[(*pit)]);
    } 
  }

  pcl::VoxelGrid<dddmr_pg_map_server::pcl_t> sor_map;
  sor_map.setInputCloud (map_cloud_after_euc);
  sor_map.setLeafSize (complete_map_voxel_size_, complete_map_voxel_size_, complete_map_voxel_size_);
  sor_map.filter (*map_cloud_after_euc);
  map_cloud_after_euc->is_dense = false;
  std::vector<int> ind_map;
  pcl::removeNaNFromPointCloud(*map_cloud_after_euc, *map_cloud_after_euc, ind_map);
  RCLCPP_INFO(this->get_logger(), "Map pointcloud size after down size: %lu", map_cloud_after_euc->points.size());
  sensor_msgs::msg::PointCloud2 map_pc;
  pcl::toROSMsg(*map_cloud_after_euc, map_pc);
  map_pc.header.frame_id = "map";
  pub_map_->publish(map_pc);

  pcl::VoxelGrid<dddmr_pg_map_server::pcl_t> sor_map_surf;
  sor_map_surf.setInputCloud (map_surf);
  sor_map_surf.setLeafSize (complete_map_voxel_size_, complete_map_voxel_size_, complete_map_voxel_size_);
  sor_map_surf.filter (*map_surf);
  map_surf->is_dense = false;
  std::vector<int> ind_map_surf;
  pcl::removeNaNFromPointCloud(*map_surf, *map_surf, ind_map_surf);
  RCLCPP_INFO(this->get_logger(), "Surf pointcloud size after down size: %lu", map_surf->points.size());
  sensor_msgs::msg::PointCloud2 surf_pc;
  pcl::toROSMsg(*map_surf, surf_pc);
  surf_pc.header.frame_id = "map";
  pub_surf_->publish(surf_pc);

  RCLCPP_INFO(this->get_logger(), "Ground pointcloud size: %lu", map_ground->points.size());
  pcl::VoxelGrid<dddmr_pg_map_server::pcl_t> sor_ground;
  sor_ground.setInputCloud (map_ground);
  sor_ground.setLeafSize (
      complete_ground_voxel_size_, complete_ground_voxel_size_,
      complete_ground_voxel_size_);
  sor_ground.filter (*map_ground);
  map_ground->is_dense = false;
  std::vector<int> ind_ground;
  pcl::removeNaNFromPointCloud(*map_ground, *map_ground, ind_ground);
  RCLCPP_INFO(this->get_logger(), "Ground pointcloud size after down size: %lu", map_ground->points.size());
  sensor_msgs::msg::PointCloud2 ground_pc;
  pcl::toROSMsg(*map_ground, ground_pc);
  ground_pc.header.frame_id = "map";
  pub_ground_->publish(ground_pc);

  // The optional synthetic overlay is deliberately introduced only after the
  // observed mapcloud, mapsurface, and mapground products are complete.  It is
  // never used by surface exclusion or returned as a keyframe cloud.
  const auto planning_ground = buildPlanningGround(map_ground);
  sensor_msgs::msg::PointCloud2 planning_ground_pc;
  pcl::toROSMsg(*planning_ground, planning_ground_pc);
  planning_ground_pc.header = ground_pc.header;
  pub_planning_ground_->publish(planning_ground_pc);
  
  key_poses_.header.frame_id = "map";
  pub_key_pose_arr_->publish(key_poses_);

  RCLCPP_INFO(
    this->get_logger(),
    "\033[1;32mMap, observed ground, and planning ground published.\033[0m ");
  

}

void DDDMRPGMapServer::getKeyFrameCloud(const std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Request> request,
          std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Response> response){
  
  if(request->key_frame_number>=cornerCloudKeyFrames_.size()){
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr emptyFrame;
    emptyFrame.reset(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::toROSMsg(*emptyFrame, response->key_frame_cloud);
    pcl::toROSMsg(*emptyFrame, response->key_frame_cloud_base_link);
  }
  else{
    pcl::toROSMsg(*cornerCloudKeyFrames_[request->key_frame_number], response->key_frame_cloud);
    pcl::toROSMsg(*cornerCloudKeyFrames_baselink_[request->key_frame_number], response->key_frame_cloud_base_link);
  }

  if(request->key_frame_number>=surfCloudKeyFrames_.size()){
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr emptyFrame;
    emptyFrame.reset(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::toROSMsg(*emptyFrame, response->key_frame_surface);
    pcl::toROSMsg(*emptyFrame, response->key_frame_surface_base_link);
  }
  else{
    pcl::toROSMsg(*surfCloudKeyFrames_[request->key_frame_number], response->key_frame_surface);
    pcl::toROSMsg(*surfCloudKeyFrames_baselink_[request->key_frame_number], response->key_frame_surface_base_link);
  }

  if(request->key_frame_number>=groundCloudKeyFrames_.size()){
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr emptyFrame;
    emptyFrame.reset(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::toROSMsg(*emptyFrame, response->key_frame_ground);
    pcl::toROSMsg(*emptyFrame, response->key_frame_ground_base_link);
  }
  else{
    pcl::toROSMsg(*groundCloudKeyFrames_[request->key_frame_number], response->key_frame_ground);
    pcl::toROSMsg(*groundCloudKeyFrames_baselink_[request->key_frame_number], response->key_frame_ground_base_link);
  }

}


}
