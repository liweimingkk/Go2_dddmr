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

#include <stdexcept>

namespace dddmr_pg_map_server
{
DDDMRPGMapServer::DDDMRPGMapServer(std::string name) : Node(name){

  clock_ = this->get_clock();

  declare_parameter("pose_graph_dir", rclcpp::ParameterValue(""));
  this->get_parameter("pose_graph_dir", pose_graph_dir_);
  RCLCPP_INFO(this->get_logger(), "pose_graph_dir: %s", pose_graph_dir_.c_str());

  source_map_sha256_ = normalizeMapSha256(
    this->declare_parameter<std::string>("source_map_sha256", ""));
  if (source_map_sha256_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "source_map_sha256 is missing/invalid; the computed artifact identity "
      "will still be reported, but terrain ROI activation requires an exact configured match");
  }

  complete_map_voxel_size_ =
    this->declare_parameter<double>("complete_map_voxel_size", 0.3);
  complete_ground_voxel_size_ = this->declare_parameter<double>(
    "complete_ground_voxel_size", complete_map_voxel_size_);

  const auto complete_map_voxel_result = validateVoxelSize(
    "complete_map_voxel_size", complete_map_voxel_size_);
  if (!complete_map_voxel_result.valid) {
    throw std::invalid_argument(complete_map_voxel_result.reason);
  }
  const auto complete_ground_voxel_result = validateVoxelSize(
    "complete_ground_voxel_size", complete_ground_voxel_size_);
  if (!complete_ground_voxel_result.valid) {
    throw std::invalid_argument(complete_ground_voxel_result.reason);
  }

  RCLCPP_INFO(
    this->get_logger(), "complete_map_voxel_size: %.3f",
    complete_map_voxel_size_);
  RCLCPP_INFO(
    this->get_logger(), "complete_ground_voxel_size: %.3f",
    complete_ground_voxel_size_);

  terrain_roi_config_.enabled =
    this->declare_parameter<bool>("terrain_roi_enabled", false);
  terrain_roi_config_.voxel_size =
    this->declare_parameter<double>("terrain_roi_voxel_size", 0.05);
  terrain_roi_config_.minimum = {{
    this->declare_parameter<double>("terrain_roi_min_x", 0.0),
    this->declare_parameter<double>("terrain_roi_min_y", 0.0),
    this->declare_parameter<double>("terrain_roi_min_z", 0.0)}};
  terrain_roi_config_.maximum = {{
    this->declare_parameter<double>("terrain_roi_max_x", 0.0),
    this->declare_parameter<double>("terrain_roi_max_y", 0.0),
    this->declare_parameter<double>("terrain_roi_max_z", 0.0)}};

  const auto terrain_roi_result = validateTerrainROIConfig(
    terrain_roi_config_, complete_ground_voxel_size_);
  if (!terrain_roi_result.valid) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Terrain ROI request rejected (fail closed): %s. "
      "No terrain_ground point cloud will be published.",
      terrain_roi_result.reason.c_str());
  } else if (terrain_roi_config_.enabled && source_map_sha256_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Terrain ROI request rejected (fail closed): source_map_sha256 must be a 64-hex "
      "hash of the loaded pose graph.");
  } else {
    terrain_roi_active_ = terrain_roi_config_.enabled;
  }

  access_ = new sub_maps_mutex_t();
  
  pub_key_pose_arr_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/key_poses",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());    

  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_surf_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/mapsurface",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_navigation_ground_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/navigation_ground",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pub_map_sha256_ = this->create_publisher<std_msgs::msg::String>(
    "~/map_sha256", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  if (terrain_roi_active_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Terrain ROI enabled: min [%.3f, %.3f, %.3f], "
      "max [%.3f, %.3f, %.3f], voxel %.3f",
      terrain_roi_config_.minimum[0], terrain_roi_config_.minimum[1],
      terrain_roi_config_.minimum[2], terrain_roi_config_.maximum[0],
      terrain_roi_config_.maximum[1], terrain_roi_config_.maximum[2],
      terrain_roi_config_.voxel_size);
  }
  
  srv_get_key_frame_ = this->create_service<dddmr_sys_core::srv::GetKeyFrameCloud>("~/get_key_frame_cloud", std::bind(&DDDMRPGMapServer::getKeyFrameCloud, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);

  readPoseGraph();

}

DDDMRPGMapServer::~DDDMRPGMapServer(){
  delete access_;
}

void DDDMRPGMapServer::readPoseGraph(){

  /*
  pcd_poses_ is original data. It is map frame to base_link, each pcd is base_link frame
  */

  pcd_poses_.reset(new pcl::PointCloud<PointTypePose>());
  if (pose_graph_dir_.empty()) {
    throw std::invalid_argument("pose_graph_dir must not be empty");
  }
  const std::string poses_file_path = pose_graph_dir_ + "/poses.pcd";
  std::vector<MapArtifact> map_artifacts{{"poses.pcd", poses_file_path}};
  if (pcl::io::loadPCDFile<PointTypePose> (poses_file_path, *pcd_poses_) == -1) //* load the file
  {
    throw std::runtime_error("failed to read pose graph poses.pcd from " + pose_graph_dir_);
  }
  if (pcd_poses_->empty()) {
    throw std::runtime_error("pose graph contains no poses: " + pose_graph_dir_);
  }
  RCLCPP_INFO(this->get_logger(), "Poses read: %lu", pcd_poses_->points.size());
  
  for(unsigned int it=0; it<pcd_poses_->points.size(); it++){
    //@ something like 0_feature.pcd
    std::string feature_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_feature.pcd";
    map_artifacts.push_back(
      {"pcd/" + std::to_string(it) + "_feature.pcd", feature_file_dir});
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_feature_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t> (feature_file_dir, *a_feature_pcd) == -1) //* load the file
    {
      throw std::runtime_error("failed to read feature PCD: " + feature_file_dir);
    }
    cornerCloudKeyFrames_baselink_.push_back(a_feature_pcd);

    std::string surface_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_surface.pcd";
    map_artifacts.push_back(
      {"pcd/" + std::to_string(it) + "_surface.pcd", surface_file_dir});
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_surface_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t> (surface_file_dir, *a_surface_pcd) == -1) //* load the file
    {
      throw std::runtime_error("failed to read surface PCD: " + surface_file_dir);
    }
    surfCloudKeyFrames_baselink_.push_back(a_surface_pcd);

    std::string ground_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_ground.pcd";
    map_artifacts.push_back(
      {"pcd/" + std::to_string(it) + "_ground.pcd", ground_file_dir});
    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_ground_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    if (pcl::io::loadPCDFile<dddmr_pg_map_server::pcl_t> (ground_file_dir, *a_ground_pcd) == -1) //* load the file
    {
      throw std::runtime_error("failed to read ground PCD: " + ground_file_dir);
    }
    groundCloudKeyFrames_baselink_.push_back(a_ground_pcd);
  }

  const auto map_identity = computeMapArtifactIdentity(map_artifacts);
  if (!map_identity.valid) {
    loaded_map_sha256_.clear();
    terrain_roi_active_ = false;
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not compute the loaded pose-graph identity: %s. Terrain outputs "
      "are disabled fail-closed; ordinary map outputs remain available.",
      map_identity.reason.c_str());
  } else {
    loaded_map_sha256_ = map_identity.sha256;
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded pose-graph artifact identity: %s (%zu artifacts, %llu bytes)",
      loaded_map_sha256_.c_str(), map_identity.artifact_count,
      static_cast<unsigned long long>(map_identity.total_bytes));

    const auto identity_validation = validateLoadedMapIdentity(
      terrain_roi_config_.enabled, source_map_sha256_, loaded_map_sha256_);
    if (!identity_validation.valid) {
      terrain_roi_active_ = false;
      RCLCPP_ERROR(
        this->get_logger(),
        "Terrain ROI identity verification failed closed: %s "
        "(configured=%s loaded=%s). No terrain-aware navigation ground will "
        "be published. If this map is intentional, independently verify it, "
        "then configure source_map_sha256 as %s.",
        identity_validation.reason.c_str(),
        source_map_sha256_.empty() ? "missing/invalid" : source_map_sha256_.c_str(),
        loaded_map_sha256_.c_str(), loaded_map_sha256_.c_str());
    } else if (terrain_roi_config_.enabled) {
      RCLCPP_INFO(
        this->get_logger(),
        "Configured source_map_sha256 matches the loaded artifact manifest.");
    } else if (!source_map_sha256_.empty() &&
      source_map_sha256_ != loaded_map_sha256_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Configured source_map_sha256 does not match the loaded artifacts "
        "(configured=%s loaded=%s). Terrain ROI is disabled, so legacy flat "
        "map/navigation publication remains enabled.",
        source_map_sha256_.c_str(), loaded_map_sha256_.c_str());
    }
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
      cornerCloudKeyFrames_.push_back(a_feature_pcd);
      *map_cloud += (*a_feature_pcd);
    }

    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_surf_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::PointCloud<dddmr_pg_map_server::pcl_t> a_surf_pcd_baselink = *surfCloudKeyFrames_baselink_[it];
    if(a_surf_pcd_baselink.points.size()>0){
      pcl::transformPointCloud(a_surf_pcd_baselink, *a_surf_pcd, trans_m2b_af3);
      surfCloudKeyFrames_.push_back(a_surf_pcd);
      *map_surf += (*a_surf_pcd);
    }

    pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr a_ground_pcd(new pcl::PointCloud<dddmr_pg_map_server::pcl_t>());
    pcl::PointCloud<dddmr_pg_map_server::pcl_t> a_ground_pcd_baselink = *groundCloudKeyFrames_baselink_[it];
    if(a_ground_pcd_baselink.points.size()>0){
      pcl::transformPointCloud(a_ground_pcd_baselink, *a_ground_pcd, trans_m2b_af3);
      groundCloudKeyFrames_.push_back(a_ground_pcd);
      *map_ground += (*a_ground_pcd);
    }

    dddmr_pg_map_server::pcl_t pt;
    pt.x = poses_.points[it].x;
    pt.y = poses_.points[it].y;
    pt.z = poses_.points[it].z;
    poses_pcl_t->push_back(pt);
  }

  RCLCPP_INFO(this->get_logger(), "\033[1;32mPose graph is loaded.\033[0m ");
  RCLCPP_INFO(this->get_logger(), "Map pointcloud size: %lu", map_cloud->points.size());
  RCLCPP_INFO(this->get_logger(), "Surface pointcloud size: %lu", map_surf->points.size());
  
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
  if (map_ground->empty()) {
    throw std::runtime_error("pose graph contains no ground points");
  }

  pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr terrain_ground(
    new pcl::PointCloud<dddmr_pg_map_server::pcl_t>);
  bool terrain_ground_ready = false;
  if (terrain_roi_active_) {
    terrain_ground->reserve(map_ground->size());
    for (const auto & point : map_ground->points) {
      if (pointIsInsideTerrainROI(
          terrain_roi_config_, point.x, point.y, point.z))
      {
        terrain_ground->push_back(point);
      }
    }

    if (terrain_ground->empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Terrain ROI contains no ground points; terrain_ground will not be "
        "published (fail closed).");
    } else {
      pcl::VoxelGrid<dddmr_pg_map_server::pcl_t> sor_terrain_ground;
      sor_terrain_ground.setInputCloud(terrain_ground);
      const auto terrain_voxel =
        static_cast<float>(terrain_roi_config_.voxel_size);
      sor_terrain_ground.setLeafSize(
        terrain_voxel, terrain_voxel, terrain_voxel);
      sor_terrain_ground.filter(*terrain_ground);
      terrain_ground->is_dense = false;
      std::vector<int> terrain_ground_indices;
      pcl::removeNaNFromPointCloud(
        *terrain_ground, *terrain_ground, terrain_ground_indices);

      if (terrain_ground->empty()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Terrain ROI became empty after downsampling; terrain_ground will "
          "not be advertised or published (fail closed).");
      } else {
        pub_terrain_ground_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/terrain_ground",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        sensor_msgs::msg::PointCloud2 terrain_ground_pc;
        pcl::toROSMsg(*terrain_ground, terrain_ground_pc);
        terrain_ground_pc.header.frame_id = "map";
        pub_terrain_ground_->publish(terrain_ground_pc);
        terrain_ground_ready = true;
        RCLCPP_INFO(
          this->get_logger(),
          "Terrain ground pointcloud published with %lu points.",
          terrain_ground->points.size());
      }
    }
  }

  pcl::VoxelGrid<dddmr_pg_map_server::pcl_t> sor_ground;
  sor_ground.setInputCloud (map_ground);
  const auto complete_ground_voxel =
    static_cast<float>(complete_ground_voxel_size_);
  sor_ground.setLeafSize(
    complete_ground_voxel, complete_ground_voxel,
    complete_ground_voxel);
  sor_ground.filter (*map_ground);
  map_ground->is_dense = false;
  std::vector<int> ind_ground;
  pcl::removeNaNFromPointCloud(*map_ground, *map_ground, ind_ground);
  RCLCPP_INFO(this->get_logger(), "Ground pointcloud size after down size: %lu", map_ground->points.size());
  sensor_msgs::msg::PointCloud2 ground_pc;
  pcl::toROSMsg(*map_ground, ground_pc);
  ground_pc.header.frame_id = "map";
  pub_ground_->publish(ground_pc);

  if (!terrain_roi_config_.enabled) {
    pub_navigation_ground_->publish(ground_pc);
    RCLCPP_INFO(
      this->get_logger(),
      "Navigation ground published from complete mapground with %lu points.",
      map_ground->points.size());
  } else if (!terrain_roi_active_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Navigation ground will not be published because the requested terrain "
      "ROI configuration is invalid (fail closed).");
  } else if (!terrain_ground_ready) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Navigation ground will not be published because terrain_ground is not "
      "ready (fail closed).");
  } else {
    const auto navigation_ground_result = mergeNavigationGroundPoints(
      map_ground->points, terrain_ground->points, terrain_roi_config_,
      complete_ground_voxel_size_);
    if (!navigation_ground_result.valid) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Navigation ground merge rejected (fail closed): %s. No "
        "navigation_ground point cloud will be published.",
        navigation_ground_result.reason.c_str());
    } else {
      pcl::PointCloud<dddmr_pg_map_server::pcl_t> navigation_ground;
      navigation_ground.points = navigation_ground_result.points;
      navigation_ground.width = navigation_ground.points.size();
      navigation_ground.height = 1;
      navigation_ground.is_dense = false;

      sensor_msgs::msg::PointCloud2 navigation_ground_pc;
      pcl::toROSMsg(navigation_ground, navigation_ground_pc);
      navigation_ground_pc.header.frame_id = "map";
      pub_navigation_ground_->publish(navigation_ground_pc);
      RCLCPP_INFO(
        this->get_logger(),
        "Navigation ground published with %lu coarse ROI-outside points, "
        "%lu high-resolution ROI points, and %lu duplicates removed.",
        navigation_ground_result.coarse_points_retained,
        navigation_ground_result.terrain_points_added,
        navigation_ground_result.duplicate_points_removed);
    }
  }
  
  key_poses_.header.frame_id = "map";
  pub_key_pose_arr_->publish(key_poses_);

  if (!loaded_map_sha256_.empty()) {
    std_msgs::msg::String map_identity;
    // Publish only the digest computed from what was actually loaded.  A
    // configured value is a verifier, never an authority that can overwrite
    // the measured identity.
    map_identity.data = loaded_map_sha256_;
    pub_map_sha256_->publish(map_identity);
  }

  RCLCPP_INFO(this->get_logger(), "\033[1;32mMap and Ground published.\033[0m ");
  

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
