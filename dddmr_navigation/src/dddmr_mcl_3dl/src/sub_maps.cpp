/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mcl_3dl/sub_maps.h>

#include <array>

namespace mcl_3dl
{
SubMaps::SubMaps(std::string name) : Node(name), is_initial_(false),
  is_current_ready_(false), key_poses_received_(false),
  key_frames_ready_(false), next_key_frame_index_(0), global_map_received_(false),
  global_ground_received_(false), is_global_ready_(false),
  map_snapshot_invalidated_(false), prepare_warm_up_(false),
  is_warm_up_ready_(false){
  
  map_current_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  ground_current_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  map_warmup_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  ground_warmup_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  map_global_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  feature_global_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  ground_global_ = std::make_shared<pcl::PointCloud<pcl_t>>();

  clock_ = this->get_clock();
  
  access_ = new sub_maps_mutex_t();

  declare_parameter("pg_map_server_name", rclcpp::ParameterValue(""));
  this->get_parameter("pg_map_server_name", pg_map_server_name_);
  RCLCPP_INFO(this->get_logger(), "pg_map_server_name: %s", pg_map_server_name_.c_str());

  declare_parameter("sub_map_search_radius", rclcpp::ParameterValue(50.0));
  this->get_parameter("sub_map_search_radius", sub_map_search_radius_);
  RCLCPP_INFO(this->get_logger(), "sub_map_search_radius: %.1f", sub_map_search_radius_);

  declare_parameter("sub_map_warmup_trigger_distance", rclcpp::ParameterValue(20.0));
  this->get_parameter("sub_map_warmup_trigger_distance", sub_map_warmup_trigger_distance_);
  RCLCPP_INFO(this->get_logger(), "sub_map_warmup_trigger_distance: %.1f", sub_map_warmup_trigger_distance_);
  
  srv_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  get_key_frame_cloud_client_ = this->create_client<dddmr_sys_core::srv::GetKeyFrameCloud>(
    pg_map_server_name_ + "/get_key_frame_cloud", rmw_qos_profile_services_default, srv_group_);

  sub_key_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    pg_map_server_name_ + "/key_poses", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&SubMaps::keyPosesCb, this, std::placeholders::_1));

  sub_global_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pg_map_server_name_ + "/mapcloud",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SubMaps::globalMapCb, this, std::placeholders::_1));

  sub_global_ground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pg_map_server_name_ + "/mapground",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SubMaps::globalGroundCb, this, std::placeholders::_1));

  //@ Latched topic, Create a publisher using the QoS settings to emulate a ROS1 latched topic
  pub_sub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_map_warmup_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapcloud_warmup",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_ground_warmup_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapground_warmup",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sync_map_timer_ = this->create_wall_timer(1ms, std::bind(&SubMaps::syncMapThread, this), timer_group_);
  warm_up_timer_ = this->create_wall_timer(200ms, std::bind(&SubMaps::warmUpThread, this), timer_group_);
}

SubMaps::~SubMaps(){
  delete access_;
}

void SubMaps::keyPosesCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (key_poses_received_.load())
  {
    map_snapshot_invalidated_.store(true);
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "Replacement key poses invalidated the immutable relocalization map; "
        "restart MCL to load a new map generation");
    return;
  }
  if (msg->poses.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Received an empty key-pose array");
    return;
  }
  poses_pcl_t_.reset(new pcl::PointCloud<pcl_t>());
  key_poses_ = msg->poses;
  for(auto i=msg->poses.begin();i!=msg->poses.end();i++){
    pcl_t pt;
    pt.x = (*i).position.x;
    pt.y = (*i).position.y;
    pt.z = (*i).position.z;
    poses_pcl_t_->push_back(pt);
  }
  kdtree_poses_.reset(new pcl::KdTreeFLANN<pcl_t>());
  kdtree_poses_->setInputCloud(poses_pcl_t_);
  cornerCloudKeyFrames_.clear();
  cornerCloudKeyFrames_.reserve(key_poses_.size());
  surfCloudKeyFrames_.clear();
  surfCloudKeyFrames_.reserve(key_poses_.size());
  groundCloudKeyFrames_.clear();
  groundCloudKeyFrames_.reserve(key_poses_.size());
  cornerCloudKeyFrames_baselink_.clear();
  cornerCloudKeyFrames_baselink_.reserve(key_poses_.size());
  surfCloudKeyFrames_baselink_.clear();
  surfCloudKeyFrames_baselink_.reserve(key_poses_.size());
  groundCloudKeyFrames_baselink_.clear();
  groundCloudKeyFrames_baselink_.reserve(key_poses_.size());
  kdtree_surface_key_frames_.clear();
  feature_global_->clear();
  next_key_frame_index_.store(0);
  key_frames_ready_.store(false);
  is_initial_.store(false);
  key_poses_received_.store(true);
  if (sync_map_timer_)
  {
    sync_map_timer_->reset();
  }
  prepareGlobalMapLocked();
}

void SubMaps::globalMapCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (global_map_received_)
  {
    map_snapshot_invalidated_.store(true);
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "Replacement complete map invalidated the immutable relocalization map; "
        "restart MCL to load a new map generation");
    return;
  }
  pcl::fromROSMsg(*msg, *map_global_);
  global_map_received_ = !map_global_->empty();
  if (!global_map_received_)
  {
    RCLCPP_ERROR(this->get_logger(), "Received an empty complete map from %s/mapcloud",
      pg_map_server_name_.c_str());
  }
  prepareGlobalMapLocked();
}

void SubMaps::globalGroundCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (global_ground_received_)
  {
    map_snapshot_invalidated_.store(true);
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "Replacement ground map invalidated the immutable relocalization map; "
        "restart MCL to load a new map generation");
    return;
  }
  pcl::fromROSMsg(*msg, *ground_global_);
  global_ground_received_ = !ground_global_->empty();
  if (!global_ground_received_)
  {
    RCLCPP_ERROR(this->get_logger(), "Received an empty complete ground map from %s/mapground",
      pg_map_server_name_.c_str());
  }
  prepareGlobalMapLocked();
}

void SubMaps::prepareGlobalMapLocked()
{
  if (is_global_ready_.load() || !key_poses_received_.load() ||
      !global_map_received_ || !global_ground_received_)
  {
    return;
  }

  kdtree_map_global_.setInputCloud(map_global_);
  kdtree_ground_global_.setInputCloud(ground_global_);

  pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> normal_estimator;
  pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree(
    new pcl::search::KdTree<mcl_3dl::pcl_t>);
  tree->setInputCloud(ground_global_);
  normal_estimator.setInputCloud(ground_global_);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setKSearch(
    std::min(20, static_cast<int>(ground_global_->points.size())));
  normal_estimator.compute(normals_ground_global_);

  is_global_ready_.store(true);
  RCLCPP_INFO(
    this->get_logger(),
    "Global relocalization map ready: %lu map points, %lu ground points, %lu key poses",
    map_global_->points.size(), ground_global_->points.size(), key_poses_.size());
}

std::vector<geometry_msgs::msg::Pose> SubMaps::getKeyPoses()
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  return key_poses_;
}

float SubMaps::keyFrameSurfaceMatchRatio(
    const std::size_t key_frame_index,
    const pcl::PointCloud<pcl_t>& observation_in_map,
    const double match_distance) const
{
  if (!areKeyFramesReady() || observation_in_map.empty() ||
      !std::isfinite(match_distance) || match_distance <= 0.0 ||
      key_frame_index >= kdtree_surface_key_frames_.size())
  {
    return 0.0f;
  }

  const auto& tree = kdtree_surface_key_frames_[key_frame_index];
  if (!tree || !tree->getInputCloud() || tree->getInputCloud()->empty())
  {
    return 0.0f;
  }

  std::size_t matched = 0;
  std::vector<int> indices;
  std::vector<float> squared_distances;
  for (const auto& point : observation_in_map.points)
  {
    indices.clear();
    squared_distances.clear();
    if (tree->radiusSearch(
          point, match_distance, indices, squared_distances, 1) > 0)
    {
      ++matched;
    }
  }
  return static_cast<float>(matched) /
    static_cast<float>(observation_in_map.size());
}

void SubMaps::finishKeyFrameSyncLocked()
{
  const std::size_t expected = key_poses_.size();
  if (cornerCloudKeyFrames_.size() != expected ||
      surfCloudKeyFrames_.size() != expected ||
      groundCloudKeyFrames_.size() != expected ||
      cornerCloudKeyFrames_baselink_.size() != expected ||
      surfCloudKeyFrames_baselink_.size() != expected ||
      groundCloudKeyFrames_baselink_.size() != expected)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Ordered keyframe synchronization is incomplete: expected %lu, got "
        "feature=%lu surface=%lu ground=%lu",
        expected, cornerCloudKeyFrames_.size(), surfCloudKeyFrames_.size(),
        groundCloudKeyFrames_.size());
    return;
  }

  feature_global_->clear();
  kdtree_surface_key_frames_.clear();
  kdtree_surface_key_frames_.reserve(surfCloudKeyFrames_.size());
  std::size_t surface_point_count = 0;
  for (std::size_t index = 0; index < cornerCloudKeyFrames_.size(); ++index)
  {
    if (cornerCloudKeyFrames_[index])
    {
      *feature_global_ += *cornerCloudKeyFrames_[index];
    }

    auto tree = std::make_shared<pcl::KdTreeFLANN<pcl_t>>();
    if (index < surfCloudKeyFrames_.size() && surfCloudKeyFrames_[index] &&
        !surfCloudKeyFrames_[index]->empty())
    {
      tree->setInputCloud(surfCloudKeyFrames_[index]);
      surface_point_count += surfCloudKeyFrames_[index]->size();
    }
    kdtree_surface_key_frames_.push_back(tree);
  }

  if (feature_global_->empty() || surface_point_count == 0)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Synchronized keyframes lack relocalization data: feature=%lu surface=%lu",
        feature_global_->size(), surface_point_count);
    return;
  }
  kdtree_feature_global_.setInputCloud(feature_global_);
  key_frames_ready_.store(true);
  is_initial_.store(true);
  RCLCPP_INFO(
    this->get_logger(),
    "Synchronized %lu ordered keyframes with %lu global feature points",
    cornerCloudKeyFrames_.size(), feature_global_->size());
}

void SubMaps::syncMapThread()
{
  if (map_snapshot_invalidated_.load())
  {
    if (pending_key_frame_request_)
    {
      get_key_frame_cloud_client_->remove_pending_request(
          *pending_key_frame_request_);
      pending_key_frame_request_.reset();
    }
    sync_map_timer_->cancel();
    return;
  }
  if (!key_poses_received_.load() || key_frames_ready_.load())
  {
    return;
  }

  constexpr auto request_timeout = std::chrono::seconds(2);
  if (pending_key_frame_request_)
  {
    if (pending_key_frame_request_->wait_for(0ms) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() - key_frame_request_started_ <
          request_timeout)
      {
        return;
      }

      const std::size_t timed_out_index = next_key_frame_index_.load();
      if (!get_key_frame_cloud_client_->remove_pending_request(
            *pending_key_frame_request_))
      {
        // A response is already being delivered. Keep the future so the next
        // timer iteration can consume it instead of issuing a duplicate.
        return;
      }
      pending_key_frame_request_.reset();
      RCLCPP_WARN(
          this->get_logger(),
          "Keyframe %lu request timed out after %.1f s; retrying",
          timed_out_index,
          std::chrono::duration<double>(request_timeout).count());
    }
    else
    {
      const std::size_t key_frame_index = next_key_frame_index_.load();
      try
      {
        const auto result = pending_key_frame_request_->get();
        pending_key_frame_request_.reset();

        pcl::PointCloud<pcl_t> pcl_cloud;
        pcl::PointCloud<pcl_t> pcl_surface_cloud;
        pcl::PointCloud<pcl_t> pcl_ground_cloud;
        pcl::PointCloud<pcl_t> pcl_cloud_base_link;
        pcl::PointCloud<pcl_t> pcl_surface_cloud_base_link;
        pcl::PointCloud<pcl_t> pcl_ground_cloud_base_link;

        pcl::fromROSMsg(result->key_frame_cloud, pcl_cloud);
        pcl::fromROSMsg(result->key_frame_surface, pcl_surface_cloud);
        pcl::fromROSMsg(result->key_frame_ground, pcl_ground_cloud);
        pcl::fromROSMsg(result->key_frame_cloud_base_link, pcl_cloud_base_link);
        pcl::fromROSMsg(
            result->key_frame_surface_base_link, pcl_surface_cloud_base_link);
        pcl::fromROSMsg(
            result->key_frame_ground_base_link, pcl_ground_cloud_base_link);

        const auto& pose = key_poses_.at(key_frame_index);
        const tf2::Transform key_frame_transform(
            tf2::Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w),
            tf2::Vector3(
                pose.position.x, pose.position.y, pose.position.z));
        const auto cloud_pair_is_valid =
            [this, key_frame_index, &key_frame_transform](
                const char* name,
                const pcl::PointCloud<pcl_t>& map_cloud,
                const pcl::PointCloud<pcl_t>& base_cloud)
            {
              if (map_cloud.size() != base_cloud.size())
              {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Invalid keyframe %lu %s pair: map=%lu base_link=%lu",
                    key_frame_index, name, map_cloud.size(), base_cloud.size());
                return false;
              }
              // Empty keyframes are a supported pose-graph placeholder. Both
              // representations must agree; non-empty neighboring frames still
              // provide the aggregate localization and local submap data.
              if (map_cloud.empty())
              {
                return true;
              }

              const std::array<std::size_t, 3> samples = {
                  0, map_cloud.size() / 2, map_cloud.size() - 1};
              for (const std::size_t sample : samples)
              {
                const auto& base_point = base_cloud.points[sample];
                const auto& map_point = map_cloud.points[sample];
                const tf2::Vector3 expected = key_frame_transform * tf2::Vector3(
                    base_point.x, base_point.y, base_point.z);
                const double dx = expected.x() - map_point.x;
                const double dy = expected.y() - map_point.y;
                const double dz = expected.z() - map_point.z;
                if (!std::isfinite(dx) || !std::isfinite(dy) ||
                    !std::isfinite(dz) || dx * dx + dy * dy + dz * dz > 1e-4)
                {
                  RCLCPP_ERROR(
                      this->get_logger(),
                      "Keyframe %lu %s does not match its immutable pose snapshot",
                      key_frame_index, name);
                  return false;
                }
              }
              return true;
            };
        if (!cloud_pair_is_valid(
              "feature", pcl_cloud, pcl_cloud_base_link) ||
            !cloud_pair_is_valid(
              "surface", pcl_surface_cloud, pcl_surface_cloud_base_link) ||
            !cloud_pair_is_valid(
              "ground", pcl_ground_cloud, pcl_ground_cloud_base_link))
        {
          map_snapshot_invalidated_.store(true);
          RCLCPP_ERROR(
              this->get_logger(),
              "Keyframe synchronization invalidated relocalization; restart MCL "
              "after repairing or reloading the pose graph");
          return;
        }

        std::unique_lock<sub_maps_mutex_t> lock(*access_);
        if (key_frame_index != next_key_frame_index_.load())
        {
          RCLCPP_ERROR(
              this->get_logger(),
              "Discarding out-of-order keyframe %lu; expected %lu",
              key_frame_index, next_key_frame_index_.load());
          return;
        }
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *clock_, 1000,
            "Sync key frame number: %lu with total size: %lu",
            key_frame_index, poses_pcl_t_->size());
        cornerCloudKeyFrames_.push_back(pcl_cloud.makeShared());
        surfCloudKeyFrames_.push_back(pcl_surface_cloud.makeShared());
        groundCloudKeyFrames_.push_back(pcl_ground_cloud.makeShared());
        cornerCloudKeyFrames_baselink_.push_back(pcl_cloud_base_link.makeShared());
        surfCloudKeyFrames_baselink_.push_back(
            pcl_surface_cloud_base_link.makeShared());
        groundCloudKeyFrames_baselink_.push_back(
            pcl_ground_cloud_base_link.makeShared());
        next_key_frame_index_.store(key_frame_index + 1);
        return;
      }
      catch (const std::exception& error)
      {
        pending_key_frame_request_.reset();
        RCLCPP_ERROR(
            this->get_logger(), "Keyframe service call failed: %s; retrying",
            error.what());
      }
    }
  }

  const std::size_t key_frame_index = next_key_frame_index_.load();
  if (key_frame_index >= poses_pcl_t_->size())
  {
    std::unique_lock<sub_maps_mutex_t> lock(*access_);
    finishKeyFrameSyncLocked();
    if (!key_frames_ready_.load())
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Keyframe synchronization failed; global localization remains blocked");
      sync_map_timer_->cancel();
      return;
    }
    RCLCPP_WARN(this->get_logger(), "All keyframes synchronized; stopping sync timer.");
    RCLCPP_INFO(this->get_logger(), "Sync cloud key frame number: %lu with total size: %lu", cornerCloudKeyFrames_.size(), poses_pcl_t_->size());
    RCLCPP_INFO(this->get_logger(), "Sync surface key frame number: %lu with total size: %lu", surfCloudKeyFrames_.size(), poses_pcl_t_->size());
    RCLCPP_INFO(this->get_logger(), "Sync ground key frame number: %lu with total size: %lu", groundCloudKeyFrames_.size(), poses_pcl_t_->size());
    sync_map_timer_->cancel();
    return;
  }

  if (!get_key_frame_cloud_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_WARN(
        this->get_logger(), "Service %s/get_key_frame_cloud not available",
        pg_map_server_name_.c_str());
    return;
  }

  auto request =
      std::make_shared<dddmr_sys_core::srv::GetKeyFrameCloud::Request>();
  request->key_frame_number = key_frame_index;
  pending_key_frame_request_.emplace(
      get_key_frame_cloud_client_->async_send_request(request));
  key_frame_request_started_ = std::chrono::steady_clock::now();
}

void SubMaps::warmUpThread(){
  
  if(!is_initial_.load() || map_snapshot_invalidated_.load())
    return;
  
  if(!is_current_ready_){
    mcl_3dl::pcl_t target_pose;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    target_pose.x = robot_pose_.pose.pose.position.x;
    target_pose.y = robot_pose_.pose.pose.position.y;
    target_pose.z = robot_pose_.pose.pose.position.z;
    if(kdtree_poses_->radiusSearch(target_pose, sub_map_search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance,0)<1)
    {
      is_current_ready_ = false;
      return;
    }
    map_current_->clear();
    ground_current_->clear();
    for(auto it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); it++){
      *map_current_ += (*cornerCloudKeyFrames_[*it]);
      //*map_current_ += (*surfCloudKeyFrames_[*it]);
      *ground_current_ += (*groundCloudKeyFrames_[*it]);
    }

    kdtree_ground_current_.setInputCloud(ground_current_);
    kdtree_map_current_.setInputCloud(map_current_);   
    //@Normal estimation for ground
    pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n;
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    tree->setInputCloud (ground_current_);
    n.setInputCloud (ground_current_);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (normals_ground_current_);

    sensor_msgs::msg::PointCloud2 map_pc;
    pcl::toROSMsg(*map_current_, map_pc);
    map_pc.header.frame_id = "map";
    pub_sub_map_->publish(map_pc);

    sensor_msgs::msg::PointCloud2 ground_pc;
    pcl::toROSMsg(*ground_current_, ground_pc);
    ground_pc.header.frame_id = "map";
    pub_sub_ground_->publish(ground_pc);
    
    current_sub_map_pose_ = robot_pose_;
    RCLCPP_INFO(this->get_logger(), "All essential KD-tree are generated.");
    
    is_current_ready_ = true;
  }
  
  else if(prepare_warm_up_ && !is_warm_up_ready_){
    mcl_3dl::pcl_t target_pose;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    target_pose.x = warm_up_pose_.pose.pose.position.x;
    target_pose.y = warm_up_pose_.pose.pose.position.y;
    target_pose.z = warm_up_pose_.pose.pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Prepare warm up at: %.2f, %.2f, %.2f", target_pose.x, target_pose.y, target_pose.z);
    if(kdtree_poses_->radiusSearch(target_pose, sub_map_search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance,0)<1)
    {
      prepare_warm_up_ = false;
      return;
    }
    map_warmup_->clear();
    ground_warmup_->clear();
    for(auto it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); it++){
      *map_warmup_ += (*cornerCloudKeyFrames_[*it]);
      //*map_warmup_ += (*surfCloudKeyFrames_[*it]);
      *ground_warmup_ += (*groundCloudKeyFrames_[*it]);
    }
    kdtree_ground_warmup_.setInputCloud(ground_warmup_);
    kdtree_map_warmup_.setInputCloud(map_warmup_);   
    //@Normal estimation for ground
    pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n;
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    tree->setInputCloud (ground_warmup_);
    n.setInputCloud (ground_warmup_);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (normals_ground_warmup_);
    sensor_msgs::msg::PointCloud2 map_pc;
    pcl::toROSMsg(*map_warmup_, map_pc);
    map_pc.header.frame_id = "map";
    pub_sub_map_warmup_->publish(map_pc);

    sensor_msgs::msg::PointCloud2 ground_pc;
    pcl::toROSMsg(*ground_warmup_, ground_pc);
    ground_pc.header.frame_id = "map";
    pub_sub_ground_warmup_->publish(ground_pc);
    
    RCLCPP_INFO(this->get_logger(), "All essential Warmup KD-tree are generated.");
    prepare_warm_up_ = false;
    is_warm_up_ready_ = true;
  }
}

bool SubMaps::isWarmUpReady(){
  return is_warm_up_ready_ && !map_snapshot_invalidated_.load();
}

void SubMaps::swapKdTree(){
  
  map_current_->clear();
  ground_current_->clear();
  *map_current_ = *map_warmup_;
  *ground_current_ = *ground_warmup_;
  kdtree_map_current_ = kdtree_map_warmup_;
  kdtree_ground_current_ = kdtree_ground_warmup_;
  normals_ground_current_ = normals_ground_warmup_;
  current_sub_map_pose_ = warm_up_pose_;

  sensor_msgs::msg::PointCloud2 map_pc;
  pcl::toROSMsg(*map_warmup_, map_pc);
  map_pc.header.frame_id = "map";
  pub_sub_map_->publish(map_pc);

  sensor_msgs::msg::PointCloud2 ground_pc;
  pcl::toROSMsg(*ground_warmup_, ground_pc);
  ground_pc.header.frame_id = "map";
  pub_sub_ground_->publish(ground_pc);

  is_warm_up_ready_ = false;
  RCLCPP_INFO(this->get_logger(), "KD-tree swapped.");
}

void SubMaps::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose){
  RCLCPP_INFO(this->get_logger(), "Receive initial pose at: %.2f, %.2f, %.2f", pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  prepare_warm_up_ = true;
  warm_up_pose_ = pose;
}

void SubMaps::setPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose){
  robot_pose_ = pose;
  double dx = pose.pose.pose.position.x - current_sub_map_pose_.pose.pose.position.x;
  double dy = pose.pose.pose.position.y - current_sub_map_pose_.pose.pose.position.y;
  double dz = pose.pose.pose.position.z - current_sub_map_pose_.pose.pose.position.z;
  if(!is_warm_up_ready_ && sqrt(dx*dx + dy*dy + dz*dz)>=sub_map_warmup_trigger_distance_){
    prepare_warm_up_ = true;
    warm_up_pose_ = pose;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Warming up sub maps");
  }
}

}
