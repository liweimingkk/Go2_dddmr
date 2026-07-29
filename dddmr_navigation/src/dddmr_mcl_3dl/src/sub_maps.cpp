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

#include <cmath>
#include <limits>

namespace mcl_3dl
{
SubMaps::SubMaps(std::string name)
: Node(name),
  is_initial_(false),
  is_current_ready_(false),
  key_poses_received_(false),
  key_frames_ready_(false),
  key_frame_request_in_flight_(false),
  key_frame_sync_failed_(false),
  next_key_frame_index_(0),
  pending_key_frame_index_(std::numeric_limits<std::size_t>::max()),
  pending_key_frame_generation_(0),
  map_generation_(0),
  expected_key_frame_count_(0),
  global_map_received_(false),
  global_ground_received_(false),
  is_global_ready_(false),
  map_snapshot_invalidated_(false),
  prepare_warm_up_(false),
  is_warm_up_ready_(false),
  warmup_request_generation_(0),
  warmup_ready_generation_(0),
  current_sub_map_generation_(0)
{
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
  RCLCPP_INFO(
      this->get_logger(), "pg_map_server_name: %s",
      pg_map_server_name_.c_str());

  declare_parameter("sub_map_search_radius", rclcpp::ParameterValue(50.0));
  this->get_parameter("sub_map_search_radius", sub_map_search_radius_);
  RCLCPP_INFO(
      this->get_logger(), "sub_map_search_radius: %.1f",
      sub_map_search_radius_);

  declare_parameter(
      "sub_map_warmup_trigger_distance", rclcpp::ParameterValue(20.0));
  this->get_parameter(
      "sub_map_warmup_trigger_distance",
      sub_map_warmup_trigger_distance_);
  RCLCPP_INFO(
      this->get_logger(), "sub_map_warmup_trigger_distance: %.1f",
      sub_map_warmup_trigger_distance_);

  declare_parameter("expected_key_frame_count", rclcpp::ParameterValue(0));
  int expected_key_frame_count = 0;
  this->get_parameter("expected_key_frame_count", expected_key_frame_count);
  expected_key_frame_count_ = static_cast<std::size_t>(
      std::max(0, expected_key_frame_count));
  RCLCPP_INFO(
      this->get_logger(),
      "expected_key_frame_count: %zu (zero accepts the pose snapshot count)",
      expected_key_frame_count_);

  srv_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  get_key_frame_cloud_client_ =
      this->create_client<dddmr_sys_core::srv::GetKeyFrameCloud>(
      pg_map_server_name_ + "/get_key_frame_cloud",
      rmw_qos_profile_services_default, srv_group_);

  sub_key_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      pg_map_server_name_ + "/key_poses",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&SubMaps::keyPosesCb, this, std::placeholders::_1));
  sub_global_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pg_map_server_name_ + "/mapcloud",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&SubMaps::globalMapCb, this, std::placeholders::_1));
  sub_global_ground_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pg_map_server_name_ + "/mapground",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&SubMaps::globalGroundCb, this, std::placeholders::_1));

  pub_sub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "sub_mapcloud",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_sub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "sub_mapground",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_sub_map_warmup_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "sub_mapcloud_warmup",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_sub_ground_warmup_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "sub_mapground_warmup",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  timer_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sync_map_timer_ = this->create_wall_timer(
      10ms, std::bind(&SubMaps::syncMapThread, this), timer_group_);
  warm_up_timer_ = this->create_wall_timer(
      200ms, std::bind(&SubMaps::warmUpThread, this), timer_group_);
}

SubMaps::~SubMaps()
{
  delete access_;
}

void SubMaps::keyPosesCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (key_poses_received_.load())
  {
    map_snapshot_invalidated_.store(true);
    key_frame_sync_failed_.store(true);
    RCLCPP_ERROR(
        this->get_logger(),
        "Replacement key poses rejected: the localization map snapshot is "
        "immutable; restart MCL to load a new generation");
    return;
  }
  if (msg->poses.empty())
  {
    key_frame_sync_failed_.store(true);
    RCLCPP_ERROR(this->get_logger(), "Received an empty key-pose array");
    return;
  }
  if (expected_key_frame_count_ != 0 &&
      msg->poses.size() != expected_key_frame_count_)
  {
    key_frame_sync_failed_.store(true);
    RCLCPP_ERROR(
        this->get_logger(),
        "Rejected key-pose snapshot: expected exactly %zu poses, received %zu",
        expected_key_frame_count_, msg->poses.size());
    sync_map_timer_->cancel();
    return;
  }

  auto poses_cloud = std::make_shared<pcl::PointCloud<pcl_t>>();
  poses_cloud->reserve(msg->poses.size());
  for (std::size_t index = 0; index < msg->poses.size(); ++index)
  {
    const auto& pose = msg->poses[index];
    const double quaternion_norm =
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w;
    if (!std::isfinite(pose.position.x) ||
        !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z) ||
        !std::isfinite(quaternion_norm) ||
        std::abs(quaternion_norm - 1.0) > 0.1)
    {
      key_frame_sync_failed_.store(true);
      RCLCPP_ERROR(
          this->get_logger(),
          "Rejected key-pose snapshot: pose %zu is non-finite or has an "
          "invalid quaternion", index);
      sync_map_timer_->cancel();
      return;
    }
    pcl_t point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    poses_cloud->push_back(point);
  }

  ++map_generation_;
  key_poses_ = msg->poses;
  poses_pcl_t_ = poses_cloud;
  kdtree_poses_ = std::make_shared<pcl::KdTreeFLANN<pcl_t>>();
  kdtree_poses_->setInputCloud(poses_pcl_t_);

  const std::size_t count = key_poses_.size();
  cornerCloudKeyFrames_.assign(count, nullptr);
  surfCloudKeyFrames_.assign(count, nullptr);
  groundCloudKeyFrames_.assign(count, nullptr);
  cornerCloudKeyFrames_baselink_.assign(count, nullptr);
  surfCloudKeyFrames_baselink_.assign(count, nullptr);
  groundCloudKeyFrames_baselink_.assign(count, nullptr);
  next_key_frame_index_ = 0;
  pending_key_frame_index_ = std::numeric_limits<std::size_t>::max();
  pending_key_frame_generation_ = map_generation_;
  key_frame_request_in_flight_.store(false);
  key_frames_ready_.store(false);
  is_initial_.store(false);
  key_poses_received_.store(true);
  sync_map_timer_->reset();
  RCLCPP_INFO(
      this->get_logger(),
      "Accepted immutable key-pose generation %llu with %zu frames",
      static_cast<unsigned long long>(map_generation_), count);
}

void SubMaps::globalMapCb(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (global_map_received_)
  {
    map_snapshot_invalidated_.store(true);
    RCLCPP_ERROR(
        this->get_logger(),
        "Replacement complete map rejected: restart MCL to load a new map "
        "generation");
    return;
  }
  pcl::fromROSMsg(*msg, *map_global_);
  global_map_received_ = !map_global_->empty();
  if (!global_map_received_)
  {
    RCLCPP_ERROR(
        this->get_logger(), "Received an empty complete map from %s/mapcloud",
        pg_map_server_name_.c_str());
  }
  prepareGlobalMapLocked();
}

void SubMaps::globalGroundCb(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (global_ground_received_)
  {
    map_snapshot_invalidated_.store(true);
    RCLCPP_ERROR(
        this->get_logger(),
        "Replacement complete ground rejected: restart MCL to load a new map "
        "generation");
    return;
  }
  pcl::fromROSMsg(*msg, *ground_global_);
  global_ground_received_ = !ground_global_->empty();
  if (!global_ground_received_)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Received an empty complete ground map from %s/mapground",
        pg_map_server_name_.c_str());
  }
  prepareGlobalMapLocked();
}

bool SubMaps::buildSearchData(
    const pcl::PointCloud<pcl_t>::Ptr& map,
    const pcl::PointCloud<pcl_t>::Ptr& ground,
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& map_tree,
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& ground_tree,
    pcl::PointCloud<pcl::Normal>& ground_normals)
{
  if (!map || !ground || map->empty() || ground->empty())
  {
    return false;
  }

  map_tree.setInputCloud(map);
  ground_tree.setInputCloud(ground);
  pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> normal_estimator;
  auto tree = std::make_shared<pcl::search::KdTree<mcl_3dl::pcl_t>>();
  tree->setInputCloud(ground);
  normal_estimator.setInputCloud(ground);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setKSearch(
      std::min(20, static_cast<int>(ground->size())));
  normal_estimator.compute(ground_normals);
  return ground_normals.size() == ground->size();
}

void SubMaps::prepareGlobalMapLocked()
{
  if (is_global_ready_.load() || map_snapshot_invalidated_.load() ||
      !key_frames_ready_.load() || !global_map_received_ ||
      !global_ground_received_)
  {
    return;
  }
  if (!buildSearchData(
        map_global_, ground_global_, kdtree_map_global_,
        kdtree_ground_global_, normals_ground_global_))
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Complete map cannot become ready: map or ground search data is empty");
    return;
  }

  is_global_ready_.store(true);
  RCLCPP_INFO(
      this->get_logger(),
      "Global localization map ready after %zu/%zu validated keyframes: "
      "%zu map points, %zu ground points",
      next_key_frame_index_, key_poses_.size(), map_global_->size(),
      ground_global_->size());
}

std::vector<geometry_msgs::msg::Pose> SubMaps::getKeyPoses()
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (!areKeyFramesReady())
  {
    return {};
  }
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
  if (key_frame_sync_failed_.load() || next_key_frame_index_ != expected)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Keyframe synchronization incomplete: received %zu/%zu",
        next_key_frame_index_, expected);
    return;
  }

  for (std::size_t index = 0; index < expected; ++index)
  {
    if (!cornerCloudKeyFrames_[index] ||
        !surfCloudKeyFrames_[index] ||
        !groundCloudKeyFrames_[index] ||
        !cornerCloudKeyFrames_baselink_[index] ||
        !surfCloudKeyFrames_baselink_[index] ||
        !groundCloudKeyFrames_baselink_[index])
    {
      key_frame_sync_failed_.store(true);
      RCLCPP_ERROR(
          this->get_logger(),
          "Keyframe synchronization has an unfilled slot at index %zu",
          index);
      return;
    }
  }

  feature_global_->clear();
  kdtree_surface_key_frames_.clear();
  kdtree_surface_key_frames_.reserve(expected);
  std::size_t surface_points = 0;
  for (std::size_t index = 0; index < expected; ++index)
  {
    *feature_global_ += *cornerCloudKeyFrames_[index];
    auto surface_tree = std::make_shared<pcl::KdTreeFLANN<pcl_t>>();
    if (!surfCloudKeyFrames_[index]->empty())
    {
      surface_tree->setInputCloud(surfCloudKeyFrames_[index]);
      surface_points += surfCloudKeyFrames_[index]->size();
    }
    kdtree_surface_key_frames_.push_back(surface_tree);
  }
  if (feature_global_->empty() || surface_points == 0)
  {
    key_frame_sync_failed_.store(true);
    RCLCPP_ERROR(
        this->get_logger(),
        "Validated keyframe slots contain no usable feature/surface data: "
        "feature=%zu surface=%zu",
        feature_global_->size(), surface_points);
    return;
  }
  kdtree_feature_global_.setInputCloud(feature_global_);
  key_frames_ready_.store(true);
  is_initial_.store(true);
  prepareGlobalMapLocked();
  RCLCPP_INFO(
      this->get_logger(),
      "Validated exactly %zu/%zu ordered keyframes; submap construction is "
      "now enabled", expected, expected);
}

void SubMaps::syncMapThread()
{
  if (!key_poses_received_.load() || key_frames_ready_.load() ||
      key_frame_sync_failed_.load() || map_snapshot_invalidated_.load() ||
      key_frame_request_in_flight_.load())
  {
    return;
  }
  if (!get_key_frame_cloud_client_->wait_for_service(0s))
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Service %s/get_key_frame_cloud not available",
        pg_map_server_name_.c_str());
    return;
  }

  std::size_t requested_index = 0;
  std::uint64_t requested_generation = 0;
  {
    std::unique_lock<sub_maps_mutex_t> lock(*access_);
    if (key_frame_request_in_flight_.load() || key_frames_ready_.load() ||
        key_frame_sync_failed_.load() || map_snapshot_invalidated_.load())
    {
      return;
    }
    if (next_key_frame_index_ >= key_poses_.size())
    {
      finishKeyFrameSyncLocked();
      sync_map_timer_->cancel();
      return;
    }
    requested_index = next_key_frame_index_;
    requested_generation = map_generation_;
    pending_key_frame_index_ = requested_index;
    pending_key_frame_generation_ = requested_generation;
    key_frame_request_in_flight_.store(true);
  }

  auto request =
      std::make_shared<dddmr_sys_core::srv::GetKeyFrameCloud::Request>();
  request->key_frame_number = static_cast<int32_t>(requested_index);
  get_key_frame_cloud_client_->async_send_request(
      request,
      [this, requested_index, requested_generation](
          rclcpp::Client<
            dddmr_sys_core::srv::GetKeyFrameCloud>::SharedFuture future)
      {
        pcl::PointCloud<pcl_t> cloud;
        pcl::PointCloud<pcl_t> surface;
        pcl::PointCloud<pcl_t> ground;
        pcl::PointCloud<pcl_t> cloud_base;
        pcl::PointCloud<pcl_t> surface_base;
        pcl::PointCloud<pcl_t> ground_base;
        bool response_valid = true;
        try
        {
          const auto result = future.get();
          pcl::fromROSMsg(result->key_frame_cloud, cloud);
          pcl::fromROSMsg(result->key_frame_surface, surface);
          pcl::fromROSMsg(result->key_frame_ground, ground);
          pcl::fromROSMsg(result->key_frame_cloud_base_link, cloud_base);
          pcl::fromROSMsg(result->key_frame_surface_base_link, surface_base);
          pcl::fromROSMsg(result->key_frame_ground_base_link, ground_base);
        }
        catch (const std::exception& error)
        {
          std::unique_lock<sub_maps_mutex_t> lock(*access_);
          if (pending_key_frame_index_ == requested_index &&
              pending_key_frame_generation_ == requested_generation)
          {
            key_frame_request_in_flight_.store(false);
          }
          RCLCPP_ERROR(
              this->get_logger(),
              "Keyframe %zu service call failed: %s; single-flight retry "
              "remains at the same index",
              requested_index, error.what());
          return;
        }

        std::unique_lock<sub_maps_mutex_t> lock(*access_);
        if (!key_frame_request_in_flight_.load() ||
            pending_key_frame_index_ != requested_index ||
            pending_key_frame_generation_ != requested_generation)
        {
          RCLCPP_ERROR(
              this->get_logger(),
              "Rejected stale or duplicate keyframe response %zu",
              requested_index);
          return;
        }
        key_frame_request_in_flight_.store(false);

        if (map_snapshot_invalidated_.load() ||
            requested_generation != map_generation_)
        {
          RCLCPP_ERROR(
              this->get_logger(),
              "Rejected keyframe %zu from obsolete map generation %llu",
              requested_index,
              static_cast<unsigned long long>(requested_generation));
          return;
        }
        if (requested_index >= key_poses_.size())
        {
          response_valid = false;
          RCLCPP_ERROR(
              this->get_logger(),
              "Rejected out-of-range keyframe response %zu (size %zu)",
              requested_index, key_poses_.size());
        }
        if (requested_index != next_key_frame_index_)
        {
          response_valid = false;
          RCLCPP_ERROR(
              this->get_logger(),
              "Rejected out-of-order keyframe response %zu; expected %zu",
              requested_index, next_key_frame_index_);
        }
        if (response_valid && cornerCloudKeyFrames_[requested_index])
        {
          response_valid = false;
          RCLCPP_ERROR(
              this->get_logger(),
              "Rejected duplicate keyframe response %zu",
              requested_index);
        }

        const auto pair_is_valid =
            [this, requested_index](
                const char* name,
                const pcl::PointCloud<pcl_t>& map_cloud,
                const pcl::PointCloud<pcl_t>& base_cloud,
                const geometry_msgs::msg::Pose& pose)
            {
              if (map_cloud.size() != base_cloud.size())
              {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Rejected keyframe %zu %s pair: map=%zu base_link=%zu",
                    requested_index, name, map_cloud.size(),
                    base_cloud.size());
                return false;
              }
              if (map_cloud.empty())
              {
                return true;
              }
              const tf2::Transform transform(
                  tf2::Quaternion(
                      pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w),
                  tf2::Vector3(
                      pose.position.x, pose.position.y, pose.position.z));
              for (std::size_t point_index = 0;
                   point_index < map_cloud.size(); ++point_index)
              {
                const auto& base_point = base_cloud.points[point_index];
                const auto& map_point = map_cloud.points[point_index];
                const tf2::Vector3 expected =
                    transform * tf2::Vector3(
                    base_point.x, base_point.y, base_point.z);
                const double dx = expected.x() - map_point.x;
                const double dy = expected.y() - map_point.y;
                const double dz = expected.z() - map_point.z;
                if (!std::isfinite(dx) || !std::isfinite(dy) ||
                    !std::isfinite(dz) ||
                    dx * dx + dy * dy + dz * dz > 1e-4)
                {
                  RCLCPP_ERROR(
                      this->get_logger(),
                      "Rejected keyframe %zu %s point %zu: map/base_link data "
                      "does not match the immutable pose snapshot",
                      requested_index, name, point_index);
                  return false;
                }
              }
              return true;
            };

        if (response_valid)
        {
          const auto& pose = key_poses_[requested_index];
          response_valid =
              pair_is_valid("feature", cloud, cloud_base, pose) &&
              pair_is_valid("surface", surface, surface_base, pose) &&
              pair_is_valid("ground", ground, ground_base, pose);
        }
        if (!response_valid)
        {
          key_frame_sync_failed_.store(true);
          sync_map_timer_->cancel();
          RCLCPP_ERROR(
              this->get_logger(),
              "Keyframe synchronization failed closed at %zu/%zu; no submap "
              "or global localization tree will be built",
              requested_index, key_poses_.size());
          return;
        }

        cornerCloudKeyFrames_[requested_index] = cloud.makeShared();
        surfCloudKeyFrames_[requested_index] = surface.makeShared();
        groundCloudKeyFrames_[requested_index] = ground.makeShared();
        cornerCloudKeyFrames_baselink_[requested_index] =
            cloud_base.makeShared();
        surfCloudKeyFrames_baselink_[requested_index] =
            surface_base.makeShared();
        groundCloudKeyFrames_baselink_[requested_index] =
            ground_base.makeShared();
        ++next_key_frame_index_;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *clock_, 1000,
            "Validated ordered keyframe %zu/%zu",
            next_key_frame_index_, key_poses_.size());
        if (next_key_frame_index_ == key_poses_.size())
        {
          finishKeyFrameSyncLocked();
          sync_map_timer_->cancel();
        }
      });
}

bool SubMaps::buildSubMap(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose,
    pcl::PointCloud<pcl_t>::Ptr& map,
    pcl::PointCloud<pcl_t>::Ptr& ground)
{
  if (!areKeyFramesReady() || !kdtree_poses_)
  {
    return false;
  }
  pcl_t target;
  target.x = pose.pose.pose.position.x;
  target.y = pose.pose.pose.position.y;
  target.z = pose.pose.pose.position.z;
  std::vector<int> indices;
  std::vector<float> squared_distances;
  if (kdtree_poses_->radiusSearch(
        target, sub_map_search_radius_, indices, squared_distances, 0) < 1)
  {
    return false;
  }

  map = std::make_shared<pcl::PointCloud<pcl_t>>();
  ground = std::make_shared<pcl::PointCloud<pcl_t>>();
  for (const int index : indices)
  {
    if (index < 0 ||
        static_cast<std::size_t>(index) >= cornerCloudKeyFrames_.size() ||
        !cornerCloudKeyFrames_[static_cast<std::size_t>(index)] ||
        !groundCloudKeyFrames_[static_cast<std::size_t>(index)])
    {
      return false;
    }
    *map += *cornerCloudKeyFrames_[static_cast<std::size_t>(index)];
    *ground += *groundCloudKeyFrames_[static_cast<std::size_t>(index)];
  }
  return !map->empty() && !ground->empty();
}

void SubMaps::warmUpThread()
{
  if (!is_initial_.load() || map_snapshot_invalidated_.load())
  {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped target_pose;
  std::uint64_t request_generation = 0;
  std::uint64_t map_generation = 0;
  bool building_current = false;
  {
    std::unique_lock<sub_maps_mutex_t> lock(*access_);
    if (!is_current_ready_.load())
    {
      building_current = true;
      target_pose = robot_pose_;
      request_generation = warmup_request_generation_;
      map_generation = map_generation_;
    }
    else if (prepare_warm_up_ && !is_warm_up_ready_.load())
    {
      target_pose = warm_up_pose_;
      request_generation = warmup_request_generation_;
      map_generation = map_generation_;
    }
    else
    {
      return;
    }
  }

  pcl::PointCloud<pcl_t>::Ptr built_map;
  pcl::PointCloud<pcl_t>::Ptr built_ground;
  if (!buildSubMap(target_pose, built_map, built_ground))
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Submap generation %llu found no validated keyframes near "
        "(%.2f, %.2f, %.2f)",
        static_cast<unsigned long long>(request_generation),
        target_pose.pose.pose.position.x,
        target_pose.pose.pose.position.y,
        target_pose.pose.pose.position.z);
    return;
  }

  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (map_snapshot_invalidated_.load() ||
      map_generation != map_generation_ ||
      request_generation != warmup_request_generation_)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Discarded stale submap warmup generation %llu (current %llu)",
        static_cast<unsigned long long>(request_generation),
        static_cast<unsigned long long>(warmup_request_generation_));
    return;
  }

  if (building_current)
  {
    map_current_ = built_map;
    ground_current_ = built_ground;
    if (!buildSearchData(
          map_current_, ground_current_, kdtree_map_current_,
          kdtree_ground_current_, normals_ground_current_))
    {
      is_current_ready_.store(false);
      return;
    }
    current_sub_map_pose_ = target_pose;
    current_sub_map_generation_ = request_generation;
    is_current_ready_.store(true);

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_current_, map_msg);
    map_msg.header.frame_id = "map";
    pub_sub_map_->publish(map_msg);
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_current_, ground_msg);
    ground_msg.header.frame_id = "map";
    pub_sub_ground_->publish(ground_msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Current submap generation %llu ready: map=%zu ground=%zu",
        static_cast<unsigned long long>(current_sub_map_generation_),
        map_current_->size(), ground_current_->size());
    return;
  }

  map_warmup_ = built_map;
  ground_warmup_ = built_ground;
  if (!buildSearchData(
        map_warmup_, ground_warmup_, kdtree_map_warmup_,
        kdtree_ground_warmup_, normals_ground_warmup_))
  {
    prepare_warm_up_ = false;
    is_warm_up_ready_.store(false);
    return;
  }
  warmup_ready_generation_ = request_generation;
  prepare_warm_up_ = false;
  is_warm_up_ready_.store(true);

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_warmup_, map_msg);
  map_msg.header.frame_id = "map";
  pub_sub_map_warmup_->publish(map_msg);
  sensor_msgs::msg::PointCloud2 ground_msg;
  pcl::toROSMsg(*ground_warmup_, ground_msg);
  ground_msg.header.frame_id = "map";
  pub_sub_ground_warmup_->publish(ground_msg);
  RCLCPP_INFO(
      this->get_logger(),
      "Warmup submap generation %llu ready: map=%zu ground=%zu",
      static_cast<unsigned long long>(warmup_ready_generation_),
      map_warmup_->size(), ground_warmup_->size());
}

bool SubMaps::isWarmUpReady()
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  return is_warm_up_ready_.load() &&
      warmup_ready_generation_ == warmup_request_generation_ &&
      !map_snapshot_invalidated_.load();
}

void SubMaps::swapKdTree()
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  if (!is_warm_up_ready_.load() ||
      warmup_ready_generation_ != warmup_request_generation_ ||
      map_snapshot_invalidated_.load())
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Rejected stale submap switch: ready=%llu requested=%llu",
        static_cast<unsigned long long>(warmup_ready_generation_),
        static_cast<unsigned long long>(warmup_request_generation_));
    return;
  }

  map_current_ = std::make_shared<pcl::PointCloud<pcl_t>>(*map_warmup_);
  ground_current_ =
      std::make_shared<pcl::PointCloud<pcl_t>>(*ground_warmup_);
  // Rebuild from the copied current clouds. PCL KD-tree objects retain input
  // cloud pointers internally and must never be copied from the warmup trees.
  if (!buildSearchData(
        map_current_, ground_current_, kdtree_map_current_,
        kdtree_ground_current_, normals_ground_current_))
  {
    is_current_ready_.store(false);
    is_warm_up_ready_.store(false);
    RCLCPP_ERROR(
        this->get_logger(),
        "Submap generation %llu failed while rebuilding current KD-trees",
        static_cast<unsigned long long>(warmup_ready_generation_));
    return;
  }
  current_sub_map_pose_ = warm_up_pose_;
  current_sub_map_generation_ = warmup_ready_generation_;
  is_current_ready_.store(true);
  is_warm_up_ready_.store(false);

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_current_, map_msg);
  map_msg.header.frame_id = "map";
  pub_sub_map_->publish(map_msg);
  sensor_msgs::msg::PointCloud2 ground_msg;
  pcl::toROSMsg(*ground_current_, ground_msg);
  ground_msg.header.frame_id = "map";
  pub_sub_ground_->publish(ground_msg);
  RCLCPP_INFO(
      this->get_logger(),
      "Switched to submap generation %llu after rebuilding current KD-trees",
      static_cast<unsigned long long>(current_sub_map_generation_));
}

void SubMaps::setInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped pose)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  robot_pose_ = pose;
  warm_up_pose_ = pose;
  prepare_warm_up_ = true;
  is_warm_up_ready_.store(false);
  ++warmup_request_generation_;
  RCLCPP_INFO(
      this->get_logger(),
      "Scheduled submap generation %llu for initial pose "
      "(%.2f, %.2f, %.2f)",
      static_cast<unsigned long long>(warmup_request_generation_),
      pose.pose.pose.position.x, pose.pose.pose.position.y,
      pose.pose.pose.position.z);
}

void SubMaps::setPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped pose)
{
  std::unique_lock<sub_maps_mutex_t> lock(*access_);
  robot_pose_ = pose;
  const double dx =
      pose.pose.pose.position.x - current_sub_map_pose_.pose.pose.position.x;
  const double dy =
      pose.pose.pose.position.y - current_sub_map_pose_.pose.pose.position.y;
  const double dz =
      pose.pose.pose.position.z - current_sub_map_pose_.pose.pose.position.z;
  if (!prepare_warm_up_ && !is_warm_up_ready_.load() &&
      std::sqrt(dx * dx + dy * dy + dz * dz) >=
        sub_map_warmup_trigger_distance_)
  {
    prepare_warm_up_ = true;
    warm_up_pose_ = pose;
    ++warmup_request_generation_;
    RCLCPP_INFO(
        this->get_logger(),
        "Scheduled rolling submap generation %llu",
        static_cast<unsigned long long>(warmup_request_generation_));
  }
}

}  // namespace mcl_3dl
