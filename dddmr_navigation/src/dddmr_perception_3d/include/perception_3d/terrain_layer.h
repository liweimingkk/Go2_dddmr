#ifndef PERCEPTION_3D__TERRAIN_LAYER_H_
#define PERCEPTION_3D__TERRAIN_LAYER_H_

#include "perception_3d/sensor.h"
#include "perception_3d/terrain_model_builder.h"

#include "dddmr_sys_core/msg/terrain_status.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace perception_3d
{

class TerrainLayer : public Sensor
{
public:
  TerrainLayer();
  ~TerrainLayer() override = default;

  void onInitialize() override;
  void selfClear() override;
  void selfMark() override;
  void updateLethalPointCloud() override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation() override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal() override;
  void resetdGraph() override;
  double get_dGraphValue(unsigned int index) override;
  bool isCurrent() override;

private:
  void terrainGroundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void liveGroundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void liveObstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void mapIdentityCallback(const std_msgs::msg::String::SharedPtr msg);
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    pcl::PointCloud<pcl::PointXYZI> * output) const;
  bool rebuildSnapshot();
  TerrainSnapshotConstPtr annotateStaircases(
    const TerrainSnapshotConstPtr & base_snapshot,
    const std::vector<Eigen::Vector3f> & ground_points,
    const pcl::PointCloud<pcl::PointXYZI> & live_ground,
    bool live_ground_available,
    const pcl::PointCloud<pcl::PointXYZI> & live_obstacles,
    bool live_obstacle_available) const;
  void publishDebugClouds(const TerrainSnapshotConstPtr & snapshot);
  void publishStairMarkers(const TerrainSnapshotConstPtr & snapshot);
  void publishRobotTerrainStatus(const TerrainSnapshotConstPtr & snapshot);
  bool liveGroundIsFresh() const;
  bool liveObstacleIsFresh() const;
  bool mapIdentityMatches() const;

  bool enabled_{false};
  bool is_local_planner_{false};
  bool publish_status_{false};
  bool require_live_ground_{false};
  bool require_live_obstacle_for_stairs_{true};
  bool publish_debug_clouds_{true};
  std::string map_hash_;
  std::string map_identity_topic_;
  std::string terrain_ground_topic_;
  std::string live_ground_topic_;
  std::string live_obstacle_topic_;
  double max_age_sec_{0.20};
  double live_support_radius_m_{0.12};
  double live_support_z_tolerance_m_{0.08};
  double status_search_radius_m_{0.40};
  double entry_distance_m_{0.35};
  double stair_height_tolerance_m_{0.06};
  double status_min_confidence_{0.90};
  double status_min_support_ratio_{0.80};
  double body_half_length_m_{0.42};
  double body_half_width_m_{0.30};
  double robot_ground_z_offset_m_{0.24};
  double rebuild_period_sec_{0.10};
  TerrainModelBuilderConfig builder_config_;
  std::vector<TerrainNode> static_base_nodes_;
  TerrainModelBuildStatistics static_model_statistics_;

  std::vector<StaircaseModel> configured_staircases_;
  std::uint64_t snapshot_version_{0U};
  std::size_t built_ground_size_{0U};
  std::uint64_t built_static_ground_generation_{0U};
  std::uint64_t terrain_cloud_generation_{0U};
  std::uint64_t built_terrain_cloud_generation_{0U};
  std::uint64_t live_cloud_generation_{0U};
  std::uint64_t built_live_cloud_generation_{0U};
  std::uint64_t live_obstacle_generation_{0U};
  std::uint64_t built_live_obstacle_generation_{0U};
  std::atomic<bool> built_with_fresh_live_ground_{false};
  std::atomic<bool> built_with_fresh_live_obstacle_{false};
  std::atomic<std::int64_t> built_live_ground_stamp_nanoseconds_{0};
  std::atomic<std::int64_t> built_live_obstacle_stamp_nanoseconds_{0};
  rclcpp::Time last_build_time_{0, 0, RCL_ROS_TIME};

  mutable std::mutex cloud_mutex_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_ground_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr live_ground_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr live_ground_kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr live_obstacle_cloud_;
  rclcpp::Time last_live_ground_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_live_obstacle_time_{0, 0, RCL_ROS_TIME};
  bool has_live_ground_{false};
  bool has_live_obstacle_{false};
  bool has_map_identity_{false};
  bool map_identity_matches_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr live_ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr live_obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_identity_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversability_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr drop_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stair_marker_pub_;
  rclcpp::Publisher<dddmr_sys_core::msg::TerrainStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rejection_reason_pub_;
};

}  // namespace perception_3d

#endif  // PERCEPTION_3D__TERRAIN_LAYER_H_
