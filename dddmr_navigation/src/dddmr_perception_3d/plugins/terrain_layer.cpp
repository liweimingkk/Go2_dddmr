/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 */

#include "perception_3d/terrain_layer.h"
#include "perception_3d/terrain_freshness.h"

#include "pluginlib/class_list_macros.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <map>
#include <stdexcept>
#include <utility>

PLUGINLIB_EXPORT_CLASS(perception_3d::TerrainLayer, perception_3d::Sensor)

namespace perception_3d
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr float kGeometryEpsilon = 1.0e-5F;

template<typename T>
T declareAndGet(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  node->declare_parameter<T>(name, default_value);
  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finiteRatio(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

std::string normalizeSha256(std::string value)
{
  if (value.size() != 64U || !std::all_of(
      value.begin(), value.end(), [](unsigned char character) {
        return std::isxdigit(character) != 0;
      }))
  {
    return {};
  }
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

bool pointOnSegment(
  const Eigen::Vector2f & point,
  const Eigen::Vector2f & first,
  const Eigen::Vector2f & second)
{
  const Eigen::Vector2f segment = second - first;
  const Eigen::Vector2f offset = point - first;
  const float cross = segment.x() * offset.y() - segment.y() * offset.x();
  if (std::abs(cross) > 1.0e-5F) {
    return false;
  }
  const float projection = offset.dot(segment);
  return projection >= -1.0e-5F &&
         projection <= segment.squaredNorm() + 1.0e-5F;
}

bool pointInPolygon(
  const Eigen::Vector2f & point,
  const std::vector<Eigen::Vector2f> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }
  bool inside = false;
  for (std::size_t index = 0U, previous = polygon.size() - 1U;
    index < polygon.size(); previous = index++)
  {
    const auto & first = polygon[previous];
    const auto & second = polygon[index];
    if (pointOnSegment(point, first, second)) {
      return true;
    }
    const bool crosses_y = (first.y() > point.y()) != (second.y() > point.y());
    if (crosses_y) {
      const float crossing_x =
        (second.x() - first.x()) * (point.y() - first.y()) /
        (second.y() - first.y()) + first.x();
      if (point.x() < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool footprintInsidePolygon(
  const Eigen::Vector2f & center,
  float yaw,
  float half_length,
  float half_width,
  const std::vector<Eigen::Vector2f> & polygon)
{
  const Eigen::Vector2f forward(std::cos(yaw), std::sin(yaw));
  const Eigen::Vector2f left(-forward.y(), forward.x());
  for (const float longitudinal : {-half_length, half_length}) {
    for (const float lateral : {-half_width, half_width}) {
      if (!pointInPolygon(center + longitudinal * forward + lateral * left, polygon)) {
        return false;
      }
    }
  }
  return true;
}

Eigen::Vector3f vector3FromParameter(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 3U ||
    !std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument(parameter_name + " must contain exactly three finite numbers");
  }
  return Eigen::Vector3f(
    static_cast<float>(values[0]),
    static_cast<float>(values[1]),
    static_cast<float>(values[2]));
}

std::vector<Eigen::Vector2f> polygonFromParameter(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() < 6U || values.size() % 2U != 0U) {
    throw std::invalid_argument(
            parameter_name + " must contain at least three flattened XY vertices");
  }
  std::vector<Eigen::Vector2f> polygon;
  polygon.reserve(values.size() / 2U);
  for (std::size_t index = 0U; index < values.size(); index += 2U) {
    if (!std::isfinite(values[index]) || !std::isfinite(values[index + 1U])) {
      throw std::invalid_argument(parameter_name + " contains a non-finite coordinate");
    }
    polygon.emplace_back(
      static_cast<float>(values[index]), static_cast<float>(values[index + 1U]));
  }
  return polygon;
}

std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA result;
  result.r = red;
  result.g = green;
  result.b = blue;
  result.a = alpha;
  return result;
}

visualization_msgs::msg::Marker polygonMarker(
  const std::vector<Eigen::Vector2f> & polygon,
  float height,
  const std::string & frame,
  const rclcpp::Time & stamp,
  const std::string & marker_namespace,
  int marker_id,
  const std_msgs::msg::ColorRGBA & marker_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.035;
  marker.color = marker_color;
  for (const auto & vertex : polygon) {
    geometry_msgs::msg::Point point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = height;
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }
  return marker;
}

std::uint8_t terrainClassMessageValue(TerrainClass terrain_class)
{
  return static_cast<std::uint8_t>(terrain_class);
}

}  // namespace

TerrainLayer::TerrainLayer()
{
  terrain_ground_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  live_ground_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  live_ground_kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  live_obstacle_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  sensor_current_observation_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  current_lethal_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
}

void TerrainLayer::onInitialize()
{
  const std::string prefix = name_ + ".";
  enabled_ = declareAndGet(node_, prefix + "enabled", false);
  is_local_planner_ = declareAndGet(node_, prefix + "is_local_planner", false);
  publish_status_ = declareAndGet(node_, prefix + "publish_status", false);
  require_live_ground_ = declareAndGet(node_, prefix + "require_live_ground", false);
  require_live_obstacle_for_stairs_ = declareAndGet(
    node_, prefix + "require_live_obstacle_for_stairs", true);
  publish_debug_clouds_ = declareAndGet(node_, prefix + "publish_debug_clouds", true);
  map_hash_ = normalizeSha256(
    declareAndGet(node_, prefix + "map_hash", std::string{}));
  map_identity_topic_ =
    declareAndGet(node_, prefix + "map_identity_topic", std::string{"map1/map_sha256"});
  terrain_ground_topic_ =
    declareAndGet(node_, prefix + "terrain_ground_topic", std::string{"map1/terrain_ground"});
  live_ground_topic_ =
    declareAndGet(node_, prefix + "live_ground_topic", std::string{"ground_cloud"});
  live_obstacle_topic_ = declareAndGet(
    node_, prefix + "live_obstacle_topic", std::string{"segmented_cloud_pure"});
  max_age_sec_ = declareAndGet(node_, prefix + "max_age_sec", 0.20);
  live_support_radius_m_ =
    declareAndGet(node_, prefix + "live_support_radius_m", 0.12);
  live_support_z_tolerance_m_ =
    declareAndGet(node_, prefix + "live_support_z_tolerance_m", 0.08);
  status_search_radius_m_ =
    declareAndGet(node_, prefix + "status_search_radius_m", 0.40);
  entry_distance_m_ = declareAndGet(node_, prefix + "entry_distance_m", 0.35);
  stair_height_tolerance_m_ =
    declareAndGet(node_, prefix + "stair_height_tolerance_m", 0.06);
  status_min_confidence_ =
    declareAndGet(node_, prefix + "status_min_confidence", 0.90);
  status_min_support_ratio_ =
    declareAndGet(node_, prefix + "status_min_support_ratio", 0.80);
  body_half_length_m_ = declareAndGet(node_, prefix + "body_half_length_m", 0.42);
  body_half_width_m_ = declareAndGet(node_, prefix + "body_half_width_m", 0.30);
  robot_ground_z_offset_m_ = declareAndGet(
    node_, prefix + "robot_ground_z_offset_m", 0.24);
  rebuild_period_sec_ = declareAndGet(node_, prefix + "rebuild_period_sec", 0.10);

  builder_config_.normal_radius_m = static_cast<float>(
    declareAndGet(node_, prefix + "model.normal_radius_m", 0.30));
  builder_config_.min_normal_neighbors = static_cast<std::size_t>(
    declareAndGet(node_, prefix + "model.min_normal_neighbors", 8));
  builder_config_.max_plane_residual_m = static_cast<float>(
    declareAndGet(node_, prefix + "model.max_plane_residual_m", 0.04));
  builder_config_.flat_slope_threshold_rad = static_cast<float>(
    declareAndGet(node_, prefix + "model.flat_slope_threshold_rad", 5.0 * kPi / 180.0));
  builder_config_.max_model_slope_rad = static_cast<float>(
    declareAndGet(node_, prefix + "model.max_model_slope_rad", 80.0 * kPi / 180.0));
  builder_config_.support_radius_m = static_cast<float>(
    declareAndGet(node_, prefix + "model.support_radius_m", 0.20));
  builder_config_.support_plane_tolerance_m = static_cast<float>(
    declareAndGet(node_, prefix + "model.support_plane_tolerance_m", 0.05));
  builder_config_.support_sector_count = static_cast<std::size_t>(
    declareAndGet(node_, prefix + "model.support_sector_count", 8));
  builder_config_.min_observed_support_ratio = static_cast<float>(
    declareAndGet(node_, prefix + "model.min_observed_support_ratio", 0.25));
  builder_config_.edge_support_ratio = static_cast<float>(
    declareAndGet(node_, prefix + "model.edge_support_ratio", 0.75));
  builder_config_.surface_connectivity_radius_m = static_cast<float>(
    declareAndGet(node_, prefix + "model.surface_connectivity_radius_m", 0.35));
  builder_config_.max_surface_height_delta_m = static_cast<float>(
    declareAndGet(node_, prefix + "model.max_surface_height_delta_m", 0.20));
  builder_config_.max_surface_normal_change_rad = static_cast<float>(
    declareAndGet(
      node_, prefix + "model.max_surface_normal_change_rad", 20.0 * kPi / 180.0));

  const auto stair_ids =
    declareAndGet(node_, prefix + "stair_ids", std::vector<std::int64_t>{});
  configured_staircases_.reserve(stair_ids.size());
  for (const auto stair_id_value : stair_ids) {
    if (stair_id_value < 0 || stair_id_value > 100000) {
      throw std::invalid_argument(prefix + "stair_ids contains an invalid id");
    }
    StaircaseModel staircase;
    staircase.id = static_cast<std::int32_t>(stair_id_value);
    const std::string stair_prefix =
      prefix + "stairs." + std::to_string(staircase.id) + ".";
    staircase.map_hash = normalizeSha256(
      declareAndGet(node_, stair_prefix + "map_hash", map_hash_));
    staircase.up_axis = vector3FromParameter(
      declareAndGet(node_, stair_prefix + "up_axis", std::vector<double>{}),
      stair_prefix + "up_axis");
    staircase.lower_landing_center = vector3FromParameter(
      declareAndGet(node_, stair_prefix + "lower_landing_center", std::vector<double>{}),
      stair_prefix + "lower_landing_center");
    staircase.upper_landing_center = vector3FromParameter(
      declareAndGet(node_, stair_prefix + "upper_landing_center", std::vector<double>{}),
      stair_prefix + "upper_landing_center");
    staircase.first_riser_center = vector3FromParameter(
      declareAndGet(node_, stair_prefix + "first_riser_center", std::vector<double>{}),
      stair_prefix + "first_riser_center");
    staircase.corridor_polygon_xy = polygonFromParameter(
      declareAndGet(node_, stair_prefix + "corridor_polygon_xy", std::vector<double>{}),
      stair_prefix + "corridor_polygon_xy");
    staircase.lower_landing_polygon_xy = polygonFromParameter(
      declareAndGet(node_, stair_prefix + "lower_landing_polygon_xy", std::vector<double>{}),
      stair_prefix + "lower_landing_polygon_xy");
    staircase.upper_landing_polygon_xy = polygonFromParameter(
      declareAndGet(node_, stair_prefix + "upper_landing_polygon_xy", std::vector<double>{}),
      stair_prefix + "upper_landing_polygon_xy");
    staircase.width_m = static_cast<float>(
      declareAndGet(node_, stair_prefix + "width_m", 0.0));
    staircase.riser_height_m = static_cast<float>(
      declareAndGet(node_, stair_prefix + "riser_height_m", 0.0));
    staircase.tread_depth_m = static_cast<float>(
      declareAndGet(node_, stair_prefix + "tread_depth_m", 0.0));
    staircase.step_count = static_cast<std::int32_t>(
      declareAndGet(node_, stair_prefix + "step_count", 0));
    staircase.confidence = static_cast<float>(
      declareAndGet(node_, stair_prefix + "confidence", 0.0));
    staircase.allow_up = declareAndGet(node_, stair_prefix + "allow_up", false);
    staircase.allow_down = declareAndGet(node_, stair_prefix + "allow_down", false);
    configured_staircases_.push_back(std::move(staircase));
  }

  if (enabled_) {
    if (map_hash_.empty()) {
      throw std::invalid_argument(prefix + "map_hash is required when TerrainLayer is enabled");
    }
    if (!finitePositive(max_age_sec_) || !finitePositive(live_support_radius_m_) ||
      !finitePositive(live_support_z_tolerance_m_) || !finitePositive(status_search_radius_m_) ||
      !finitePositive(entry_distance_m_) || !finitePositive(stair_height_tolerance_m_) ||
      !finitePositive(body_half_length_m_) || !finitePositive(body_half_width_m_) ||
      !std::isfinite(robot_ground_z_offset_m_) || robot_ground_z_offset_m_ < 0.0 ||
      !finitePositive(rebuild_period_sec_) || !finiteRatio(status_min_confidence_) ||
      !finiteRatio(status_min_support_ratio_))
    {
      throw std::invalid_argument(prefix + "contains an invalid terrain safety parameter");
    }
    if (!finitePositive(builder_config_.normal_radius_m) ||
      builder_config_.min_normal_neighbors < 3U ||
      builder_config_.min_normal_neighbors > 100000U ||
      !std::isfinite(builder_config_.max_plane_residual_m) ||
      builder_config_.max_plane_residual_m < 0.0F ||
      !std::isfinite(builder_config_.flat_slope_threshold_rad) ||
      builder_config_.flat_slope_threshold_rad < 0.0F ||
      !std::isfinite(builder_config_.max_model_slope_rad) ||
      builder_config_.max_model_slope_rad <= builder_config_.flat_slope_threshold_rad ||
      builder_config_.max_model_slope_rad > static_cast<float>(kPi / 2.0) ||
      !finitePositive(builder_config_.support_radius_m) ||
      !std::isfinite(builder_config_.support_plane_tolerance_m) ||
      builder_config_.support_plane_tolerance_m < 0.0F ||
      builder_config_.support_sector_count < 3U ||
      builder_config_.support_sector_count > 360U ||
      !finiteRatio(builder_config_.min_observed_support_ratio) ||
      !finiteRatio(builder_config_.edge_support_ratio) ||
      !finitePositive(builder_config_.surface_connectivity_radius_m) ||
      !std::isfinite(builder_config_.max_surface_height_delta_m) ||
      builder_config_.max_surface_height_delta_m < 0.0F ||
      !std::isfinite(builder_config_.max_surface_normal_change_rad) ||
      builder_config_.max_surface_normal_change_rad < 0.0F ||
      builder_config_.max_surface_normal_change_rad > static_cast<float>(kPi))
    {
      throw std::invalid_argument(prefix + "model contains an invalid terrain segmentation value");
    }
    for (const auto & staircase : configured_staircases_) {
      if (staircase.map_hash != map_hash_ ||
        (!staircase.allow_up && !staircase.allow_down) ||
        staircase.step_count <= 0 || staircase.step_count > 9999)
      {
        throw std::invalid_argument(
                prefix + "staircase map identity/direction is invalid for id " +
                std::to_string(staircase.id));
      }
    }
    if (!configured_staircases_.empty() && require_live_obstacle_for_stairs_ &&
      live_obstacle_topic_.empty())
    {
      throw std::invalid_argument(
              prefix + "live_obstacle_topic is required for configured staircases");
    }
    if ((require_live_ground_ ||
      (!configured_staircases_.empty() && require_live_obstacle_for_stairs_)) &&
      rebuild_period_sec_ > max_age_sec_)
    {
      throw std::invalid_argument(
              prefix + "rebuild_period_sec must not exceed max_age_sec");
    }
    TerrainNode validation_node;
    validation_node.ground_index = 0U;
    const TerrainSnapshot configuration_probe(
      map_hash_, 1U, 0, {validation_node}, configured_staircases_);
    std::string staircase_error;
    if (!configuration_probe.valid(&staircase_error)) {
      throw std::invalid_argument(prefix + "stair model invalid: " + staircase_error);
    }
  }

  current_ = !enabled_;
  resetdGraph();

  if (!enabled_) {
    shared_data_->dgraph_update_request_[name_] = false;
    RCLCPP_WARN(
      node_->get_logger().get_child(name_),
      "TerrainLayer is disabled; no terrain snapshot or constraints will be produced");
    return;
  }

  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  const auto live_qos = rclcpp::SensorDataQoS().keep_last(2);
  terrain_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrain_ground_topic_, map_qos,
    std::bind(&TerrainLayer::terrainGroundCallback, this, std::placeholders::_1));
  live_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    live_ground_topic_, live_qos,
    std::bind(&TerrainLayer::liveGroundCallback, this, std::placeholders::_1));
  live_obstacle_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    live_obstacle_topic_, live_qos,
    std::bind(&TerrainLayer::liveObstacleCallback, this, std::placeholders::_1));
  map_identity_sub_ = node_->create_subscription<std_msgs::msg::String>(
    map_identity_topic_, map_qos,
    std::bind(&TerrainLayer::mapIdentityCallback, this, std::placeholders::_1));

  traversability_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/dddmr_terrain/traversability_cloud", 1);
  support_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/dddmr_terrain/support_cloud", 1);
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/dddmr_terrain/unknown_cloud", 1);
  drop_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/dddmr_terrain/drop_cloud", 1);
  stair_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/dddmr_terrain/stair_markers", map_qos);
  rejection_reason_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/dddmr_terrain/rejection_reason", 10);
  if (publish_status_) {
    status_pub_ = node_->create_publisher<dddmr_sys_core::msg::TerrainStatus>(
      "/dddmr_terrain/status", 10);
  }

  RCLCPP_WARN(
    node_->get_logger().get_child(name_),
    "TerrainLayer ENABLED for map '%s' (%zu manually configured staircases); "
    "live ground required: %s, live stair obstacles required: %s",
    map_hash_.c_str(), configured_staircases_.size(),
    require_live_ground_ ? "true" : "false",
    (require_live_obstacle_for_stairs_ && !configured_staircases_.empty()) ?
    "true" : "false");
}

void TerrainLayer::terrainGroundCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (!transformCloud(*msg, transformed.get())) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
      "Rejected terrain ground cloud because it cannot be transformed to %s",
      gbl_utils_->getGblFrame().c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  terrain_ground_cloud_ = std::move(transformed);
  ++terrain_cloud_generation_;
}

void TerrainLayer::liveGroundCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const rclcpp::Time message_time(msg->header.stamp);
  const double message_age = (node_->now() - message_time).seconds();
  if (message_time.nanoseconds() <= 0 || !std::isfinite(message_age) ||
    message_age < 0.0 || message_age > max_age_sec_)
  {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
      "Rejected stale/future live ground message (age %.3f s)", message_age);
    return;
  }
  auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (!transformCloud(*msg, transformed.get()) || transformed->empty()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
      "Rejected empty/untransformable live ground cloud");
    return;
  }
  auto tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  tree->setInputCloud(transformed);
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  live_ground_cloud_ = std::move(transformed);
  live_ground_kdtree_ = std::move(tree);
  last_live_ground_time_ = message_time;
  has_live_ground_ = true;
  ++live_cloud_generation_;
}

void TerrainLayer::liveObstacleCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const rclcpp::Time message_time(msg->header.stamp);
  const double message_age = (node_->now() - message_time).seconds();
  if (message_time.nanoseconds() <= 0 || !std::isfinite(message_age) ||
    message_age < 0.0 || message_age > max_age_sec_)
  {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
      "Rejected stale/future live stair-obstacle message (age %.3f s)", message_age);
    return;
  }
  auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (!transformCloud(*msg, transformed.get()) || transformed->empty()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
      "Rejected empty/untransformable live stair-obstacle cloud");
    return;
  }
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  live_obstacle_cloud_ = std::move(transformed);
  last_live_obstacle_time_ = message_time;
  has_live_obstacle_ = true;
  ++live_obstacle_generation_;
}

void TerrainLayer::mapIdentityCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string received = normalizeSha256(msg->data);
  bool matches = false;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    has_map_identity_ = !received.empty();
    map_identity_matches_ = has_map_identity_ && received == map_hash_;
    matches = map_identity_matches_;
  }
  if (!matches) {
    shared_data_->setTerrainSnapshot(nullptr);
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "Terrain map identity rejected: expected %s, received %s",
      map_hash_.c_str(), received.empty() ? "invalid" : received.c_str());
  }
}

bool TerrainLayer::transformCloud(
  const sensor_msgs::msg::PointCloud2 & input,
  pcl::PointCloud<pcl::PointXYZI> * output) const
{
  if (output == nullptr || input.data.empty()) {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI> raw;
  pcl::fromROSMsg(input, raw);
  std::vector<int> retained_indices;
  raw.is_dense = false;
  pcl::removeNaNFromPointCloud(raw, raw, retained_indices);
  if (raw.empty()) {
    return false;
  }
  if (input.header.frame_id.empty() || input.header.frame_id == gbl_utils_->getGblFrame()) {
    *output = std::move(raw);
  } else {
    try {
      const rclcpp::Time cloud_time(input.header.stamp);
      if (cloud_time.nanoseconds() <= 0) {
        return false;
      }
      const auto transform = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), input.header.frame_id,
        cloud_time, rclcpp::Duration::from_seconds(0.20));
      const Eigen::Affine3d affine = tf2::transformToEigen(transform);
      pcl::transformPointCloud(raw, *output, affine.matrix().cast<float>());
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger().get_child(name_), *node_->get_clock(), 2000,
        "Terrain cloud transform failed: %s", exception.what());
      return false;
    }
  }
  output->header.frame_id = gbl_utils_->getGblFrame();
  return !output->empty();
}

bool TerrainLayer::liveGroundIsFresh() const
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  if (!has_live_ground_) {
    return false;
  }
  const double age = (node_->now() - last_live_ground_time_).seconds();
  return std::isfinite(age) && age >= 0.0 && age <= max_age_sec_;
}

bool TerrainLayer::liveObstacleIsFresh() const
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  if (!has_live_obstacle_) {
    return false;
  }
  const double age = (node_->now() - last_live_obstacle_time_).seconds();
  return std::isfinite(age) && age >= 0.0 && age <= max_age_sec_;
}

bool TerrainLayer::mapIdentityMatches() const
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  return has_map_identity_ && map_identity_matches_;
}

bool TerrainLayer::rebuildSnapshot()
{
  if (!enabled_) {
    return false;
  }
  if (!mapIdentityMatches()) {
    shared_data_->setTerrainSnapshot(nullptr);
    return false;
  }

  std::vector<Eigen::Vector3f> ground_points;
  std::vector<Eigen::Vector3f> support_points;
  std::size_t current_ground_size = 0U;
  std::uint64_t static_ground_generation = 0U;
  {
    std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
    if (!shared_data_->is_static_layer_ready_ || !shared_data_->pcl_ground_ ||
      shared_data_->pcl_ground_->empty())
    {
      shared_data_->setTerrainSnapshot(nullptr);
      return false;
    }
    current_ground_size = shared_data_->pcl_ground_->size();
    static_ground_generation = shared_data_->getStaticGroundGeneration();
    ground_points.reserve(current_ground_size);
    support_points.reserve(current_ground_size);
    for (const auto & point : shared_data_->pcl_ground_->points) {
      const Eigen::Vector3f value(point.x, point.y, point.z);
      ground_points.push_back(value);
      support_points.push_back(value);
    }
  }

  pcl::PointCloud<pcl::PointXYZI> terrain_ground;
  pcl::PointCloud<pcl::PointXYZI> live_ground;
  pcl::PointCloud<pcl::PointXYZI> live_obstacles;
  std::uint64_t terrain_generation = 0U;
  std::uint64_t live_generation = 0U;
  std::uint64_t live_obstacle_generation = 0U;
  std::int64_t live_ground_stamp_nanoseconds = 0;
  std::int64_t live_obstacle_stamp_nanoseconds = 0;
  bool captured_live_ground = false;
  bool captured_live_obstacle = false;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    terrain_ground = *terrain_ground_cloud_;
    live_ground = *live_ground_cloud_;
    live_obstacles = *live_obstacle_cloud_;
    terrain_generation = terrain_cloud_generation_;
    live_generation = live_cloud_generation_;
    live_obstacle_generation = live_obstacle_generation_;
    live_ground_stamp_nanoseconds = last_live_ground_time_.nanoseconds();
    live_obstacle_stamp_nanoseconds = last_live_obstacle_time_.nanoseconds();
    captured_live_ground = has_live_ground_;
    captured_live_obstacle = has_live_obstacle_;
  }
  support_points.reserve(support_points.size() + terrain_ground.size());
  for (const auto & point : terrain_ground.points) {
    support_points.emplace_back(point.x, point.y, point.z);
  }

  const std::uint64_t next_version = ++snapshot_version_;
  const std::int64_t build_stamp_nanoseconds = node_->now().nanoseconds();
  const bool rebuild_static_model = static_base_nodes_.empty() ||
    static_ground_generation != built_static_ground_generation_ ||
    current_ground_size != built_ground_size_ ||
    terrain_generation != built_terrain_cloud_generation_;
  TerrainSnapshotConstPtr base_snapshot;
  TerrainModelBuildStatistics statistics = static_model_statistics_;
  if (rebuild_static_model) {
    TerrainModelBuildInput input;
    input.map_hash = map_hash_;
    input.version = next_version;
    input.stamp_nanoseconds = build_stamp_nanoseconds;
    input.mapground_points = ground_points;
    input.support_points = std::move(support_points);
    const TerrainModelBuildResult result = TerrainModelBuilder::build(input, builder_config_);
    if (!result.ok()) {
      static_base_nodes_.clear();
      shared_data_->setTerrainSnapshotForStaticGroundGeneration(
        nullptr, static_ground_generation);
      RCLCPP_ERROR(
        node_->get_logger().get_child(name_), "Terrain model build failed closed: %s",
        result.error.c_str());
      return false;
    }
    base_snapshot = result.snapshot;
    static_base_nodes_ = result.snapshot->nodes();
    static_model_statistics_ = result.statistics;
    statistics = result.statistics;
  } else {
    base_snapshot = std::make_shared<const TerrainSnapshot>(
      map_hash_, next_version, build_stamp_nanoseconds, static_base_nodes_);
  }

  const std::int64_t annotation_time_nanoseconds = node_->now().nanoseconds();
  const bool live_available = capturedTerrainInputIsFresh(
    captured_live_ground, annotation_time_nanoseconds,
    live_ground_stamp_nanoseconds, max_age_sec_);
  const bool live_obstacle_available = capturedTerrainInputIsFresh(
    captured_live_obstacle, annotation_time_nanoseconds,
    live_obstacle_stamp_nanoseconds, max_age_sec_);
  const auto annotated = annotateStaircases(
    base_snapshot, ground_points, live_ground, live_available,
    live_obstacles, live_obstacle_available);
  std::string validation_error;
  if (!annotated || !annotated->valid(&validation_error)) {
    shared_data_->setTerrainSnapshotForStaticGroundGeneration(
      nullptr, static_ground_generation);
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_), "Terrain annotation failed closed: %s",
      validation_error.c_str());
    return false;
  }

  bool published = false;
  bool captured_inputs_expired = false;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const std::int64_t publication_time_nanoseconds = node_->now().nanoseconds();
    captured_inputs_expired =
      (live_available && !capturedTerrainInputIsFresh(
        true, publication_time_nanoseconds, live_ground_stamp_nanoseconds, max_age_sec_)) ||
      (live_obstacle_available && !capturedTerrainInputIsFresh(
        true, publication_time_nanoseconds, live_obstacle_stamp_nanoseconds, max_age_sec_));
    if (has_map_identity_ && map_identity_matches_ && !captured_inputs_expired) {
      published = shared_data_->setTerrainSnapshotForStaticGroundGeneration(
        annotated, static_ground_generation);
    }
  }
  if (!published) {
    if (captured_inputs_expired) {
      shared_data_->setTerrainSnapshotForStaticGroundGeneration(
        nullptr, static_ground_generation);
    }
    RCLCPP_WARN(
      node_->get_logger().get_child(name_),
      "Discarded terrain snapshot %lu because map identity/static generation changed "
      "or a copied live input expired during build",
      annotated->version());
    return false;
  }
  built_ground_size_ = current_ground_size;
  built_static_ground_generation_ = static_ground_generation;
  built_terrain_cloud_generation_ = terrain_generation;
  built_live_cloud_generation_ = live_generation;
  built_live_obstacle_generation_ = live_obstacle_generation;
  built_with_fresh_live_ground_ = live_available;
  built_with_fresh_live_obstacle_ = live_obstacle_available;
  built_live_ground_stamp_nanoseconds_ =
    live_available ? live_ground_stamp_nanoseconds : 0;
  built_live_obstacle_stamp_nanoseconds_ =
    live_obstacle_available ? live_obstacle_stamp_nanoseconds : 0;
  last_build_time_ = node_->now();
  resetdGraph();
  shared_data_->dgraph_update_request_[name_] = false;

  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "Published terrain snapshot %lu: nodes=%zu flat=%zu ramp=%zu edge=%zu unknown=%zu "
    "surfaces=%zu live_ground=%s live_stair_obstacles=%s",
    annotated->version(), annotated->nodes().size(), statistics.flat_count,
    statistics.ramp_count, statistics.edge_count,
    statistics.unknown_count, statistics.surface_count,
    live_available ? "fresh" : "unavailable",
    live_obstacle_available ? "fresh" : "unavailable");
  return true;
}

TerrainSnapshotConstPtr TerrainLayer::annotateStaircases(
  const TerrainSnapshotConstPtr & base_snapshot,
  const std::vector<Eigen::Vector3f> & ground_points,
  const pcl::PointCloud<pcl::PointXYZI> & live_ground,
  bool live_ground_available,
  const pcl::PointCloud<pcl::PointXYZI> & live_obstacles,
  bool live_obstacle_available) const
{
  if (!base_snapshot || ground_points.size() != base_snapshot->nodes().size()) {
    return nullptr;
  }
  std::vector<TerrainNode> nodes = base_snapshot->nodes();
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr live_ground_ptr;
  pcl::KdTreeFLANN<pcl::PointXYZI> live_ground_tree;
  if (live_ground_available && !live_ground.empty()) {
    live_ground_ptr = live_ground.makeShared();
    live_ground_tree.setInputCloud(live_ground_ptr);
  } else {
    live_ground_available = false;
  }
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr live_obstacle_ptr;
  pcl::KdTreeFLANN<pcl::PointXYZI> live_obstacle_tree;
  if (live_obstacle_available && !live_obstacles.empty()) {
    live_obstacle_ptr = live_obstacles.makeShared();
    live_obstacle_tree.setInputCloud(live_obstacle_ptr);
  } else {
    live_obstacle_available = false;
  }
  const auto livePointMatches = [this](
      const Eigen::Vector3f & point,
      const pcl::PointCloud<pcl::PointXYZI> & cloud,
      pcl::KdTreeFLANN<pcl::PointXYZI> & tree,
      bool available) {
      if (!available || cloud.empty()) {
        return false;
      }
      pcl::PointXYZI query;
      query.x = point.x();
      query.y = point.y();
      query.z = point.z();
      std::vector<int> indices(1);
      std::vector<float> squared_distances(1);
      if (tree.nearestKSearch(query, 1, indices, squared_distances) != 1 ||
        indices.front() < 0 ||
        static_cast<std::size_t>(indices.front()) >= cloud.size())
      {
        return false;
      }
      return squared_distances.front() <=
             static_cast<float>(live_support_radius_m_ * live_support_radius_m_) &&
             std::abs(cloud.points[static_cast<std::size_t>(indices.front())].z - point.z()) <=
             live_support_z_tolerance_m_;
    };

  for (std::size_t index = 0U; index < ground_points.size(); ++index) {
    const Eigen::Vector3f & point = ground_points[index];
    const Eigen::Vector2f point_xy = point.head<2>();
    if (livePointMatches(point, live_ground, live_ground_tree, live_ground_available)) {
      nodes[index].flags |= TERRAIN_NODE_ONLINE_CONFIRMED;
    }
    for (const auto & staircase : configured_staircases_) {
      const float lower_height_error =
        std::abs(point.z() - staircase.lower_landing_center.z());
      const float upper_height_error =
        std::abs(point.z() - staircase.upper_landing_center.z());
      const bool lower_candidate =
        pointInPolygon(point_xy, staircase.lower_landing_polygon_xy) &&
        lower_height_error <= stair_height_tolerance_m_;
      const bool upper_candidate =
        pointInPolygon(point_xy, staircase.upper_landing_polygon_xy) &&
        upper_height_error <= stair_height_tolerance_m_;
      // If landing polygons overlap in XY, z must select exactly one level.
      // An exact tie is ambiguous and remains unannotated/fail-closed.
      const bool on_lower_landing = lower_candidate &&
        (!upper_candidate || lower_height_error < upper_height_error);
      const bool on_upper_landing = upper_candidate &&
        (!lower_candidate || upper_height_error < lower_height_error);
      const float relative_height = point.z() - staircase.lower_landing_center.z();
      const float total_rise =
        static_cast<float>(staircase.step_count) * staircase.riser_height_m;
      const bool in_corridor = pointInPolygon(
        point_xy, staircase.corridor_polygon_xy) &&
        relative_height >= -stair_height_tolerance_m_ &&
        relative_height <= total_rise + stair_height_tolerance_m_;
      if (!on_lower_landing && !on_upper_landing && !in_corridor) {
        continue;
      }

      TerrainNode & node = nodes[index];
      // Stair nodes use a class-specific source: support/landing from live
      // ground and vertical risers from the independent obstacle stream.
      node.flags &= ~static_cast<std::uint32_t>(TERRAIN_NODE_ONLINE_CONFIRMED);
      node.staircase_id = staircase.id;
      node.flags |= TERRAIN_NODE_MANUAL_CORRIDOR;
      node.confidence = std::min(node.confidence, staircase.confidence);
      if (on_lower_landing || on_upper_landing) {
        node.flags |= TERRAIN_NODE_LANDING;
        node.flags |= on_lower_landing ?
          TERRAIN_NODE_LOWER_LANDING : TERRAIN_NODE_UPPER_LANDING;
      } else {
        const int height_level = static_cast<int>(std::lround(
          relative_height / staircase.riser_height_m));
        const float expected_height =
          staircase.lower_landing_center.z() +
          static_cast<float>(height_level) * staircase.riser_height_m;
        const int step_index = std::clamp(height_level - 1, 0, staircase.step_count - 1);
        node.step_index = step_index;
        node.surface_id = 1000000 + staircase.id * 10000 + step_index;
        if (height_level >= 1 && height_level <= staircase.step_count &&
          std::abs(point.z() - expected_height) <= stair_height_tolerance_m_)
        {
          node.terrain_class = TerrainClass::STAIR_TREAD;
        } else {
          node.terrain_class = TerrainClass::STAIR_RISER;
          node.support_ratio = 0.0F;
          node.confidence = 0.0F;
        }
      }

      const bool confirm_from_obstacles =
        node.terrain_class == TerrainClass::STAIR_RISER;
      const bool confirmation_available = confirm_from_obstacles ?
        live_obstacle_available : live_ground_available;
      auto & confirmation_tree = confirm_from_obstacles ?
        live_obstacle_tree : live_ground_tree;
      const auto & confirmation_cloud = confirm_from_obstacles ?
        live_obstacles : live_ground;
      if (livePointMatches(
          point, confirmation_cloud, confirmation_tree, confirmation_available))
      {
        node.flags |= TERRAIN_NODE_ONLINE_CONFIRMED;
      }
      break;
    }
  }

  // Generic PCA mixes adjacent tread elevations when its radius is comparable
  // to one tread depth. Refit each surveyed landing/tread using only points on
  // that explicit stair level; otherwise a perfectly regular staircase is
  // mislabeled low-confidence because the neighborhood contains two planes.
  using StairLevelKey = std::pair<std::int32_t, std::int32_t>;
  std::map<StairLevelKey, std::vector<std::size_t>> stair_level_indices;
  for (std::size_t index = 0U; index < nodes.size(); ++index) {
    const TerrainNode & node = nodes[index];
    if (node.staircase_id < 0) {
      continue;
    }
    // The base snapshot intentionally has no staircase list; use the validated
    // configured model while building this annotated snapshot.
    const auto configured = std::find_if(
      configured_staircases_.begin(), configured_staircases_.end(),
      [&node](const StaircaseModel & candidate) {
        return candidate.id == node.staircase_id;
      });
    if (configured == configured_staircases_.end()) {
      continue;
    }
    std::int32_t location = -2;
    if ((node.flags & TERRAIN_NODE_LANDING) != 0U) {
      const bool lower = (node.flags & TERRAIN_NODE_LOWER_LANDING) != 0U;
      const bool upper = (node.flags & TERRAIN_NODE_UPPER_LANDING) != 0U;
      if (lower != upper) {
        location = lower ? -1 : configured->step_count;
      }
    } else if (node.terrain_class == TerrainClass::STAIR_TREAD &&
      node.step_index >= 0 && node.step_index < configured->step_count)
    {
      location = node.step_index;
    }
    if (location >= -1) {
      stair_level_indices[{node.staircase_id, location}].push_back(index);
    }
  }

  for (const auto & [key, indices] : stair_level_indices) {
    const auto configured = std::find_if(
      configured_staircases_.begin(), configured_staircases_.end(),
      [&key](const StaircaseModel & candidate) {return candidate.id == key.first;});
    if (configured == configured_staircases_.end() ||
      indices.size() < builder_config_.min_normal_neighbors)
    {
      for (const std::size_t index : indices) {
        nodes[index].support_ratio = 0.0F;
        nodes[index].confidence = 0.0F;
      }
      continue;
    }

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const std::size_t index : indices) {
      centroid += ground_points[index];
    }
    centroid /= static_cast<float>(indices.size());
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const std::size_t index : indices) {
      const Eigen::Vector3f delta = ground_points[index] - centroid;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<float>(indices.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) {
      for (const std::size_t index : indices) {
        nodes[index].support_ratio = 0.0F;
        nodes[index].confidence = 0.0F;
      }
      continue;
    }
    Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0.0F) {
      normal = -normal;
    }
    const float roughness = std::sqrt(std::max(0.0F, solver.eigenvalues()(0)));
    const float plane_confidence = builder_config_.max_plane_residual_m > kGeometryEpsilon ?
      std::clamp(
      1.0F - roughness / builder_config_.max_plane_residual_m, 0.0F, 1.0F) :
      (roughness <= kGeometryEpsilon ? 1.0F : 0.0F);
    const float fit_radius = std::min(
      builder_config_.normal_radius_m,
      std::max(
        0.06F, std::min(0.75F * configured->tread_depth_m, 0.25F * configured->width_m)));
    const float fit_radius_squared = fit_radius * fit_radius;
    const float expected_height = key.second < 0 ?
      configured->lower_landing_center.z() :
      (key.second == configured->step_count ? configured->upper_landing_center.z() :
      configured->lower_landing_center.z() +
      static_cast<float>(key.second + 1) * configured->riser_height_m);
    const std::int32_t surface_id = 1000000 + configured->id * 10000 +
      (key.second < 0 ? 9000 :
      (key.second == configured->step_count ? 9001 : key.second));

    for (const std::size_t index : indices) {
      std::size_t neighbor_count = 0U;
      for (const std::size_t candidate : indices) {
        if ((ground_points[candidate].head<2>() -
          ground_points[index].head<2>()).squaredNorm() <= fit_radius_squared)
        {
          ++neighbor_count;
        }
      }
      TerrainNode & node = nodes[index];
      const float density_support = std::min(
        1.0F, static_cast<float>(neighbor_count) /
        static_cast<float>(builder_config_.min_normal_neighbors));
      const float height_confidence = stair_height_tolerance_m_ > kGeometryEpsilon ?
        std::clamp(
        1.0F - std::abs(ground_points[index].z() - expected_height) /
        static_cast<float>(stair_height_tolerance_m_), 0.0F, 1.0F) : 0.0F;
      node.normal = normal;
      node.slope_rad = std::acos(std::clamp(normal.z(), -1.0F, 1.0F));
      node.roughness_m = roughness;
      node.support_ratio = density_support;
      node.confidence = std::min({
        plane_confidence, height_confidence, density_support, configured->confidence});
      node.surface_id = surface_id;
      if ((node.flags & TERRAIN_NODE_LANDING) != 0U) {
        node.terrain_class = node.slope_rad <= builder_config_.flat_slope_threshold_rad ?
          TerrainClass::FLAT : TerrainClass::RAMP;
      }
      if (roughness > builder_config_.max_plane_residual_m ||
        node.slope_rad > builder_config_.max_model_slope_rad ||
        density_support < builder_config_.min_observed_support_ratio)
      {
        node.confidence = 0.0F;
      } else {
        node.flags |= TERRAIN_NODE_OBSERVED;
      }
    }
  }

  return std::make_shared<const TerrainSnapshot>(
    base_snapshot->mapHash(), base_snapshot->version(), base_snapshot->stampNanoseconds(),
    std::move(nodes), configured_staircases_);
}

void TerrainLayer::selfClear()
{
  if (shared_data_->dgraph_update_request_[name_]) {
    resetdGraph();
    shared_data_->dgraph_update_request_[name_] = false;
  }
}

void TerrainLayer::selfMark()
{
  if (!enabled_) {
    return;
  }
  std::size_t ground_size = 0U;
  std::uint64_t static_ground_generation = 0U;
  {
    std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
    ground_size = shared_data_->pcl_ground_ ? shared_data_->pcl_ground_->size() : 0U;
    static_ground_generation = shared_data_->getStaticGroundGeneration();
  }
  std::uint64_t terrain_generation = 0U;
  std::uint64_t live_generation = 0U;
  std::uint64_t live_obstacle_generation = 0U;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    terrain_generation = terrain_cloud_generation_;
    live_generation = live_cloud_generation_;
    live_obstacle_generation = live_obstacle_generation_;
  }
  const double since_build = last_build_time_.nanoseconds() == 0 ?
    std::numeric_limits<double>::infinity() : (node_->now() - last_build_time_).seconds();
  const bool static_changed =
    static_ground_generation != built_static_ground_generation_ ||
    ground_size != built_ground_size_ ||
    terrain_generation != built_terrain_cloud_generation_;
  const bool live_update_due = live_generation != built_live_cloud_generation_ &&
    since_build >= rebuild_period_sec_;
  const bool live_obstacle_update_due =
    live_obstacle_generation != built_live_obstacle_generation_ &&
    since_build >= rebuild_period_sec_;
  const bool stale_confirmation_due =
    (built_with_fresh_live_ground_ && !capturedTerrainInputIsFresh(
      true, node_->now().nanoseconds(), built_live_ground_stamp_nanoseconds_, max_age_sec_)) ||
    (built_with_fresh_live_obstacle_ && !capturedTerrainInputIsFresh(
      true, node_->now().nanoseconds(), built_live_obstacle_stamp_nanoseconds_, max_age_sec_));
  if (static_changed || live_update_due || live_obstacle_update_due ||
    stale_confirmation_due ||
    !shared_data_->getTerrainSnapshot())
  {
    rebuildSnapshot();
  }

  const auto snapshot = shared_data_->getTerrainSnapshot();
  if (snapshot) {
    publishDebugClouds(snapshot);
    publishStairMarkers(snapshot);
  }
  if (publish_status_) {
    publishRobotTerrainStatus(snapshot);
  }
}

void TerrainLayer::updateLethalPointCloud()
{
  current_lethal_->clear();
  current_lethal_->header.frame_id = gbl_utils_->getGblFrame();
  if (!enabled_) {
    return;
  }
  const auto snapshot = shared_data_->getTerrainSnapshot();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if (!snapshot || !shared_data_->pcl_ground_ ||
    snapshot->nodes().size() != shared_data_->pcl_ground_->size())
  {
    return;
  }
  for (std::size_t index = 0U; index < snapshot->nodes().size(); ++index) {
    const TerrainClass terrain_class = snapshot->nodes()[index].terrain_class;
    if (terrain_class == TerrainClass::UNKNOWN || terrain_class == TerrainClass::EDGE ||
      terrain_class == TerrainClass::DROP || terrain_class == TerrainClass::STAIR_RISER)
    {
      current_lethal_->push_back(shared_data_->pcl_ground_->points[index]);
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TerrainLayer::getObservation()
{
  return sensor_current_observation_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TerrainLayer::getLethal()
{
  return current_lethal_;
}

void TerrainLayer::resetdGraph()
{
  std::size_t size = 0U;
  {
    std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
    size = shared_data_->static_ground_size_;
  }
  dGraph_.clear();
  dGraph_.initial(size, enabled_ ? 0.0 : gbl_utils_->getMaxObstacleDistance());
  if (!enabled_) {
    return;
  }
  const auto snapshot = shared_data_->getTerrainSnapshot();
  if (!snapshot || snapshot->nodes().size() != size) {
    return;
  }
  for (std::size_t index = 0U; index < size; ++index) {
    const TerrainClass terrain_class = snapshot->nodes()[index].terrain_class;
    const bool traversable = terrain_class == TerrainClass::FLAT ||
      terrain_class == TerrainClass::RAMP || terrain_class == TerrainClass::STAIR_TREAD;
    const bool online_support_ready = !is_local_planner_ || !require_live_ground_ ||
      (snapshot->nodes()[index].flags & TERRAIN_NODE_ONLINE_CONFIRMED) != 0U;
    dGraph_.setValue(
      static_cast<unsigned int>(index),
      (traversable && online_support_ready) ? gbl_utils_->getMaxObstacleDistance() : 0.0);
  }
}

double TerrainLayer::get_dGraphValue(unsigned int index)
{
  if (!enabled_) {
    return gbl_utils_->getMaxObstacleDistance();
  }
  return dGraph_.getValue(index);
}

bool TerrainLayer::isCurrent()
{
  if (!enabled_) {
    return true;
  }
  const auto snapshot = shared_data_->getTerrainSnapshot();
  std::string error;
  const std::int64_t now_nanoseconds = node_->now().nanoseconds();
  const bool stair_obstacle_required =
    !configured_staircases_.empty() && require_live_obstacle_for_stairs_;
  const bool online_inputs_required = require_live_ground_ || stair_obstacle_required;
  const bool snapshot_fresh = snapshot && terrainSnapshotIsFresh(
    online_inputs_required, now_nanoseconds, snapshot->stampNanoseconds(), max_age_sec_);
  const bool live_ground_ready = !require_live_ground_ ||
    (built_with_fresh_live_ground_ && capturedTerrainInputIsFresh(
      true, now_nanoseconds, built_live_ground_stamp_nanoseconds_, max_age_sec_));
  const bool stair_obstacle_ready = !stair_obstacle_required ||
    (built_with_fresh_live_obstacle_ && capturedTerrainInputIsFresh(
      true, now_nanoseconds, built_live_obstacle_stamp_nanoseconds_, max_age_sec_));
  return snapshot && snapshot_fresh && snapshot->mapHash() == map_hash_ &&
    snapshot->valid(&error) &&
    mapIdentityMatches() && live_ground_ready &&
    stair_obstacle_ready;
}

void TerrainLayer::publishDebugClouds(const TerrainSnapshotConstPtr & snapshot)
{
  if (!publish_debug_clouds_ || !snapshot) {
    return;
  }
  if (traversability_pub_->get_subscription_count() == 0U &&
    support_pub_->get_subscription_count() == 0U &&
    unknown_pub_->get_subscription_count() == 0U &&
    drop_pub_->get_subscription_count() == 0U)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI> traversability;
  pcl::PointCloud<pcl::PointXYZI> support;
  pcl::PointCloud<pcl::PointXYZI> unknown;
  pcl::PointCloud<pcl::PointXYZI> drop;
  {
    std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
    if (!shared_data_->pcl_ground_ ||
      shared_data_->pcl_ground_->size() != snapshot->nodes().size())
    {
      return;
    }
    traversability.reserve(snapshot->nodes().size());
    support.reserve(snapshot->nodes().size());
    for (std::size_t index = 0U; index < snapshot->nodes().size(); ++index) {
      pcl::PointXYZI point = shared_data_->pcl_ground_->points[index];
      point.intensity = static_cast<float>(terrainClassMessageValue(
        snapshot->nodes()[index].terrain_class));
      traversability.push_back(point);
      point.intensity = snapshot->nodes()[index].support_ratio;
      support.push_back(point);
      if (snapshot->nodes()[index].terrain_class == TerrainClass::UNKNOWN) {
        unknown.push_back(point);
      }
      if (snapshot->nodes()[index].terrain_class == TerrainClass::EDGE ||
        snapshot->nodes()[index].terrain_class == TerrainClass::DROP ||
        snapshot->nodes()[index].terrain_class == TerrainClass::STAIR_RISER)
      {
        drop.push_back(point);
      }
    }
  }

  const auto stamp = node_->now();
  const auto publish = [this, &stamp](
      const pcl::PointCloud<pcl::PointXYZI> & cloud,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
    {
      if (publisher->get_subscription_count() == 0U) {
        return;
      }
      sensor_msgs::msg::PointCloud2 message;
      pcl::toROSMsg(cloud, message);
      message.header.frame_id = gbl_utils_->getGblFrame();
      message.header.stamp = stamp;
      publisher->publish(message);
    };
  publish(traversability, traversability_pub_);
  publish(support, support_pub_);
  publish(unknown, unknown_pub_);
  publish(drop, drop_pub_);
}

void TerrainLayer::publishStairMarkers(const TerrainSnapshotConstPtr & snapshot)
{
  if (!publish_debug_clouds_ || !snapshot ||
    stair_marker_pub_->get_subscription_count() == 0U)
  {
    return;
  }
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);
  const auto stamp = node_->now();
  int marker_id = 0;
  for (const auto & staircase : snapshot->staircases()) {
    array.markers.push_back(polygonMarker(
      staircase.corridor_polygon_xy, staircase.lower_landing_center.z(),
      gbl_utils_->getGblFrame(), stamp, "stair_corridor", marker_id++,
      color(1.0F, 0.65F, 0.0F, 1.0F)));
    array.markers.push_back(polygonMarker(
      staircase.lower_landing_polygon_xy, staircase.lower_landing_center.z(),
      gbl_utils_->getGblFrame(), stamp, "stair_landing", marker_id++,
      color(0.0F, 0.8F, 0.2F, 1.0F)));
    array.markers.push_back(polygonMarker(
      staircase.upper_landing_polygon_xy, staircase.upper_landing_center.z(),
      gbl_utils_->getGblFrame(), stamp, "stair_landing", marker_id++,
      color(0.0F, 0.8F, 0.2F, 1.0F)));
  }
  stair_marker_pub_->publish(array);
}

void TerrainLayer::publishRobotTerrainStatus(const TerrainSnapshotConstPtr & snapshot)
{
  if (!status_pub_) {
    return;
  }
  dddmr_sys_core::msg::TerrainStatus status;
  status.header.frame_id = gbl_utils_->getGblFrame();
  status.header.stamp = node_->now();
  status.terrain_class = dddmr_sys_core::msg::TerrainStatus::TERRAIN_UNKNOWN;
  status.traversal_state = dddmr_sys_core::msg::TerrainStatus::STATE_NORMAL;
  status.surface_id = -1;
  status.staircase_id = -1;
  status.step_index = -1;
  status.step_count = 0;
  status.allow_forward = false;
  status.allow_reverse = false;
  status.gait_unchanged = true;
  status.rejection_code = static_cast<std::uint16_t>(
    TerrainRejectionReason::MISSING_SNAPSHOT);
  status.rejection_reason = toString(TerrainRejectionReason::MISSING_SNAPSHOT);

  if (!snapshot) {
    status_pub_->publish(status);
    return;
  }
  status.snapshot_version = snapshot->version();
  status.static_ground_generation = built_static_ground_generation_;
  status.map_hash = snapshot->mapHash();
  const std::int64_t status_now_nanoseconds = node_->now().nanoseconds();
  const bool stair_obstacle_required =
    require_live_obstacle_for_stairs_ && !configured_staircases_.empty();
  const bool online_inputs_required = require_live_ground_ || stair_obstacle_required;
  const double snapshot_age = static_cast<double>(
    status_now_nanoseconds - snapshot->stampNanoseconds()) / 1.0e9;
  if (!terrainSnapshotIsFresh(
      online_inputs_required, status_now_nanoseconds,
      snapshot->stampNanoseconds(), max_age_sec_))
  {
    status.data_age_sec = static_cast<float>(std::max(0.0, snapshot_age));
    status.rejection_code = static_cast<std::uint16_t>(TerrainRejectionReason::STALE);
    status.rejection_reason = toString(TerrainRejectionReason::STALE);
    status_pub_->publish(status);
    return;
  }

  geometry_msgs::msg::TransformStamped robot_transform;
  try {
    robot_transform = gbl_utils_->tf2Buffer()->lookupTransform(
      gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & exception) {
    status.rejection_code = static_cast<std::uint16_t>(
      TerrainRejectionReason::INVALID_GEOMETRY);
    status.rejection_reason = std::string{"ROBOT_TF: "} + exception.what();
    status_pub_->publish(status);
    return;
  }
  pcl::PointXYZI query;
  query.x = static_cast<float>(robot_transform.transform.translation.x);
  query.y = static_cast<float>(robot_transform.transform.translation.y);
  query.z = static_cast<float>(
    robot_transform.transform.translation.z - robot_ground_z_offset_m_);
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  int found = 0;
  {
    std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
    if (shared_data_->kdtree_ground_ && shared_data_->pcl_ground_ &&
      !shared_data_->pcl_ground_->empty())
    {
      found = shared_data_->kdtree_ground_->nearestKSearch(
        query, 1, indices, squared_distances);
    }
  }
  if (found <= 0 || indices.front() < 0 ||
    static_cast<std::size_t>(indices.front()) >= snapshot->nodes().size() ||
    squared_distances.front() > status_search_radius_m_ * status_search_radius_m_)
  {
    status.rejection_code = static_cast<std::uint16_t>(TerrainRejectionReason::NO_SUPPORT);
    status.rejection_reason = toString(TerrainRejectionReason::NO_SUPPORT);
    status_pub_->publish(status);
    return;
  }

  const TerrainNode & node = snapshot->nodes()[static_cast<std::size_t>(indices.front())];
  status.terrain_class = terrainClassMessageValue(node.terrain_class);
  status.surface_id = node.surface_id;
  status.staircase_id = node.staircase_id;
  status.step_index = node.step_index;
  status.slope_rad = node.slope_rad;
  status.roughness_m = node.roughness_m;
  status.support_ratio = node.support_ratio;
  status.confidence = node.confidence;
  const double robot_yaw = tf2::getYaw(robot_transform.transform.rotation);
  const Eigen::Vector3f lateral_direction(
    static_cast<float>(-std::sin(robot_yaw)),
    static_cast<float>(std::cos(robot_yaw)), 0.0F);
  const Eigen::Vector3f normalized_node_normal = node.normal.normalized();
  const Eigen::Vector2f forward_direction(
    static_cast<float>(std::cos(robot_yaw)), static_cast<float>(std::sin(robot_yaw)));
  status.longitudinal_slope_rad = std::atan2(
    -normalized_node_normal.head<2>().dot(forward_direction),
    std::max(std::abs(normalized_node_normal.z()), kGeometryEpsilon));
  status.cross_slope_rad = std::atan2(
    std::abs(normalized_node_normal.dot(lateral_direction)),
    std::max(std::abs(normalized_node_normal.z()), kGeometryEpsilon));
  tf2::Quaternion body_orientation(
    robot_transform.transform.rotation.x, robot_transform.transform.rotation.y,
    robot_transform.transform.rotation.z, robot_transform.transform.rotation.w);
  double body_roll = 0.0;
  double body_pitch = 0.0;
  double body_yaw = 0.0;
  tf2::Matrix3x3(body_orientation).getRPY(body_roll, body_pitch, body_yaw);
  status.body_roll_rad = static_cast<float>(body_roll);
  status.body_pitch_rad = static_cast<float>(body_pitch);
  status.on_stair = node.terrain_class == TerrainClass::STAIR_TREAD;
  status.drop_detected = node.terrain_class == TerrainClass::EDGE ||
    node.terrain_class == TerrainClass::DROP ||
    node.terrain_class == TerrainClass::STAIR_RISER;

  TerrainRejectionReason rejection = TerrainRejectionReason::NONE;
  if (node.terrain_class == TerrainClass::UNKNOWN) {
    rejection = TerrainRejectionReason::UNKNOWN;
  } else if (status.drop_detected) {
    rejection = TerrainRejectionReason::DROP;
  } else if (node.confidence < status_min_confidence_) {
    rejection = TerrainRejectionReason::LOW_CONFIDENCE;
  } else if (node.support_ratio < status_min_support_ratio_) {
    rejection = TerrainRejectionReason::NO_SUPPORT;
  }

  const StaircaseModel * staircase = snapshot->staircaseById(node.staircase_id);
  if (staircase != nullptr) {
    status.staircase_id = staircase->id;
    status.step_count = staircase->step_count;
    const Eigen::Vector2f robot_xy(query.x, query.y);
    const Eigen::Vector2f lower_xy = staircase->lower_landing_center.head<2>();
    const Eigen::Vector2f upper_xy = staircase->upper_landing_center.head<2>();
    const float lower_distance = (robot_xy - lower_xy).norm();
    const float upper_distance = (robot_xy - upper_xy).norm();
    const bool node_is_landing =
      (node.flags & TERRAIN_NODE_LANDING) != 0U;
    const bool node_is_lower =
      (node.flags & TERRAIN_NODE_LOWER_LANDING) != 0U;
    const bool node_is_upper =
      (node.flags & TERRAIN_NODE_UPPER_LANDING) != 0U;
    const bool on_lower_landing = node_is_landing && node_is_lower && !node_is_upper &&
      std::abs(query.z - staircase->lower_landing_center.z()) <= stair_height_tolerance_m_ &&
      pointInPolygon(robot_xy, staircase->lower_landing_polygon_xy);
    const bool on_upper_landing = node_is_landing && node_is_upper && !node_is_lower &&
      std::abs(query.z - staircase->upper_landing_center.z()) <= stair_height_tolerance_m_ &&
      pointInPolygon(robot_xy, staircase->upper_landing_polygon_xy);
    Eigen::Vector2f up_direction = staircase->up_axis.head<2>().normalized();
    const double up_yaw = std::atan2(up_direction.y(), up_direction.x());
    const double up_error = std::abs(
      std::atan2(std::sin(up_yaw - robot_yaw), std::cos(up_yaw - robot_yaw)));
    const double down_yaw = std::atan2(-up_direction.y(), -up_direction.x());
    const double down_error = std::abs(
      std::atan2(std::sin(down_yaw - robot_yaw), std::cos(down_yaw - robot_yaw)));
    // Infer the traversal direction from the unchanged normal gait heading.
    // Landing position alone is ambiguous: the upper landing is both a descent
    // entry and an ascent exit (and vice versa for the lower landing).
    const bool ascending = up_error <= down_error;
    Eigen::Vector2f direction = ascending ? up_direction : -up_direction;
    const Eigen::Vector2f target = ascending ? lower_xy : upper_xy;
    const Eigen::Vector2f offset = robot_xy - target;
    status.lateral_error_m = direction.x() * offset.y() - direction.y() * offset.x();
    const double desired_yaw = std::atan2(direction.y(), direction.x());
    status.heading_error_rad = static_cast<float>(
      std::atan2(std::sin(desired_yaw - robot_yaw), std::cos(desired_yaw - robot_yaw)));
    status.landing_valid = on_lower_landing || on_upper_landing;
    status.full_body_on_landing = on_lower_landing ?
      footprintInsidePolygon(
        robot_xy, static_cast<float>(robot_yaw), static_cast<float>(body_half_length_m_),
        static_cast<float>(body_half_width_m_), staircase->lower_landing_polygon_xy) :
      (on_upper_landing && footprintInsidePolygon(
        robot_xy, static_cast<float>(robot_yaw), static_cast<float>(body_half_length_m_),
        static_cast<float>(body_half_width_m_), staircase->upper_landing_polygon_xy));
    const bool correct_entry_landing =
      (ascending && on_lower_landing) || (!ascending && on_upper_landing);
    const bool direction_allowed =
      ascending ? staircase->allow_up : staircase->allow_down;
    const bool live_ready = (!require_live_ground_ ||
      (built_with_fresh_live_ground_ && capturedTerrainInputIsFresh(
        true, node_->now().nanoseconds(),
        built_live_ground_stamp_nanoseconds_, max_age_sec_))) &&
      (!require_live_obstacle_for_stairs_ ||
      (built_with_fresh_live_obstacle_ && capturedTerrainInputIsFresh(
        true, node_->now().nanoseconds(),
        built_live_obstacle_stamp_nanoseconds_, max_age_sec_)));
    const bool traversal_valid = staircase->confidence >= status_min_confidence_ &&
      direction_allowed && live_ready;
    status.at_entry = correct_entry_landing &&
      std::min(lower_distance, upper_distance) <= entry_distance_m_;
    status.entry_valid = traversal_valid && correct_entry_landing;
    status.allow_forward = rejection == TerrainRejectionReason::NONE && traversal_valid;
    status.allow_reverse = false;
  } else {
    status.allow_forward = rejection == TerrainRejectionReason::NONE;
    status.allow_reverse = status.allow_forward;
  }

  if (!online_inputs_required) {
    status.data_age_sec = 0.0F;
  } else {
    double maximum_age = snapshot_age;
    bool has_all_required_sources = true;
    if (require_live_ground_) {
      has_all_required_sources = has_all_required_sources && built_with_fresh_live_ground_;
      if (built_with_fresh_live_ground_) {
        maximum_age = std::max(
          maximum_age, static_cast<double>(
            status_now_nanoseconds - built_live_ground_stamp_nanoseconds_) / 1.0e9);
      }
    }
    if (stair_obstacle_required) {
      has_all_required_sources =
        has_all_required_sources && built_with_fresh_live_obstacle_;
      if (built_with_fresh_live_obstacle_) {
        maximum_age = std::max(
          maximum_age, static_cast<double>(
            status_now_nanoseconds - built_live_obstacle_stamp_nanoseconds_) / 1.0e9);
      }
    }
    status.data_age_sec = has_all_required_sources ?
      static_cast<float>(std::max(0.0, maximum_age)) :
      std::numeric_limits<float>::infinity();
  }
  if ((require_live_ground_ &&
    (!built_with_fresh_live_ground_ || !capturedTerrainInputIsFresh(
      true, status_now_nanoseconds,
      built_live_ground_stamp_nanoseconds_, max_age_sec_))) ||
    (stair_obstacle_required &&
    (!built_with_fresh_live_obstacle_ || !capturedTerrainInputIsFresh(
      true, status_now_nanoseconds,
      built_live_obstacle_stamp_nanoseconds_, max_age_sec_))))
  {
    rejection = TerrainRejectionReason::STALE;
    status.allow_forward = false;
    status.allow_reverse = false;
  }
  status.rejection_code = static_cast<std::uint16_t>(rejection);
  status.rejection_reason = toString(rejection);
  status_pub_->publish(status);

  if (rejection != TerrainRejectionReason::NONE && rejection_reason_pub_) {
    std_msgs::msg::String reason;
    reason.data = status.rejection_reason;
    rejection_reason_pub_->publish(reason);
  }
}

}  // namespace perception_3d
