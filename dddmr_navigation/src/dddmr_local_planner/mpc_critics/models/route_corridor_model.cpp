#include <mpc_critics/route_corridor_model.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mpc_critics::RouteCorridorModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

void RouteCorridorModel::onInitialize()
{
  node_->declare_parameter(name_ + ".max_xy_distance", rclcpp::ParameterValue(0.60));
  node_->declare_parameter(name_ + ".max_z_distance", rclcpp::ParameterValue(0.35));
  node_->declare_parameter(
    name_ + ".adaptive_xy_enabled", rclcpp::ParameterValue(false));
  node_->declare_parameter(
    name_ + ".adaptive_requires_lateral_motion", rclcpp::ParameterValue(true));
  node_->declare_parameter(
    name_ + ".adaptive_max_xy_distance", rclcpp::ParameterValue(0.60));
  node_->declare_parameter(
    name_ + ".adaptive_min_obstacle_clearance", rclcpp::ParameterValue(0.50));
  node_->declare_parameter(
    name_ + ".adaptive_max_ground_distance", rclcpp::ParameterValue(0.15));
  node_->get_parameter(name_ + ".max_xy_distance", max_xy_distance_);
  node_->get_parameter(name_ + ".max_z_distance", max_z_distance_);
  node_->get_parameter(name_ + ".adaptive_xy_enabled", adaptive_xy_enabled_);
  node_->get_parameter(
    name_ + ".adaptive_requires_lateral_motion",
    adaptive_requires_lateral_motion_);
  node_->get_parameter(
    name_ + ".adaptive_max_xy_distance", adaptive_max_xy_distance_);
  node_->get_parameter(
    name_ + ".adaptive_min_obstacle_clearance",
    adaptive_min_obstacle_clearance_);
  node_->get_parameter(
    name_ + ".adaptive_max_ground_distance",
    adaptive_max_ground_distance_);
  if (!std::isfinite(max_xy_distance_) || max_xy_distance_ <= 0.0 ||
    !std::isfinite(max_z_distance_) || max_z_distance_ <= 0.0)
  {
    throw std::invalid_argument("route corridor distances must be finite and positive");
  }
  if (adaptive_xy_enabled_ &&
    (!std::isfinite(adaptive_max_xy_distance_) ||
    adaptive_max_xy_distance_ < max_xy_distance_ ||
    !std::isfinite(adaptive_min_obstacle_clearance_) ||
    adaptive_min_obstacle_clearance_ <= 0.0 ||
    !std::isfinite(adaptive_max_ground_distance_) ||
    adaptive_max_ground_distance_ <= 0.0))
  {
    throw std::invalid_argument(
      "adaptive route corridor limits must be finite and positive, and "
      "adaptive_max_xy_distance must not be smaller than max_xy_distance");
  }
  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "route corridor: hard_xy=%.2f max_z=%.2f adaptive=%d "
    "adaptive_xy=%.2f min_clearance=%.2f max_ground_distance=%.2f "
    "requires_lateral=%d",
    max_xy_distance_, max_z_distance_, adaptive_xy_enabled_,
    adaptive_max_xy_distance_, adaptive_min_obstacle_clearance_,
    adaptive_max_ground_distance_, adaptive_requires_lateral_motion_);
}

double RouteCorridorModel::scoreTrajectory(base_trajectory::Trajectory & trajectory)
{
  if (!shared_data_ || !shared_data_->pcl_prune_plan_ ||
    shared_data_->pcl_prune_plan_->points.size() < 2U || trajectory.getPointsSize() == 0U)
  {
    RCLCPP_DEBUG(
      node_->get_logger().get_child(name_),
      "RouteCorridorModel rejects because the reference route or trajectory is empty.");
    return -1.0;
  }

  for (unsigned int trajectory_index = 0; trajectory_index < trajectory.getPointsSize();
    ++trajectory_index)
  {
    const pcl::PointXYZI trajectory_point = trajectory.getPCLPoint(trajectory_index);
    bool inside = false;
    double nearest_xy_distance = std::numeric_limits<double>::infinity();
    pcl::PointXYZI nearest_route_point{};
    for (const auto & route_point : shared_data_->pcl_prune_plan_->points) {
      const double xy_distance = std::hypot(
        static_cast<double>(trajectory_point.x - route_point.x),
        static_cast<double>(trajectory_point.y - route_point.y));
      const double z_distance = std::abs(
        static_cast<double>(trajectory_point.z - route_point.z));
      if (z_distance <= max_z_distance_ && xy_distance < nearest_xy_distance) {
        nearest_xy_distance = xy_distance;
        nearest_route_point = route_point;
      }
      if (xy_distance <= max_xy_distance_ && z_distance <= max_z_distance_) {
        inside = true;
        break;
      }
    }
    if (inside) {
      continue;
    }

    const bool lateral_candidate =
      std::fabs(static_cast<double>(trajectory.yv_)) > 1e-6;
    if (!adaptive_xy_enabled_ ||
      (adaptive_requires_lateral_motion_ && !lateral_candidate) ||
      nearest_xy_distance > adaptive_max_xy_distance_ + 1e-9 ||
      !shared_data_->ground_clearance_query_)
    {
      return -1.0;
    }

    pcl::PointXYZI ground_query = trajectory_point;
    ground_query.z = nearest_route_point.z;
    double nearest_ground_distance = 0.0;
    double obstacle_clearance = 0.0;
    if (!shared_data_->ground_clearance_query_(
        ground_query, nearest_ground_distance, obstacle_clearance) ||
      !std::isfinite(nearest_ground_distance) ||
      nearest_ground_distance > adaptive_max_ground_distance_ + 1e-9 ||
      !std::isfinite(obstacle_clearance) ||
      obstacle_clearance + 1e-9 < adaptive_min_obstacle_clearance_)
    {
      return -1.0;
    }
  }
  return 0.0;
}

}  // namespace mpc_critics
