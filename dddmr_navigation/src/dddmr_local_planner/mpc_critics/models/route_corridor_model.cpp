#include <mpc_critics/route_corridor_model.h>

#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mpc_critics::RouteCorridorModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

void RouteCorridorModel::onInitialize()
{
  node_->declare_parameter(name_ + ".max_xy_distance", rclcpp::ParameterValue(0.60));
  node_->declare_parameter(name_ + ".max_z_distance", rclcpp::ParameterValue(0.35));
  node_->get_parameter(name_ + ".max_xy_distance", max_xy_distance_);
  node_->get_parameter(name_ + ".max_z_distance", max_z_distance_);
  if (!std::isfinite(max_xy_distance_) || max_xy_distance_ <= 0.0 ||
    !std::isfinite(max_z_distance_) || max_z_distance_ <= 0.0)
  {
    throw std::invalid_argument("route corridor distances must be finite and positive");
  }
  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "hard route corridor: max_xy_distance=%.2f max_z_distance=%.2f",
    max_xy_distance_, max_z_distance_);
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
    for (const auto & route_point : shared_data_->pcl_prune_plan_->points) {
      const double xy_distance = std::hypot(
        static_cast<double>(trajectory_point.x - route_point.x),
        static_cast<double>(trajectory_point.y - route_point.y));
      const double z_distance = std::abs(
        static_cast<double>(trajectory_point.z - route_point.z));
      if (xy_distance <= max_xy_distance_ && z_distance <= max_z_distance_) {
        inside = true;
        break;
      }
    }
    if (!inside) {
      return -1.0;
    }
  }
  return 0.0;
}

}  // namespace mpc_critics
