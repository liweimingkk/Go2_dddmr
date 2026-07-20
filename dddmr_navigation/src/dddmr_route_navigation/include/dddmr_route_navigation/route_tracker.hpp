#ifndef DDDMR_ROUTE_NAVIGATION__ROUTE_TRACKER_HPP_
#define DDDMR_ROUTE_NAVIGATION__ROUTE_TRACKER_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace dddmr_route_navigation
{

struct RouteTrackerConfig
{
  double duplicate_distance{0.02};
  double resample_spacing{0.10};
  double max_input_segment_length{2.0};
  double progress_search_backward{0.60};
  double progress_search_forward{2.00};
  double local_plan_backward{0.50};
  double local_plan_forward{5.00};
  double corridor_max_xy_error{0.60};
  double corridor_max_z_error{0.35};
  double start_max_xy_error{0.60};
  double start_max_z_error{0.35};
  double goal_max_xy_error{0.35};
  double goal_max_z_error{0.25};
};

struct RouteProjection
{
  bool valid{false};
  double progress{0.0};
  double xy_error{std::numeric_limits<double>::infinity()};
  double z_error{std::numeric_limits<double>::infinity()};
  double distance_3d{std::numeric_limits<double>::infinity()};
  std::size_t segment_index{0};
  geometry_msgs::msg::Point point;
};

class RouteTracker
{
public:
  explicit RouteTracker(RouteTrackerConfig config = RouteTrackerConfig());

  bool setRoute(const nav_msgs::msg::Path & route, std::string * error = nullptr);
  bool initialize(
    const geometry_msgs::msg::Point & robot_position,
    bool allow_nearest_start,
    std::string * error = nullptr);
  RouteProjection update(const geometry_msgs::msg::Point & robot_position);
  void resetProgress();

  bool ready() const;
  bool initialized() const;
  bool insideCorridor(const RouteProjection & projection) const;
  bool goalReached(const geometry_msgs::msg::Point & robot_position) const;
  double progress() const;
  double length() const;
  double progressRatio() const;
  const nav_msgs::msg::Path & route() const;
  nav_msgs::msg::Path localPlan() const;

private:
  static double distance3d(
    const geometry_msgs::msg::Point & lhs,
    const geometry_msgs::msg::Point & rhs);
  static bool finitePose(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose interpolatePose(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    double ratio);

  bool validateConfig(std::string * error) const;
  RouteProjection project(
    const geometry_msgs::msg::Point & robot_position,
    double minimum_progress,
    double maximum_progress) const;
  geometry_msgs::msg::PoseStamped poseAt(double route_progress) const;

  RouteTrackerConfig config_;
  nav_msgs::msg::Path route_;
  std::vector<double> arc_lengths_;
  double progress_{0.0};
  bool initialized_{false};
};

}  // namespace dddmr_route_navigation

#endif  // DDDMR_ROUTE_NAVIGATION__ROUTE_TRACKER_HPP_
