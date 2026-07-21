#ifndef DDDMR_ROUTE_NAVIGATION__RECORDED_ROUTE_CONTROLLER_HPP_
#define DDDMR_ROUTE_NAVIGATION__RECORDED_ROUTE_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <local_planner/local_planner.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "dddmr_route_navigation/route_tracker.hpp"

namespace dddmr_route_navigation
{

class RecordedRouteController : public rclcpp::Node
{
public:
  explicit RecordedRouteController(const std::string & name = "recorded_route_controller");
  ~RecordedRouteController() override;

  void initialize(const std::shared_ptr<local_planner::Local_Planner> & local_planner);

private:
  enum class State
  {
    DISABLED,
    WAITING_FOR_ROUTE,
    READY,
    ALIGNING_INITIAL_HEADING,
    TRACKING,
    BLOCKED,
    ALIGNING_GOAL_HEADING,
    COMPLETED,
    FAULT
  };

  void routeCallback(const nav_msgs::msg::Path::SharedPtr message);
  void localizationStatusCallback(const std_msgs::msg::String::SharedPtr message);
  void setEnabledCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void controlCycle();

  bool localizationTracking(std::string * reason = nullptr) const;
  bool currentPose(
    geometry_msgs::msg::TransformStamped * pose,
    std::string * reason = nullptr) const;
  void runPlanner(const std::string & generator, const std::string & decision);
  void handlePlannerFailure(dddmr_sys_core::PlannerState planner_state);
  void disable(const std::string & detail, State terminal_state = State::DISABLED);
  void setState(State state, const std::string & detail);
  void publishVelocity(const base_trajectory::Trajectory & trajectory, const std::string & decision);
  void publishZero(const std::string & decision);
  void publishStatus();
  void publishPreparedRoute();

  static std::string stateName(State state);
  static std::string plannerStateName(dddmr_sys_core::PlannerState state);
  static std::string normalizedStatus(const std::string & status);

  std::shared_ptr<local_planner::Local_Planner> local_planner_;
  RouteTracker tracker_;
  mutable std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr progress_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_ready_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prepared_route_publisher_;

  State state_{State::WAITING_FOR_ROUTE};
  State resume_after_block_{State::TRACKING};
  std::string state_detail_{"no route received"};
  std::string localization_status_;
  rclcpp::Time last_localization_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time blocked_since_{0, 0, RCL_ROS_TIME};
  bool have_localization_status_{false};
  bool enabled_{false};
  bool align_goal_heading_{true};
  bool allow_nearest_start_{false};
  bool allow_reverse_{false};

  double controller_frequency_{10.0};
  double localization_status_timeout_{0.75};
  double pose_timeout_{0.75};
  double blocked_timeout_{5.0};
  double max_linear_x_{0.35};
  double max_linear_y_{0.0};
  double max_angular_z_{0.50};
  std::string route_topic_{"/recorded_route"};
  std::string localization_status_topic_{"/localization_status"};
  std::string command_topic_{"cmd_vel"};
  std::string decision_topic_{"/dddmr_go2/p2p_decision"};
  std::string main_generator_{"differential_drive_simple"};
  std::string initial_heading_generator_{"differential_drive_rotate_shortest_angle"};
  std::string goal_heading_generator_{"differential_drive_rotate_shortest_angle"};
};

}  // namespace dddmr_route_navigation

#endif  // DDDMR_ROUTE_NAVIGATION__RECORDED_ROUTE_CONTROLLER_HPP_
