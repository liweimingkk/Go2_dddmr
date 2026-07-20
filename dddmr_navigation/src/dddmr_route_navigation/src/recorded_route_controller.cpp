#include "dddmr_route_navigation/recorded_route_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <dddmr_sys_core/dddmr_enum_states.h>

namespace dddmr_route_navigation
{
namespace
{

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

RouteTrackerConfig readTrackerConfig(rclcpp::Node & node)
{
  RouteTrackerConfig config;
  config.duplicate_distance = node.declare_parameter("route.duplicate_distance", 0.02);
  config.resample_spacing = node.declare_parameter("route.resample_spacing", 0.10);
  config.max_input_segment_length = node.declare_parameter(
    "route.max_input_segment_length", 2.0);
  config.progress_search_backward = node.declare_parameter(
    "route.progress_search_backward", 0.60);
  config.progress_search_forward = node.declare_parameter(
    "route.progress_search_forward", 2.00);
  config.local_plan_backward = node.declare_parameter("route.local_plan_backward", 0.50);
  config.local_plan_forward = node.declare_parameter("route.local_plan_forward", 5.00);
  config.corridor_max_xy_error = node.declare_parameter(
    "route.corridor_max_xy_error", 0.60);
  config.corridor_max_z_error = node.declare_parameter(
    "route.corridor_max_z_error", 0.35);
  config.start_max_xy_error = node.declare_parameter("route.start_max_xy_error", 0.60);
  config.start_max_z_error = node.declare_parameter("route.start_max_z_error", 0.35);
  config.goal_max_xy_error = node.declare_parameter("route.goal_max_xy_error", 0.35);
  config.goal_max_z_error = node.declare_parameter("route.goal_max_z_error", 0.25);
  return config;
}

}  // namespace

RecordedRouteController::RecordedRouteController(const std::string & name)
: Node(name), tracker_(readTrackerConfig(*this))
{
  controller_frequency_ = declare_parameter("controller_frequency", 10.0);
  localization_status_timeout_ = declare_parameter("localization_status_timeout_sec", 0.75);
  pose_timeout_ = declare_parameter("pose_timeout_sec", 0.75);
  blocked_timeout_ = declare_parameter("blocked_timeout_sec", 5.0);
  max_linear_x_ = declare_parameter("max_linear_x", 0.35);
  max_linear_y_ = declare_parameter("max_linear_y", 0.0);
  max_angular_z_ = declare_parameter("max_angular_z", 0.50);
  align_goal_heading_ = declare_parameter("align_goal_heading", true);
  allow_nearest_start_ = declare_parameter("allow_nearest_start", false);
  allow_reverse_ = declare_parameter("allow_reverse", false);
  route_topic_ = declare_parameter("route_topic", std::string("/recorded_route"));
  localization_status_topic_ = declare_parameter(
    "localization_status_topic", std::string("/localization_status"));
  main_generator_ = declare_parameter(
    "main_trajectory_generator", std::string("differential_drive_simple"));
  initial_heading_generator_ = declare_parameter(
    "initial_heading_trajectory_generator",
    std::string("differential_drive_rotate_shortest_angle"));
  goal_heading_generator_ = declare_parameter(
    "goal_heading_trajectory_generator",
    std::string("differential_drive_rotate_shortest_angle"));

  if (!std::isfinite(controller_frequency_) || controller_frequency_ <= 0.0 ||
    !std::isfinite(localization_status_timeout_) || localization_status_timeout_ <= 0.0 ||
    !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0 ||
    !std::isfinite(blocked_timeout_) || blocked_timeout_ <= 0.0 ||
    !std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0 ||
    !std::isfinite(max_linear_y_) || max_linear_y_ < 0.0 ||
    !std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0 ||
    route_topic_.empty() || localization_status_topic_.empty() ||
    main_generator_.empty() || initial_heading_generator_.empty() || goal_heading_generator_.empty())
  {
    throw std::invalid_argument("recorded route controller parameters are invalid");
  }

  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  decision_publisher_ = create_publisher<std_msgs::msg::String>(
    "/dddmr_go2/p2p_decision", 1);
  status_publisher_ = create_publisher<std_msgs::msg::String>("~/status", latched_qos);
  progress_publisher_ = create_publisher<std_msgs::msg::Float64>("~/progress", 1);
  route_ready_publisher_ = create_publisher<std_msgs::msg::Bool>("~/route_ready", latched_qos);
  prepared_route_publisher_ = create_publisher<nav_msgs::msg::Path>(
    "~/prepared_route", latched_qos);

  route_subscription_ = create_subscription<nav_msgs::msg::Path>(
    route_topic_, latched_qos,
    std::bind(&RecordedRouteController::routeCallback, this, std::placeholders::_1));
  localization_status_subscription_ = create_subscription<std_msgs::msg::String>(
    localization_status_topic_, rclcpp::QoS(10),
    std::bind(
      &RecordedRouteController::localizationStatusCallback, this, std::placeholders::_1));
  set_enabled_service_ = create_service<std_srvs::srv::SetBool>(
    "~/set_enabled",
    std::bind(
      &RecordedRouteController::setEnabledCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_WARN(
    get_logger(),
    "Recorded-route controller starts DISABLED. Loading a route never enables motion; "
    "call ~/set_enabled explicitly after localization is TRACKING.");
}

RecordedRouteController::~RecordedRouteController()
{
  local_planner_.reset();
}

void RecordedRouteController::initialize(
  const std::shared_ptr<local_planner::Local_Planner> & local_planner)
{
  if (!local_planner) {
    throw std::invalid_argument("local planner is null");
  }
  local_planner_ = local_planner;
  const auto period = std::chrono::duration<double>(1.0 / controller_frequency_);
  control_timer_ = create_wall_timer(period, std::bind(&RecordedRouteController::controlCycle, this));
  publishStatus();
  publishPreparedRoute();
}

void RecordedRouteController::routeCallback(const nav_msgs::msg::Path::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = false;
  publishZero("d_route_disabled");

  std::string error;
  if (!tracker_.setRoute(*message, &error)) {
    setState(State::FAULT, "route rejected: " + error);
    RCLCPP_ERROR(get_logger(), "%s", state_detail_.c_str());
  } else {
    std::ostringstream detail;
    detail << "route ready: " << tracker_.route().poses.size() << " poses, " <<
      tracker_.length() << " m; explicit enable required";
    setState(State::READY, detail.str());
    RCLCPP_INFO(get_logger(), "%s", state_detail_.c_str());
  }
  publishPreparedRoute();
}

void RecordedRouteController::localizationStatusCallback(
  const std_msgs::msg::String::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  localization_status_ = normalizedStatus(message->data);
  last_localization_status_time_ = now();
  have_localization_status_ = true;
}

bool RecordedRouteController::localizationTracking(std::string * reason) const
{
  if (!have_localization_status_) {
    if (reason) {
      *reason = "no localization status received";
    }
    return false;
  }
  const double age = (now() - last_localization_status_time_).seconds();
  if (!std::isfinite(age) || age < 0.0 || age > localization_status_timeout_) {
    if (reason) {
      std::ostringstream message;
      message << "localization status stale: age=" << age << " s";
      *reason = message.str();
    }
    return false;
  }
  if (localization_status_ != "TRACKING") {
    if (reason) {
      *reason = "localization state is " + localization_status_ + ", not TRACKING";
    }
    return false;
  }
  return true;
}

bool RecordedRouteController::currentPose(
  geometry_msgs::msg::TransformStamped * pose,
  std::string * reason) const
{
  if (!local_planner_) {
    if (reason) {
      *reason = "local planner is not initialized";
    }
    return false;
  }
  *pose = local_planner_->getGlobalPose();
  if (pose->header.frame_id.empty()) {
    if (reason) {
      *reason = "global robot pose has no frame";
    }
    return false;
  }
  if (tracker_.ready() && pose->header.frame_id != tracker_.route().header.frame_id) {
    if (reason) {
      *reason = "robot pose frame '" + pose->header.frame_id + "' differs from route frame '" +
        tracker_.route().header.frame_id + "'";
    }
    return false;
  }
  const rclcpp::Time stamp(pose->header.stamp);
  const double age = (now() - stamp).seconds();
  if (stamp.nanoseconds() == 0 || !std::isfinite(age) || age < 0.0 || age > pose_timeout_) {
    if (reason) {
      std::ostringstream message;
      message << "global robot pose stale or invalid: age=" << age << " s";
      *reason = message.str();
    }
    return false;
  }
  const auto & translation = pose->transform.translation;
  if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
    !std::isfinite(translation.z))
  {
    if (reason) {
      *reason = "global robot position is not finite";
    }
    return false;
  }
  return true;
}

void RecordedRouteController::setEnabledCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!request->data) {
    disable("disabled by operator", State::DISABLED);
    tracker_.resetProgress();
    response->success = true;
    response->message = "recorded-route control disabled and progress reset";
    return;
  }

  if (!tracker_.ready()) {
    response->success = false;
    response->message = "cannot enable: no valid route is loaded";
    publishZero("d_route_waiting_for_route");
    return;
  }

  std::string reason;
  if (!localizationTracking(&reason)) {
    response->success = false;
    response->message = "cannot enable: " + reason;
    publishZero("d_route_localization_blocked");
    return;
  }

  geometry_msgs::msg::TransformStamped pose;
  if (!currentPose(&pose, &reason)) {
    response->success = false;
    response->message = "cannot enable: " + reason;
    publishZero("d_route_pose_blocked");
    return;
  }

  geometry_msgs::msg::Point robot_position;
  robot_position.x = pose.transform.translation.x;
  robot_position.y = pose.transform.translation.y;
  robot_position.z = pose.transform.translation.z;
  if (!tracker_.initialize(robot_position, allow_nearest_start_, &reason)) {
    response->success = false;
    response->message = "cannot enable: " + reason;
    publishZero("d_route_start_blocked");
    return;
  }

  auto local_plan = tracker_.localPlan();
  const auto stamp = now();
  local_plan.header.stamp = stamp;
  for (auto & route_pose : local_plan.poses) {
    route_pose.header.stamp = stamp;
  }
  local_planner_->setPlan(local_plan.poses);
  local_planner_->resetInPlaceRotationHysteresis();
  enabled_ = true;
  blocked_since_ = now();
  setState(State::ALIGNING_INITIAL_HEADING, "enabled; checking initial route heading");
  response->success = true;
  response->message = "recorded-route control enabled";
}

void RecordedRouteController::controlCycle()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    publishZero(state_ == State::COMPLETED ? "d_route_completed" : "d_route_disabled");
    publishStatus();
    return;
  }

  std::string reason;
  if (!localizationTracking(&reason)) {
    disable(reason, State::FAULT);
    return;
  }

  geometry_msgs::msg::TransformStamped pose;
  if (!currentPose(&pose, &reason)) {
    disable(reason, State::FAULT);
    return;
  }
  geometry_msgs::msg::Point robot_position;
  robot_position.x = pose.transform.translation.x;
  robot_position.y = pose.transform.translation.y;
  robot_position.z = pose.transform.translation.z;

  const RouteProjection projection = tracker_.update(robot_position);
  if (!projection.valid) {
    disable("route projection failed inside bounded progress window", State::FAULT);
    return;
  }
  if (!tracker_.insideCorridor(projection)) {
    std::ostringstream detail;
    detail << "left route corridor: xy_error=" << projection.xy_error <<
      " m, z_error=" << projection.z_error << " m";
    disable(detail.str(), State::FAULT);
    return;
  }

  auto local_plan = tracker_.localPlan();
  const auto stamp = now();
  local_plan.header.stamp = stamp;
  for (auto & route_pose : local_plan.poses) {
    route_pose.header.stamp = stamp;
  }
  local_planner_->setPlan(local_plan.poses);

  if (tracker_.goalReached(robot_position)) {
    if (!align_goal_heading_) {
      disable("route completed", State::COMPLETED);
      return;
    }
    // Preserve a blocked final-alignment episode so its timeout keeps running.
    // A block that occurred during normal tracking may still transition to the
    // final heading phase once the positional goal is reached.
    if (state_ != State::BLOCKED || resume_after_block_ != State::ALIGNING_GOAL_HEADING) {
      setState(State::ALIGNING_GOAL_HEADING, "position reached; aligning final heading");
    }
  }

  switch (state_) {
    case State::ALIGNING_INITIAL_HEADING:
      if (local_planner_->isInitialHeadingAligned()) {
        setState(State::TRACKING, "initial heading aligned");
        publishZero("d_controlling");
      } else {
        runPlanner(initial_heading_generator_, "d_align_heading");
      }
      break;
    case State::TRACKING:
      runPlanner(main_generator_, "d_controlling");
      break;
    case State::BLOCKED:
      if (resume_after_block_ == State::ALIGNING_INITIAL_HEADING) {
        runPlanner(initial_heading_generator_, "d_align_heading");
      } else if (resume_after_block_ == State::ALIGNING_GOAL_HEADING) {
        runPlanner(goal_heading_generator_, "d_align_goal_heading");
      } else {
        runPlanner(main_generator_, "d_controlling");
      }
      break;
    case State::ALIGNING_GOAL_HEADING:
      if (local_planner_->isGoalHeadingAligned()) {
        disable("route completed", State::COMPLETED);
      } else {
        runPlanner(goal_heading_generator_, "d_align_goal_heading");
      }
      break;
    default:
      disable("invalid active controller state", State::FAULT);
      break;
  }
  publishStatus();
}

void RecordedRouteController::runPlanner(
  const std::string & generator,
  const std::string & decision)
{
  base_trajectory::Trajectory trajectory;
  const auto planner_state = local_planner_->computeVelocityCommand(generator, trajectory);
  if (planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND) {
    if (state_ == State::BLOCKED) {
      setState(resume_after_block_, "safe local trajectory recovered");
    }
    publishVelocity(trajectory, decision);
    return;
  }
  handlePlannerFailure(planner_state);
}

void RecordedRouteController::handlePlannerFailure(dddmr_sys_core::PlannerState planner_state)
{
  if (planner_state == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL ||
    planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT ||
    planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING)
  {
    if (state_ != State::BLOCKED) {
      resume_after_block_ = state_;
      blocked_since_ = now();
      setState(
        State::BLOCKED,
        "no safe route-corridor trajectory: " + plannerStateName(planner_state));
    }
    publishZero("d_route_waiting");
    if ((now() - blocked_since_).seconds() > blocked_timeout_) {
      disable("blocked timeout; no global replanning is permitted", State::FAULT);
    }
    return;
  }

  disable("local planner fault: " + plannerStateName(planner_state), State::FAULT);
}

void RecordedRouteController::disable(const std::string & detail, State terminal_state)
{
  enabled_ = false;
  setState(terminal_state, detail);
  publishZero(terminal_state == State::COMPLETED ? "d_route_completed" : "d_route_disabled");
  RCLCPP_WARN(get_logger(), "Recorded-route output disabled: %s", detail.c_str());
}

void RecordedRouteController::setState(State state, const std::string & detail)
{
  if (state_ != state || state_detail_ != detail) {
    RCLCPP_INFO(
      get_logger(), "Recorded-route state %s -> %s: %s",
      stateName(state_).c_str(), stateName(state).c_str(), detail.c_str());
  }
  state_ = state;
  state_detail_ = detail;
  publishStatus();
}

void RecordedRouteController::publishVelocity(
  const base_trajectory::Trajectory & trajectory,
  const std::string & decision)
{
  if (!enabled_ || !std::isfinite(trajectory.xv_) || !std::isfinite(trajectory.yv_) ||
    !std::isfinite(trajectory.thetav_))
  {
    disable("planner returned a non-finite or unauthorized command", State::FAULT);
    return;
  }

  geometry_msgs::msg::Twist command;
  const double minimum_x = allow_reverse_ ? -max_linear_x_ : 0.0;
  command.linear.x = clamp(trajectory.xv_, minimum_x, max_linear_x_);
  command.linear.y = max_linear_y_ > 0.0 ?
    clamp(trajectory.yv_, -max_linear_y_, max_linear_y_) : 0.0;
  command.angular.z = clamp(trajectory.thetav_, -max_angular_z_, max_angular_z_);

  std_msgs::msg::String decision_message;
  decision_message.data = decision;
  decision_publisher_->publish(decision_message);
  command_publisher_->publish(command);
}

void RecordedRouteController::publishZero(const std::string & decision)
{
  std_msgs::msg::String decision_message;
  decision_message.data = decision;
  decision_publisher_->publish(decision_message);
  command_publisher_->publish(geometry_msgs::msg::Twist());
}

void RecordedRouteController::publishStatus()
{
  if (!status_publisher_) {
    return;
  }
  std_msgs::msg::String status;
  status.data = stateName(state_) + ": " + state_detail_;
  status_publisher_->publish(status);

  std_msgs::msg::Float64 progress;
  progress.data = tracker_.progressRatio();
  progress_publisher_->publish(progress);
}

void RecordedRouteController::publishPreparedRoute()
{
  if (!route_ready_publisher_ || !prepared_route_publisher_) {
    return;
  }
  std_msgs::msg::Bool ready;
  ready.data = tracker_.ready();
  route_ready_publisher_->publish(ready);
  if (ready.data) {
    auto route = tracker_.route();
    route.header.stamp = now();
    for (auto & pose : route.poses) {
      pose.header.stamp = route.header.stamp;
    }
    prepared_route_publisher_->publish(route);
  }
}

std::string RecordedRouteController::stateName(State state)
{
  switch (state) {
    case State::DISABLED:
      return "DISABLED";
    case State::WAITING_FOR_ROUTE:
      return "WAITING_FOR_ROUTE";
    case State::READY:
      return "READY";
    case State::ALIGNING_INITIAL_HEADING:
      return "ALIGNING_INITIAL_HEADING";
    case State::TRACKING:
      return "TRACKING";
    case State::BLOCKED:
      return "BLOCKED";
    case State::ALIGNING_GOAL_HEADING:
      return "ALIGNING_GOAL_HEADING";
    case State::COMPLETED:
      return "COMPLETED";
    case State::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

std::string RecordedRouteController::plannerStateName(dddmr_sys_core::PlannerState state)
{
  switch (state) {
    case dddmr_sys_core::TF_FAIL:
      return "TF_FAIL";
    case dddmr_sys_core::PRUNE_PLAN_FAIL:
      return "PRUNE_PLAN_FAIL";
    case dddmr_sys_core::ALL_TRAJECTORIES_FAIL:
      return "ALL_TRAJECTORIES_FAIL";
    case dddmr_sys_core::PERCEPTION_MALFUNCTION:
      return "PERCEPTION_MALFUNCTION";
    case dddmr_sys_core::TRAJECTORY_FOUND:
      return "TRAJECTORY_FOUND";
    case dddmr_sys_core::PATH_BLOCKED_WAIT:
      return "PATH_BLOCKED_WAIT";
    case dddmr_sys_core::PATH_BLOCKED_REPLANNING:
      return "PATH_BLOCKED_REPLANNING";
    case dddmr_sys_core::CONFIGURATION_ERROR:
      return "CONFIGURATION_ERROR";
  }
  return "UNKNOWN";
}

std::string RecordedRouteController::normalizedStatus(const std::string & status)
{
  std::string normalized;
  normalized.reserve(status.size());
  for (const unsigned char character : status) {
    if (!std::isspace(character)) {
      normalized.push_back(static_cast<char>(std::toupper(character)));
    }
  }
  return normalized;
}

}  // namespace dddmr_route_navigation
