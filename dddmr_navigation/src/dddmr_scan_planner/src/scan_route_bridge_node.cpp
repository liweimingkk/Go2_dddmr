#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <dddmr_sys_core/action/get_plan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

namespace dddmr_scan_planner
{

class ScanRouteBridge : public rclcpp::Node
{
public:
  using GetPlan = dddmr_sys_core::action::GetPlan;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GetPlan>;

  ScanRouteBridge()
  : Node("scan_route_bridge")
  {
    action_name_ = declare_parameter<std::string>("global_plan_action", "/get_plan");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    min_path_poses_ = declare_parameter<int>("min_path_poses", 2);
    body_pose_timeout_ = declare_parameter<double>("body_pose_timeout", 0.25);
    start_exclusion_xy_ = declare_parameter<double>("start_exclusion_xy", 0.15);
    min_path_point_separation_ =
      declare_parameter<double>("min_path_point_separation", 0.02);
    if (
      action_name_.empty() || map_frame_.empty() || min_path_poses_ < 2 ||
      !std::isfinite(body_pose_timeout_) || body_pose_timeout_ <= 0.0 ||
      !std::isfinite(start_exclusion_xy_) || start_exclusion_xy_ < 0.0 ||
      !std::isfinite(min_path_point_separation_) ||
      min_path_point_separation_ <= 0.0)
    {
      throw std::invalid_argument(
              "SCAN route bridge requires valid frames, path, and timing limits");
    }

    route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "initial_path", rclcpp::QoS(1).reliable().transient_local());
    route_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "route_ready", rclcpp::QoS(1).reliable().transient_local());
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose_3d", 3,
      std::bind(&ScanRouteBridge::goalPoseCallback, this, std::placeholders::_1));
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 3,
      std::bind(&ScanRouteBridge::clickedPointCallback, this, std::placeholders::_1));
    body_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&ScanRouteBridge::bodyPoseCallback, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<GetPlan>(this, action_name_);
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ScanRouteBridge::trySendPendingGoal, this));

    publishRouteReady(false);
    RCLCPP_INFO(
      get_logger(), "SCAN route bridge ready: goals -> %s -> initial_path",
      action_name_.c_str());
  }

private:
  static bool finitePosition(const geometry_msgs::msg::Point & position)
  {
    return
      std::isfinite(position.x) &&
      std::isfinite(position.y) &&
      std::isfinite(position.z);
  }

  bool bodyPoseIsFresh() const
  {
    if (!body_pose_received_) {
      return false;
    }
    const double age = (now() - body_pose_receipt_).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= body_pose_timeout_;
  }

  static double pointDistance(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second)
  {
    return std::hypot(
      std::hypot(first.x - second.x, first.y - second.y),
      first.z - second.z);
  }

  bool normalizeGoal(geometry_msgs::msg::PoseStamped & goal)
  {
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = map_frame_;
    }
    if (goal.header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Goal rejected: expected frame '%s', received '%s'",
        map_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }
    if (!finitePosition(goal.pose.position)) {
      RCLCPP_ERROR(get_logger(), "Goal rejected: position contains a non-finite value");
      return false;
    }

    const auto & q = goal.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1e-6) {
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = 0.0;
      goal.pose.orientation.w = 1.0;
    }
    return true;
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    queueGoal(*msg);
  }

  void clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header = msg->header;
    goal.pose.position = msg->point;
    goal.pose.orientation.w = 1.0;
    queueGoal(goal);
  }

  void bodyPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (
      msg->header.frame_id != map_frame_ ||
      !finitePosition(msg->pose.pose.position))
    {
      return;
    }
    body_position_ = msg->pose.pose.position;
    body_pose_receipt_ = now();
    body_pose_received_ = true;
  }

  void queueGoal(geometry_msgs::msg::PoseStamped goal)
  {
    if (!normalizeGoal(goal)) {
      return;
    }

    ++goal_generation_;
    pending_goal_ = goal;
    publishRouteReady(false);
    RCLCPP_INFO(
      get_logger(), "Queued SCAN goal %.2f, %.2f, %.2f",
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    trySendPendingGoal();
  }

  void trySendPendingGoal()
  {
    if (request_in_flight_ || !pending_goal_) {
      return;
    }
    if (!bodyPoseIsFresh()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for a fresh SCAN body pose before requesting a route");
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for global planner action %s", action_name_.c_str());
      return;
    }

    const uint64_t generation = goal_generation_;
    GetPlan::Goal request;
    request.goal = *pending_goal_;
    request.activate_threading = true;
    pending_goal_.reset();
    request_in_flight_ = true;

    auto options = rclcpp_action::Client<GetPlan>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](const GoalHandle::SharedPtr & handle)
      {
        if (!handle) {
          request_in_flight_ = false;
        }
        if (generation != goal_generation_) {
          return;
        }
        if (!handle) {
          publishRouteReady(false);
          RCLCPP_ERROR(get_logger(), "Global planner rejected the SCAN route request");
        }
      };
    options.result_callback =
      [this, generation](const GoalHandle::WrappedResult & result)
      {
        request_in_flight_ = false;
        if (generation != goal_generation_) {
          return;
        }
        if (
          result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !result.result ||
          result.result->path.poses.size() < static_cast<size_t>(min_path_poses_))
        {
          publishRouteReady(false);
          RCLCPP_ERROR(
            get_logger(), "Global planner did not return a usable SCAN route");
          return;
        }

        if (!bodyPoseIsFresh()) {
          publishRouteReady(false);
          RCLCPP_ERROR(
            get_logger(), "Global planner route dropped because the body pose is stale");
          return;
        }

        nav_msgs::msg::Path input_path = result.result->path;
        if (input_path.header.frame_id.empty()) {
          input_path.header.frame_id = map_frame_;
        }
        if (input_path.header.frame_id != map_frame_) {
          publishRouteReady(false);
          RCLCPP_ERROR(
            get_logger(), "Global planner returned frame '%s', expected '%s'",
            input_path.header.frame_id.c_str(), map_frame_.c_str());
          return;
        }

        nav_msgs::msg::Path path;
        path.header = input_path.header;
        bool filtering_start = true;
        for (size_t index = 0; index < input_path.poses.size(); ++index) {
          auto pose = input_path.poses[index];
          if (
            (!pose.header.frame_id.empty() && pose.header.frame_id != map_frame_) ||
            !finitePosition(pose.pose.position))
          {
            publishRouteReady(false);
            RCLCPP_ERROR(
              get_logger(), "Global planner returned an invalid SCAN route pose");
            return;
          }
          pose.header.frame_id = map_frame_;

          const double start_xy = std::hypot(
            pose.pose.position.x - body_position_.x,
            pose.pose.position.y - body_position_.y);
          if (filtering_start && start_xy <= start_exclusion_xy_) {
            continue;
          }
          filtering_start = false;

          if (
            !path.poses.empty() &&
            pointDistance(path.poses.back().pose.position, pose.pose.position) <
            min_path_point_separation_)
          {
            if (index + 1 == input_path.poses.size()) {
              path.poses.back() = pose;
            }
            continue;
          }
          path.poses.push_back(pose);
        }
        if (path.poses.empty()) {
          publishRouteReady(false);
          RCLCPP_ERROR(
            get_logger(), "Global planner route has no waypoint beyond the robot start pose");
          return;
        }
        route_pub_->publish(path);
        publishRouteReady(true);
        RCLCPP_INFO(
          get_logger(), "Published SCAN reference route with %zu poses",
          path.poses.size());
      };
    action_client_->async_send_goal(request, options);
  }

  void publishRouteReady(const bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    route_ready_pub_->publish(message);
  }

  std::string action_name_;
  std::string map_frame_;
  int min_path_poses_{2};
  double body_pose_timeout_{0.25};
  double start_exclusion_xy_{0.15};
  double min_path_point_separation_{0.02};
  uint64_t goal_generation_{0};
  bool request_in_flight_{false};
  bool body_pose_received_{false};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  geometry_msgs::msg::Point body_position_;
  rclcpp::Time body_pose_receipt_{0, 0, RCL_ROS_TIME};

  rclcpp_action::Client<GetPlan>::SharedPtr action_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_ready_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_pose_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace dddmr_scan_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dddmr_scan_planner::ScanRouteBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("scan_route_bridge"), "Initialization failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
