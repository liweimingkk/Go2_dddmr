#include "dddmr_scan_planner/command_guard_policy.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace dddmr_scan_planner
{

class ScanCommandGuard : public rclcpp::Node
{
public:
  ScanCommandGuard()
  : Node("scan_command_guard")
  {
    CommandGuardLimits limits;
    limits.max_x = declare_parameter<double>("max_x", 0.40);
    limits.max_y = declare_parameter<double>("max_y", 0.0);
    limits.max_yaw = declare_parameter<double>("max_yaw", 0.50);
    limits.max_translation = declare_parameter<double>("max_translation", 0.40);
    limits.raw_command_timeout =
      declare_parameter<double>("raw_command_timeout", 0.15);
    limits.odom_timeout = declare_parameter<double>("odom_timeout", 0.25);
    limits.cloud_timeout = declare_parameter<double>("cloud_timeout", 0.35);
    limits.planner_timeout =
      declare_parameter<double>("planner_timeout", 0.50);
    limits.odom_header_max_age =
      declare_parameter<double>("odom_header_max_age", 0.35);
    limits.cloud_header_max_age =
      declare_parameter<double>("cloud_header_max_age", 0.35);
    limits.max_future_skew = declare_parameter<double>("max_future_skew", 0.05);
    limits.zero_epsilon = declare_parameter<double>("zero_epsilon", 0.001);
    policy_ = std::make_unique<CommandGuardPolicy>(limits);

    const double publish_rate = declare_parameter<double>("publish_rate", 50.0);
    if (!std::isfinite(publish_rate) || publish_rate < 10.0 || publish_rate > 100.0) {
      throw std::invalid_argument("SCAN command guard publish_rate must be in [10, 100] Hz");
    }

    guarded_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("guarded_cmd", 10);
    decision_pub_ = create_publisher<std_msgs::msg::String>("decision", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);

    raw_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "raw_cmd", 10,
      std::bind(&ScanCommandGuard::rawCommandCallback, this, std::placeholders::_1));
    body_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&ScanCommandGuard::bodyPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(&ScanCommandGuard::cloudCallback, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
      "trajectory", 10,
      std::bind(&ScanCommandGuard::trajectoryCallback, this, std::placeholders::_1));
    planner_heartbeat_sub_ = create_subscription<scan_planner_msgs::msg::DataDisp>(
      "planner_heartbeat", 10,
      std::bind(&ScanCommandGuard::plannerHeartbeatCallback, this, std::placeholders::_1));
    route_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "route_ready", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ScanCommandGuard::routeReadyCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ScanCommandGuard::publishCycle, this));
    RCLCPP_INFO(
      get_logger(),
      "SCAN command guard ready: max=(%.2f, %.2f, %.2f), rate=%.1f Hz",
      limits.max_x, limits.max_y, limits.max_yaw, publish_rate);
  }

private:
  static double ageOrInfinity(
    const bool received, const rclcpp::Time & now, const rclcpp::Time & stamp)
  {
    if (!received) {
      return std::numeric_limits<double>::infinity();
    }
    return (now - stamp).seconds();
  }

  void rawCommandCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    state_.raw_command.x = msg->linear.x;
    state_.raw_command.y = msg->linear.y;
    state_.raw_command.yaw = msg->angular.z;
    raw_command_receipt_ = now();
    state_.raw_command_received = true;
  }

  void bodyPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_receipt_ = now();
    odom_header_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    state_.odom_received = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    cloud_receipt_ = now();
    cloud_header_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    state_.cloud_received = true;
  }

  void trajectoryCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr)
  {
    if (!state_.route_ready) {
      return;
    }
    state_.trajectory_received = true;
  }

  void plannerHeartbeatCallback(const scan_planner_msgs::msg::DataDisp::ConstSharedPtr)
  {
    planner_heartbeat_receipt_ = now();
    state_.planner_heartbeat_received = true;
  }

  void routeReadyCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    state_.route_ready = msg->data;
    if (!state_.route_ready) {
      state_.trajectory_received = false;
    }
  }

  void publishCycle()
  {
    const rclcpp::Time current_time = now();
    state_.raw_command_age = ageOrInfinity(
      state_.raw_command_received, current_time, raw_command_receipt_);
    state_.odom_age = ageOrInfinity(
      state_.odom_received, current_time, odom_receipt_);
    state_.cloud_age = ageOrInfinity(
      state_.cloud_received, current_time, cloud_receipt_);
    state_.planner_heartbeat_age = ageOrInfinity(
      state_.planner_heartbeat_received, current_time, planner_heartbeat_receipt_);
    state_.odom_header_age = ageOrInfinity(
      state_.odom_received, current_time, odom_header_);
    state_.cloud_header_age = ageOrInfinity(
      state_.cloud_received, current_time, cloud_header_);

    const CommandGuardResult result = policy_->evaluate(state_);
    geometry_msgs::msg::Twist output;
    output.linear.x = result.command.x;
    output.linear.y = result.command.y;
    output.angular.z = result.command.yaw;
    guarded_cmd_pub_->publish(output);

    std_msgs::msg::String decision;
    decision.data = result.allowed && result.moving ? "d_controlling" : "d_waiting";
    decision_pub_->publish(decision);

    std_msgs::msg::String status;
    status.data = result.reason;
    status_pub_->publish(status);

    if (result.reason != last_reason_) {
      if (result.allowed) {
        RCLCPP_INFO(get_logger(), "SCAN command guard enabled");
      } else if (result.reason == "route_not_ready") {
        RCLCPP_INFO(get_logger(), "SCAN command guard waiting for a valid route");
      } else {
        RCLCPP_WARN(
          get_logger(), "SCAN command guard stopped output: %s", result.reason.c_str());
      }
      last_reason_ = result.reason;
    }
  }

  std::unique_ptr<CommandGuardPolicy> policy_;
  CommandGuardInput state_;
  std::string last_reason_;
  rclcpp::Time raw_command_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cloud_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time planner_heartbeat_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_header_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cloud_header_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr guarded_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::DataDisp>::SharedPtr
    planner_heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr route_ready_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace dddmr_scan_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dddmr_scan_planner::ScanCommandGuard>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("scan_command_guard"), "Initialization failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
