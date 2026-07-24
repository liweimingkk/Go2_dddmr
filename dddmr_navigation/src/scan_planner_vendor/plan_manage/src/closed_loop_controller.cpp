#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/final_goal_control.hpp"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    finish_yaw_tolerance_ = declare_parameter<double>("finish_yaw_tolerance", 0.15);
    if (
      !std::isfinite(finish_dist_) || finish_dist_ <= 0.0 ||
      !std::isfinite(finish_yaw_tolerance_) ||
      finish_yaw_tolerance_ <= 0.0 || finish_yaw_tolerance_ > M_PI)
    {
      throw std::invalid_argument(
              "finish_dist and finish_yaw_tolerance must be finite positive tolerances");
    }

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void logFinalPhase(
    const FinalGoalStatus & status, const double position_error)
  {
    if (status.phase == final_goal_phase_) {
      return;
    }
    final_goal_phase_ = status.phase;

    switch (status.phase) {
      case FinalGoalPhase::REACQUIRE_POSITION:
        RCLCPP_WARN(
          get_logger(),
          "Trajectory ended %.3f m from the goal; holding for a safe replan",
          position_error);
        break;
      case FinalGoalPhase::ALIGN_YAW:
        RCLCPP_INFO(
          get_logger(),
          "Final position reached; aligning yaw (error %.3f rad, tolerance %.3f rad)",
          status.yaw_error, finish_yaw_tolerance_);
        break;
      case FinalGoalPhase::COMPLETE:
        RCLCPP_INFO(
          get_logger(),
          "Goal pose reached (position error %.3f m, yaw error %.3f rad)",
          position_error, status.yaw_error);
        break;
      case FinalGoalPhase::INVALID:
        RCLCPP_ERROR(get_logger(), "Invalid final-goal state; commanding stop");
        break;
      case FinalGoalPhase::TRACKING:
        break;
    }
  }

  bool handleFinishedTrajectory(
    const Eigen::Vector3d & position_desired,
    const rclcpp::Time & current_time)
  {
    const Eigen::Vector2d position_error(
      position_desired.x() - odom_pos_.x(),
      position_desired.y() - odom_pos_.y());
    const FinalGoalStatus status = evaluateFinalGoal(
      exec_time_ >= traj_duration_,
      position_error.norm(),
      has_final_yaw_,
      final_yaw_,
      odom_yaw_,
      finish_dist_,
      finish_yaw_tolerance_);

    if (status.phase == FinalGoalPhase::TRACKING) {
      return false;
    }

    logFinalPhase(status, position_error.norm());
    last_update_time_ = current_time;
    if (status.phase == FinalGoalPhase::ALIGN_YAW) {
      publishExecutionFrozen(true);
      publishStop(kp_yaw_ * status.yaw_error);
    } else if (status.phase == FinalGoalPhase::INVALID) {
      publishExecutionFrozen(true);
      publishStop();
    } else {
      publishExecutionFrozen(false);
      publishStop();
    }
    return true;
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    has_final_yaw_ = !msg->yaw_pts.empty();
    if (has_final_yaw_) {
      const double terminal_yaw = msg->yaw_pts.back();
      if (!std::isfinite(terminal_yaw)) {
        receive_traj_ = false;
        publishExecutionFrozen(true);
        publishStop();
        RCLCPP_ERROR(
          get_logger(), "Trajectory %lld has a non-finite terminal yaw; commanding stop",
          static_cast<long long>(traj_id_));
        return;
      }
      final_yaw_ = normalizeAngle(terminal_yaw);
    }
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    final_goal_phase_ = FinalGoalPhase::TRACKING;
    if (has_final_yaw_) {
      RCLCPP_INFO(
        get_logger(), "Received trajectory %lld, duration %.3fs, final yaw %.3f rad",
        static_cast<long long>(traj_id_), traj_duration_, final_yaw_);
    } else {
      RCLCPP_INFO(
        get_logger(), "Received trajectory %lld, duration %.3fs, no final yaw",
        static_cast<long long>(traj_id_), traj_duration_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    if (handleFinishedTrajectory(pos_des, current_time)) {
      return;
    }
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    if (handleFinishedTrajectory(pos_des, current_time)) {
      return;
    }
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool has_final_yaw_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double final_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  FinalGoalPhase final_goal_phase_{FinalGoalPhase::TRACKING};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_, finish_yaw_tolerance_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
