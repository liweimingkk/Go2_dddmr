#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "odom_sync_utils.h"

namespace
{

tf2::Quaternion makeYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return q;
}

}  // namespace

class Go2OdomStandardizer : public rclcpp::Node
{
public:
  Go2OdomStandardizer()
  : Node("go2_odom_standardizer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/utlidar/robot_odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/dddmr_go2/robot_odom_standard");
    output_parent_frame_ = declare_parameter<std::string>("output_parent_frame", "odom");
    output_child_frame_ = declare_parameter<std::string>("output_child_frame", "base_link");
    raw_to_standard_yaw_ = declare_parameter<double>("raw_to_standard_yaw", 0.0);
    stamp_time_offset_sec_ = declare_parameter<double>("stamp_time_offset_sec", 0.0);
    rotate_parent_frame_ = declare_parameter<bool>("rotate_parent_frame", false);
    rotate_twist_ = declare_parameter<bool>("rotate_twist", false);

    if (input_topic_ == output_topic_) {
      RCLCPP_FATAL(
        get_logger(),
        "go2_odom_standardizer refuses to run with identical input_topic and output_topic: %s",
        input_topic_.c_str());
      throw std::runtime_error("go2_odom_standardizer input_topic equals output_topic");
    }
    if (!std::isfinite(stamp_time_offset_sec_)) {
      RCLCPP_FATAL(get_logger(), "stamp_time_offset_sec must be finite");
      throw std::runtime_error("go2_odom_standardizer stamp_time_offset_sec is not finite");
    }

    raw_to_standard_ = makeYaw(raw_to_standard_yaw_);
    standard_to_raw_ = raw_to_standard_.inverse();

    if (std::abs(raw_to_standard_yaw_) > 1e-6) {
      RCLCPP_WARN(
        get_logger(),
        "raw_to_standard_yaw is nonzero. Do not use this for /utlidar/robot_odom "
        "unless an axis probe proves the raw odom frame is not standard.");
    }

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(20));
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Go2OdomStandardizer::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "standardizing odom %s -> %s, raw_to_standard_yaw=%.6f rad, "
      "stamp_time_offset_sec=%.9f, output %s -> %s",
      input_topic_.c_str(), output_topic_.c_str(), raw_to_standard_yaw_,
      stamp_time_offset_sec_,
      output_parent_frame_.c_str(), output_child_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr in)
  {
    nav_msgs::msg::Odometry out = *in;
    if (!lego_loam_bor::applyTimeOffset(
        in->header.stamp, stamp_time_offset_sec_, out.header.stamp)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject odometry: applying stamp_time_offset_sec=%.9f produced an invalid stamp",
        stamp_time_offset_sec_);
      return;
    }
    // Covariance is copied from the raw odometry and is not rotated here.
    // Validate or implement covariance rotation before using it downstream.
    out.header.frame_id = output_parent_frame_.empty() ? in->header.frame_id : output_parent_frame_;
    out.child_frame_id = output_child_frame_;

    tf2::Quaternion q_raw;
    tf2::fromMsg(in->pose.pose.orientation, q_raw);
    q_raw.normalize();

    tf2::Quaternion q_out = q_raw * raw_to_standard_;

    if (rotate_parent_frame_) {
      const tf2::Vector3 p_raw(
        in->pose.pose.position.x,
        in->pose.pose.position.y,
        in->pose.pose.position.z);
      const tf2::Vector3 p_out = tf2::quatRotate(standard_to_raw_, p_raw);
      out.pose.pose.position.x = p_out.x();
      out.pose.pose.position.y = p_out.y();
      out.pose.pose.position.z = p_out.z();

      q_out = standard_to_raw_ * q_out;
    }

    q_out.normalize();
    out.pose.pose.orientation = tf2::toMsg(q_out);

    if (rotate_twist_) {
      out.twist.twist.linear = rotateRawVectorToStandard(in->twist.twist.linear);
      out.twist.twist.angular = rotateRawVectorToStandard(in->twist.twist.angular);
    }

    if (!logged_first_) {
      logFirstSample(*in, out);
      logged_first_ = true;
    }

    pub_->publish(out);
  }

  geometry_msgs::msg::Vector3 rotateRawVectorToStandard(const geometry_msgs::msg::Vector3 & raw) const
  {
    const tf2::Vector3 raw_v(raw.x, raw.y, raw.z);
    const tf2::Vector3 out_v = tf2::quatRotate(standard_to_raw_, raw_v);
    geometry_msgs::msg::Vector3 out;
    out.x = out_v.x();
    out.y = out_v.y();
    out.z = out_v.z();
    return out;
  }

  void logFirstSample(const nav_msgs::msg::Odometry & in, const nav_msgs::msg::Odometry & out)
  {
    double raw_roll = 0.0;
    double raw_pitch = 0.0;
    double raw_yaw = 0.0;
    double out_roll = 0.0;
    double out_pitch = 0.0;
    double out_yaw = 0.0;

    tf2::Quaternion q_raw;
    tf2::fromMsg(in.pose.pose.orientation, q_raw);
    tf2::Matrix3x3(q_raw).getRPY(raw_roll, raw_pitch, raw_yaw);

    tf2::Quaternion q_out;
    tf2::fromMsg(out.pose.pose.orientation, q_out);
    tf2::Matrix3x3(q_out).getRPY(out_roll, out_pitch, out_yaw);

    RCLCPP_INFO(
      get_logger(),
      "first odom standardized: raw child=%s p=(%.3f, %.3f, %.3f) yaw=%.3f -> "
      "child=%s p=(%.3f, %.3f, %.3f) yaw=%.3f",
      in.child_frame_id.c_str(),
      in.pose.pose.position.x, in.pose.pose.position.y, in.pose.pose.position.z, raw_yaw,
      out.child_frame_id.c_str(),
      out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z, out_yaw);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_parent_frame_;
  std::string output_child_frame_;
  double raw_to_standard_yaw_;
  double stamp_time_offset_sec_;
  bool rotate_parent_frame_;
  bool rotate_twist_;
  bool logged_first_{false};
  tf2::Quaternion raw_to_standard_;
  tf2::Quaternion standard_to_raw_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<Go2OdomStandardizer>());
  } catch (const std::exception &) {
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
