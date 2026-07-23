#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace dddmr_scan_planner
{

class ScanInputAdapter : public rclcpp::Node
{
public:
  ScanInputAdapter()
  : Node("scan_input_adapter"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "hesai_lidar");
    input_cloud_frame_ = declare_parameter<std::string>("input_cloud_frame", "map");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    transform_max_age_sec_ =
      declare_parameter<double>("transform_max_age_sec", 0.25);
    transform_max_future_skew_sec_ =
      declare_parameter<double>("transform_max_future_skew_sec", 0.05);
    prefer_latest_transform_ =
      declare_parameter<bool>("prefer_latest_transform", true);
    allow_latest_transform_fallback_ =
      declare_parameter<bool>("allow_latest_transform_fallback", true);
    input_twist_in_body_frame_ =
      declare_parameter<bool>("input_twist_in_body_frame", true);

    if (
      map_frame_.empty() || body_frame_.empty() || sensor_frame_.empty() ||
      input_cloud_frame_.empty() || !std::isfinite(transform_timeout_sec_) ||
      transform_timeout_sec_ <= 0.0 || transform_timeout_sec_ > 0.20 ||
      !std::isfinite(transform_max_age_sec_) || transform_max_age_sec_ <= 0.0 ||
      !std::isfinite(transform_max_future_skew_sec_) ||
      transform_max_future_skew_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "SCAN input adapter requires valid frames and finite transform timing limits");
    }

    body_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS());
    sensor_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "sensor_pose", rclcpp::SensorDataQoS());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "input_odom", rclcpp::SensorDataQoS(),
      std::bind(&ScanInputAdapter::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", rclcpp::SensorDataQoS(),
      std::bind(&ScanInputAdapter::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "SCAN input adapter: body=%s sensor=%s cloud_frame=%s -> map=%s",
      body_frame_.c_str(), sensor_frame_.c_str(), input_cloud_frame_.c_str(),
      map_frame_.c_str());
  }

private:
  bool transformIsFresh(
    const geometry_msgs::msg::TransformStamped & transform,
    const builtin_interfaces::msg::Time & source_stamp)
  {
    const rclcpp::Time source_time(source_stamp, get_clock()->get_clock_type());
    const rclcpp::Time transform_time(
      transform.header.stamp, get_clock()->get_clock_type());
    const double age = (source_time - transform_time).seconds();
    if (
      std::isfinite(age) && age <= transform_max_age_sec_ &&
      age >= -transform_max_future_skew_sec_)
    {
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "SCAN input dropped: %s -> %s transform age %.3fs outside [-%.3f, %.3f]",
      transform.child_frame_id.c_str(), map_frame_.c_str(), age,
      transform_max_future_skew_sec_, transform_max_age_sec_);
    return false;
  }

  bool lookupCandidate(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & source_stamp,
    const bool latest,
    geometry_msgs::msg::TransformStamped & transform)
  {
    const rclcpp::Time lookup_time = latest ?
      rclcpp::Time(0, 0, get_clock()->get_clock_type()) :
      rclcpp::Time(source_stamp, get_clock()->get_clock_type());
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, lookup_time,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      return transformIsFresh(transform, source_stamp);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SCAN input dropped: %s %s -> %s transform unavailable: %s",
        latest ? "latest" : "timestamped", source_frame.c_str(),
        map_frame_.c_str(), error.what());
      return false;
    }
  }

  bool lookupTransform(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & source_stamp,
    geometry_msgs::msg::TransformStamped & transform)
  {
    if (prefer_latest_transform_) {
      return lookupCandidate(source_frame, source_stamp, true, transform);
    }

    if (lookupCandidate(source_frame, source_stamp, false, transform)) {
      return true;
    }
    return
      allow_latest_transform_fallback_ &&
      lookupCandidate(source_frame, source_stamp, true, transform);
  }

  static nav_msgs::msg::Odometry poseFromTransform(
    const geometry_msgs::msg::TransformStamped & transform,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & map_frame,
    const std::string & child_frame)
  {
    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp;
    output.header.frame_id = map_frame;
    output.child_frame_id = child_frame;
    output.pose.pose.position.x = transform.transform.translation.x;
    output.pose.pose.position.y = transform.transform.translation.y;
    output.pose.pose.position.z = transform.transform.translation.z;
    output.pose.pose.orientation = transform.transform.rotation;
    return output;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped map_to_body;
    if (!lookupTransform(body_frame_, msg->header.stamp, map_to_body)) {
      return;
    }

    nav_msgs::msg::Odometry output =
      poseFromTransform(map_to_body, msg->header.stamp, map_frame_, body_frame_);
    output.pose.covariance = msg->pose.covariance;
    output.twist.covariance = msg->twist.covariance;

    if (input_twist_in_body_frame_) {
      tf2::Quaternion orientation;
      tf2::fromMsg(map_to_body.transform.rotation, orientation);
      if (orientation.length2() < 1e-12) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SCAN body pose dropped: invalid map-to-body orientation");
        return;
      }
      orientation.normalize();
      const tf2::Vector3 linear_body(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
      const tf2::Vector3 angular_body(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z);
      const tf2::Vector3 linear_map = tf2::quatRotate(orientation, linear_body);
      const tf2::Vector3 angular_map = tf2::quatRotate(orientation, angular_body);
      output.twist.twist.linear.x = linear_map.x();
      output.twist.twist.linear.y = linear_map.y();
      output.twist.twist.linear.z = linear_map.z();
      output.twist.twist.angular.x = angular_map.x();
      output.twist.twist.angular.y = angular_map.y();
      output.twist.twist.angular.z = angular_map.z();
    } else {
      output.twist.twist = msg->twist.twist;
    }

    body_pose_pub_->publish(output);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != input_cloud_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SCAN cloud dropped: expected frame '%s', received '%s'",
        input_cloud_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_cloud;
    if (!lookupTransform(msg->header.frame_id, msg->header.stamp, map_to_cloud)) {
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_sensor;
    if (!lookupTransform(sensor_frame_, msg->header.stamp, map_to_sensor)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_in_map;
    try {
      tf2::doTransform(*msg, cloud_in_map, map_to_cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SCAN cloud dropped: failed to transform %s -> %s: %s",
        msg->header.frame_id.c_str(), map_frame_.c_str(), error.what());
      return;
    }
    cloud_in_map.header.stamp = msg->header.stamp;
    cloud_in_map.header.frame_id = map_frame_;

    const nav_msgs::msg::Odometry sensor_pose =
      poseFromTransform(map_to_sensor, msg->header.stamp, map_frame_, sensor_frame_);
    sensor_pose_pub_->publish(sensor_pose);
    cloud_pub_->publish(cloud_in_map);
  }

  std::string map_frame_;
  std::string body_frame_;
  std::string sensor_frame_;
  std::string input_cloud_frame_;
  double transform_timeout_sec_{0.05};
  double transform_max_age_sec_{0.25};
  double transform_max_future_skew_sec_{0.05};
  bool prefer_latest_transform_{true};
  bool allow_latest_transform_fallback_{true};
  bool input_twist_in_body_frame_{true};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sensor_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace dddmr_scan_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dddmr_scan_planner::ScanInputAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("scan_input_adapter"), "Initialization failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
