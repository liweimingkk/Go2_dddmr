#ifndef LEGO_LOAM_BOR_ODOM_SYNC_UTILS_H
#define LEGO_LOAM_BOR_ODOM_SYNC_UTILS_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lego_loam_bor
{

inline int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

inline builtin_interfaces::msg::Time nanosecondsToStamp(const int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  const int64_t clamped = std::max<int64_t>(nanoseconds, 0);
  stamp.sec = static_cast<int32_t>(clamped / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(clamped % 1000000000LL);
  return stamp;
}

inline bool applyTimeOffset(
  const builtin_interfaces::msg::Time & stamp,
  const double time_offset_sec,
  builtin_interfaces::msg::Time & corrected_stamp)
{
  if (!std::isfinite(time_offset_sec)) {
    return false;
  }
  const long double corrected_ns =
    static_cast<long double>(stampNanoseconds(stamp)) +
    static_cast<long double>(time_offset_sec) * 1000000000.0L;
  const long double max_stamp_ns =
    static_cast<long double>(std::numeric_limits<int32_t>::max()) * 1000000000.0L +
    999999999.0L;
  if (corrected_ns < 0.0L || corrected_ns > max_stamp_ns) {
    return false;
  }
  corrected_stamp = nanosecondsToStamp(
    static_cast<int64_t>(std::llround(corrected_ns)));
  return true;
}

inline bool interpolateOdometry(
  const nav_msgs::msg::Odometry & before,
  const nav_msgs::msg::Odometry & after,
  const builtin_interfaces::msg::Time & target_stamp,
  nav_msgs::msg::Odometry & output)
{
  if (before.header.frame_id != after.header.frame_id ||
      before.child_frame_id != after.child_frame_id) {
    return false;
  }

  const int64_t before_ns = stampNanoseconds(before.header.stamp);
  const int64_t after_ns = stampNanoseconds(after.header.stamp);
  const int64_t target_ns = stampNanoseconds(target_stamp);
  if (before_ns <= 0 || after_ns <= before_ns || target_ns < before_ns ||
      target_ns > after_ns) {
    return false;
  }

  const double ratio = std::clamp(
    static_cast<double>(target_ns - before_ns) /
      static_cast<double>(after_ns - before_ns),
    0.0, 1.0);
  const auto lerp = [ratio](const double a, const double b) {
      return a + ratio * (b - a);
    };

  output = before;
  output.header.stamp = target_stamp;
  output.pose.pose.position.x = lerp(before.pose.pose.position.x, after.pose.pose.position.x);
  output.pose.pose.position.y = lerp(before.pose.pose.position.y, after.pose.pose.position.y);
  output.pose.pose.position.z = lerp(before.pose.pose.position.z, after.pose.pose.position.z);

  tf2::Quaternion q_before;
  tf2::Quaternion q_after;
  tf2::fromMsg(before.pose.pose.orientation, q_before);
  tf2::fromMsg(after.pose.pose.orientation, q_after);
  if (q_before.length2() <= 1e-12 || q_after.length2() <= 1e-12) {
    return false;
  }
  q_before.normalize();
  q_after.normalize();
  tf2::Quaternion q_interpolated = q_before.slerp(q_after, ratio);
  q_interpolated.normalize();
  output.pose.pose.orientation = tf2::toMsg(q_interpolated);

  output.twist.twist.linear.x = lerp(before.twist.twist.linear.x, after.twist.twist.linear.x);
  output.twist.twist.linear.y = lerp(before.twist.twist.linear.y, after.twist.twist.linear.y);
  output.twist.twist.linear.z = lerp(before.twist.twist.linear.z, after.twist.twist.linear.z);
  output.twist.twist.angular.x = lerp(before.twist.twist.angular.x, after.twist.twist.angular.x);
  output.twist.twist.angular.y = lerp(before.twist.twist.angular.y, after.twist.twist.angular.y);
  output.twist.twist.angular.z = lerp(before.twist.twist.angular.z, after.twist.twist.angular.z);

  for (std::size_t i = 0; i < output.pose.covariance.size(); ++i) {
    output.pose.covariance[i] = lerp(before.pose.covariance[i], after.pose.covariance[i]);
    output.twist.covariance[i] = lerp(before.twist.covariance[i], after.twist.covariance[i]);
  }
  return true;
}

struct OdomSelectionResult
{
  nav_msgs::msg::Odometry odometry;
  bool valid = false;
  bool interpolated = false;
  double sync_error_sec = 0.0;
  double bracket_span_sec = 0.0;
  double oldest_corrected_stamp_sec = 0.0;
  double newest_corrected_stamp_sec = 0.0;
};

inline OdomSelectionResult selectTimeAlignedOdometry(
  const std::deque<nav_msgs::msg::Odometry> & buffer,
  const builtin_interfaces::msg::Time & target_stamp,
  const double time_offset_sec,
  const double tolerance_sec,
  const double interpolation_max_gap_sec,
  const bool allow_nearest)
{
  OdomSelectionResult result;
  if (buffer.empty()) {
    return result;
  }

  const int64_t target_ns = stampNanoseconds(target_stamp);
  const int64_t offset_ns = static_cast<int64_t>(std::llround(time_offset_sec * 1e9));
  const int64_t tolerance_ns = static_cast<int64_t>(
    std::llround(std::max(tolerance_sec, 0.0) * 1e9));
  const int64_t max_gap_ns = static_cast<int64_t>(
    std::llround(std::max(interpolation_max_gap_sec, 0.0) * 1e9));
  const auto corrected_stamp_ns = [offset_ns](const nav_msgs::msg::Odometry & odom) {
      return stampNanoseconds(odom.header.stamp) + offset_ns;
    };

  result.oldest_corrected_stamp_sec =
    static_cast<double>(corrected_stamp_ns(buffer.front())) * 1e-9;
  result.newest_corrected_stamp_sec =
    static_cast<double>(corrected_stamp_ns(buffer.back())) * 1e-9;

  const auto after_it = std::lower_bound(
    buffer.begin(), buffer.end(), target_ns,
    [&corrected_stamp_ns](const nav_msgs::msg::Odometry & odom, const int64_t stamp) {
      return corrected_stamp_ns(odom) < stamp;
    });

  if (after_it != buffer.end() && corrected_stamp_ns(*after_it) == target_ns) {
    result.odometry = *after_it;
    result.odometry.header.stamp = target_stamp;
    result.valid = true;
    return result;
  }

  if (after_it != buffer.begin() && after_it != buffer.end()) {
    const auto before_it = std::prev(after_it);
    const int64_t before_ns = corrected_stamp_ns(*before_it);
    const int64_t after_ns = corrected_stamp_ns(*after_it);
    const int64_t bracket_span_ns = after_ns - before_ns;
    const int64_t nearest_error_ns = std::min(
      target_ns - before_ns, after_ns - target_ns);
    const int64_t furthest_error_ns = std::max(
      target_ns - before_ns, after_ns - target_ns);
    if (bracket_span_ns > 0 && bracket_span_ns <= max_gap_ns &&
        furthest_error_ns <= tolerance_ns) {
      nav_msgs::msg::Odometry corrected_before = *before_it;
      nav_msgs::msg::Odometry corrected_after = *after_it;
      corrected_before.header.stamp = nanosecondsToStamp(before_ns);
      corrected_after.header.stamp = nanosecondsToStamp(after_ns);
      if (interpolateOdometry(
          corrected_before, corrected_after, target_stamp, result.odometry)) {
        result.valid = true;
        result.interpolated = true;
        result.sync_error_sec = static_cast<double>(nearest_error_ns) * 1e-9;
        result.bracket_span_sec = static_cast<double>(bracket_span_ns) * 1e-9;
        return result;
      }
      // A frame-id change or invalid quaternion makes the bracket unsafe.
      return result;
    }
  }

  if (!allow_nearest) {
    return result;
  }

  const nav_msgs::msg::Odometry * nearest = nullptr;
  int64_t nearest_error_ns = std::numeric_limits<int64_t>::max();
  if (after_it != buffer.end()) {
    nearest = &(*after_it);
    nearest_error_ns = std::abs(corrected_stamp_ns(*after_it) - target_ns);
  }
  if (after_it != buffer.begin()) {
    const auto before_it = std::prev(after_it);
    const int64_t before_error_ns = std::abs(corrected_stamp_ns(*before_it) - target_ns);
    if (before_error_ns < nearest_error_ns) {
      nearest = &(*before_it);
      nearest_error_ns = before_error_ns;
    }
  }
  if (nearest != nullptr && nearest_error_ns <= tolerance_ns) {
    result.odometry = *nearest;
    result.odometry.header.stamp = target_stamp;
    result.valid = true;
    result.sync_error_sec = static_cast<double>(nearest_error_ns) * 1e-9;
  }
  return result;
}

}  // namespace lego_loam_bor

#endif  // LEGO_LOAM_BOR_ODOM_SYNC_UTILS_H
