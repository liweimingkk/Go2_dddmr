#ifndef LOCAL_PLANNER__PLAN_VALIDATION_H_
#define LOCAL_PLANNER__PLAN_VALIDATION_H_

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace local_planner
{

inline bool validGlobalPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const std::string & expected_frame,
  double maximum_segment_length,
  std::string * error = nullptr)
{
  const auto fail = [error](const char * message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
  if (plan.size() < 2U) {
    return fail("plan has fewer than two poses");
  }
  if (expected_frame.empty() || !std::isfinite(maximum_segment_length) ||
    maximum_segment_length <= 0.0)
  {
    return fail("plan validation configuration is invalid");
  }
  bool has_nonzero_segment = false;
  for (std::size_t index = 0U; index < plan.size(); ++index) {
    const auto & pose = plan[index];
    const auto & position = pose.pose.position;
    const auto & orientation = pose.pose.orientation;
    if (pose.header.frame_id != expected_frame) {
      return fail("plan frame does not match the local planner global frame");
    }
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return fail("plan contains a non-finite pose");
    }
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (quaternion_norm <= 1.0e-6) {
      return fail("plan contains an invalid orientation");
    }
    if (index == 0U) {
      continue;
    }
    const auto & previous = plan[index - 1U].pose.position;
    const double dx = position.x - previous.x;
    const double dy = position.y - previous.y;
    const double dz = position.z - previous.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(length) || length > maximum_segment_length) {
      return fail("plan contains a discontinuous segment");
    }
    has_nonzero_segment = has_nonzero_segment || length > 1.0e-6;
  }
  if (!has_nonzero_segment) {
    return fail("plan contains no motion segment");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace local_planner

#endif  // LOCAL_PLANNER__PLAN_VALIDATION_H_
