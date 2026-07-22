#ifndef GLOBAL_PLANNER__DWA_PATH_STITCHER_H_
#define GLOBAL_PLANNER__DWA_PATH_STITCHER_H_

#include <cmath>
#include <cstddef>
#include <string>

#include <nav_msgs/msg/path.hpp>

namespace global_planner
{

// Join a freshly planned robot-to-pivot connector to the untouched remainder
// of the reference path.  A missing connector must never be treated as a
// valid path: doing so makes the result begin several metres in front of the
// robot and can drive the local planner into its deviation/recovery states.
inline bool stitchDwaPath(
  const nav_msgs::msg::Path & connector,
  const nav_msgs::msg::Path & reference,
  std::size_t pivot,
  double max_join_xy,
  double max_join_z,
  nav_msgs::msg::Path & output,
  std::string * failure_reason = nullptr)
{
  output = nav_msgs::msg::Path{};
  const auto fail = [&](const char * reason) {
      if (failure_reason != nullptr) {
        *failure_reason = reason;
      }
      return false;
    };

  if (!std::isfinite(max_join_xy) || max_join_xy <= 0.0 ||
    !std::isfinite(max_join_z) || max_join_z <= 0.0)
  {
    return fail("invalid_join_limits");
  }
  if (connector.poses.empty()) {
    return fail("empty_connector");
  }
  if (reference.poses.empty()) {
    return fail("empty_reference");
  }
  if (pivot >= reference.poses.size()) {
    return fail("pivot_out_of_range");
  }

  const auto & connector_end = connector.poses.back().pose.position;
  const auto & reference_pivot = reference.poses[pivot].pose.position;
  const double dx = connector_end.x - reference_pivot.x;
  const double dy = connector_end.y - reference_pivot.y;
  const double dz = connector_end.z - reference_pivot.z;
  const double join_xy = std::hypot(dx, dy);
  const double join_z = std::abs(dz);
  if (!std::isfinite(join_xy) || !std::isfinite(join_z)) {
    return fail("nonfinite_join");
  }
  if (join_xy > max_join_xy || join_z > max_join_z) {
    return fail("discontinuous_join");
  }

  output = connector;
  output.poses.reserve(
    connector.poses.size() + reference.poses.size() - pivot);
  for (std::size_t index = pivot; index < reference.poses.size(); ++index) {
    const auto & pose = reference.poses[index];
    if (index == pivot && join_xy <= 1e-6 && join_z <= 1e-6) {
      continue;
    }
    output.poses.push_back(pose);
  }
  if (failure_reason != nullptr) {
    failure_reason->clear();
  }
  return true;
}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__DWA_PATH_STITCHER_H_
