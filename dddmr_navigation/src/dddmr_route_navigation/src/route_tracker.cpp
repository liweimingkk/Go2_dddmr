#include "dddmr_route_navigation/route_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dddmr_route_navigation
{
namespace
{

void setError(std::string * error, const std::string & value)
{
  if (error != nullptr) {
    *error = value;
  }
}

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

}  // namespace

RouteTracker::RouteTracker(RouteTrackerConfig config)
: config_(std::move(config))
{
}

bool RouteTracker::validateConfig(std::string * error) const
{
  const bool valid =
    std::isfinite(config_.duplicate_distance) && config_.duplicate_distance >= 0.0 &&
    std::isfinite(config_.resample_spacing) && config_.resample_spacing > 0.0 &&
    std::isfinite(config_.max_input_segment_length) &&
    config_.max_input_segment_length >= config_.resample_spacing &&
    std::isfinite(config_.progress_search_backward) &&
    config_.progress_search_backward >= 0.0 &&
    std::isfinite(config_.progress_search_forward) && config_.progress_search_forward > 0.0 &&
    std::isfinite(config_.local_plan_backward) && config_.local_plan_backward >= 0.0 &&
    std::isfinite(config_.local_plan_forward) && config_.local_plan_forward > 0.0 &&
    std::isfinite(config_.corridor_max_xy_error) && config_.corridor_max_xy_error > 0.0 &&
    std::isfinite(config_.corridor_max_z_error) && config_.corridor_max_z_error > 0.0 &&
    std::isfinite(config_.start_max_xy_error) && config_.start_max_xy_error > 0.0 &&
    std::isfinite(config_.start_max_z_error) && config_.start_max_z_error > 0.0 &&
    std::isfinite(config_.goal_max_xy_error) && config_.goal_max_xy_error > 0.0 &&
    std::isfinite(config_.goal_max_z_error) && config_.goal_max_z_error > 0.0;
  if (!valid) {
    setError(error, "route tracker distances must be finite and positive (duplicate/backward may be zero)");
  }
  return valid;
}

bool RouteTracker::finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
    !std::isfinite(q.w))
  {
    return false;
  }
  const double quaternion_norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return quaternion_norm_squared > 1.0e-12;
}

double RouteTracker::distance3d(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

geometry_msgs::msg::Pose RouteTracker::interpolatePose(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  double ratio)
{
  ratio = clamp01(ratio);
  geometry_msgs::msg::Pose output;
  output.position.x = start.position.x + ratio * (end.position.x - start.position.x);
  output.position.y = start.position.y + ratio * (end.position.y - start.position.y);
  output.position.z = start.position.z + ratio * (end.position.z - start.position.z);

  tf2::Quaternion start_q;
  tf2::Quaternion end_q;
  tf2::fromMsg(start.orientation, start_q);
  tf2::fromMsg(end.orientation, end_q);
  start_q.normalize();
  end_q.normalize();
  output.orientation = tf2::toMsg(start_q.slerp(end_q, ratio).normalized());
  return output;
}

bool RouteTracker::setRoute(const nav_msgs::msg::Path & route, std::string * error)
{
  route_.poses.clear();
  route_.header = route.header;
  arc_lengths_.clear();
  resetProgress();

  if (!validateConfig(error)) {
    return false;
  }
  if (route.header.frame_id.empty()) {
    setError(error, "route frame_id is empty");
    return false;
  }
  if (route.poses.size() < 3U) {
    setError(error, "route must contain at least three poses");
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> filtered;
  filtered.reserve(route.poses.size());
  for (std::size_t index = 0; index < route.poses.size(); ++index) {
    auto pose = route.poses[index];
    if (!pose.header.frame_id.empty() && pose.header.frame_id != route.header.frame_id) {
      std::ostringstream message;
      message << "pose " << index << " uses frame '" << pose.header.frame_id <<
        "' instead of route frame '" << route.header.frame_id << "'";
      setError(error, message.str());
      return false;
    }
    if (!finitePose(pose.pose)) {
      std::ostringstream message;
      message << "pose " << index << " contains a non-finite value or invalid quaternion";
      setError(error, message.str());
      return false;
    }
    pose.header.frame_id = route.header.frame_id;
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.pose.orientation, orientation);
    pose.pose.orientation = tf2::toMsg(orientation.normalized());

    if (!filtered.empty() &&
      distance3d(filtered.back().pose.position, pose.pose.position) <= config_.duplicate_distance)
    {
      filtered.back().pose.orientation = pose.pose.orientation;
      continue;
    }
    filtered.push_back(pose);
  }

  if (filtered.size() < 3U) {
    setError(error, "route has fewer than three distinct positions after duplicate removal");
    return false;
  }

  std::vector<double> input_arc_lengths(filtered.size(), 0.0);
  for (std::size_t index = 1; index < filtered.size(); ++index) {
    const double segment = distance3d(
      filtered[index - 1].pose.position, filtered[index].pose.position);
    if (segment > config_.max_input_segment_length) {
      std::ostringstream message;
      message << "route segment " << (index - 1U) << "->" << index << " is " << segment <<
        " m, exceeding max_input_segment_length=" << config_.max_input_segment_length;
      setError(error, message.str());
      return false;
    }
    input_arc_lengths[index] = input_arc_lengths[index - 1] + segment;
  }

  const double total_length = input_arc_lengths.back();
  if (total_length < 2.0 * config_.resample_spacing) {
    setError(error, "route is too short for three resampled positions");
    return false;
  }

  const auto append_at = [&](double target, nav_msgs::msg::Path & output) {
      const auto upper = std::upper_bound(input_arc_lengths.begin(), input_arc_lengths.end(), target);
      std::size_t end_index = static_cast<std::size_t>(
        std::distance(input_arc_lengths.begin(), upper));
      end_index = std::max<std::size_t>(1U, std::min(end_index, filtered.size() - 1U));
      const std::size_t start_index = end_index - 1U;
      const double segment_length = input_arc_lengths[end_index] - input_arc_lengths[start_index];
      const double ratio = segment_length > 0.0 ?
        (target - input_arc_lengths[start_index]) / segment_length : 0.0;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = route.header.frame_id;
      pose.pose = interpolatePose(filtered[start_index].pose, filtered[end_index].pose, ratio);
      output.poses.push_back(pose);
    };

  nav_msgs::msg::Path resampled;
  resampled.header = route.header;
  for (double target = 0.0; target < total_length; target += config_.resample_spacing) {
    append_at(target, resampled);
  }
  if (resampled.poses.empty() ||
    distance3d(resampled.poses.back().pose.position, filtered.back().pose.position) > 1.0e-9)
  {
    auto final_pose = filtered.back();
    final_pose.header.frame_id = route.header.frame_id;
    resampled.poses.push_back(final_pose);
  }

  if (resampled.poses.size() < 3U) {
    setError(error, "route resampling produced fewer than three poses");
    return false;
  }

  route_ = std::move(resampled);
  arc_lengths_.resize(route_.poses.size(), 0.0);
  for (std::size_t index = 1; index < route_.poses.size(); ++index) {
    arc_lengths_[index] = arc_lengths_[index - 1] + distance3d(
      route_.poses[index - 1].pose.position, route_.poses[index].pose.position);
  }
  setError(error, "");
  return true;
}

RouteProjection RouteTracker::project(
  const geometry_msgs::msg::Point & robot_position,
  double minimum_progress,
  double maximum_progress) const
{
  RouteProjection best;
  if (!ready()) {
    return best;
  }

  minimum_progress = std::max(0.0, minimum_progress);
  maximum_progress = std::min(length(), maximum_progress);
  if (maximum_progress < minimum_progress) {
    return best;
  }

  constexpr double replacement_epsilon = 1.0e-9;
  for (std::size_t index = 0; index + 1U < route_.poses.size(); ++index) {
    const double segment_start = arc_lengths_[index];
    const double segment_end = arc_lengths_[index + 1U];
    if (segment_end < minimum_progress || segment_start > maximum_progress) {
      continue;
    }

    const auto & start = route_.poses[index].pose.position;
    const auto & end = route_.poses[index + 1U].pose.position;
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double vz = end.z - start.z;
    const double segment_length_squared = vx * vx + vy * vy + vz * vz;
    if (segment_length_squared <= 0.0) {
      continue;
    }

    const double wx = robot_position.x - start.x;
    const double wy = robot_position.y - start.y;
    const double wz = robot_position.z - start.z;
    double ratio = clamp01((wx * vx + wy * vy + wz * vz) / segment_length_squared);
    double candidate_progress = segment_start + ratio * (segment_end - segment_start);
    candidate_progress = std::max(minimum_progress, std::min(maximum_progress, candidate_progress));
    ratio = (candidate_progress - segment_start) / (segment_end - segment_start);

    geometry_msgs::msg::Point projected;
    projected.x = start.x + ratio * vx;
    projected.y = start.y + ratio * vy;
    projected.z = start.z + ratio * vz;
    const double dx = robot_position.x - projected.x;
    const double dy = robot_position.y - projected.y;
    const double dz = robot_position.z - projected.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Strict comparison deliberately retains the earlier route segment when two
    // crossing segments are equally close, preventing a progress jump at a crossing.
    if (!best.valid || distance + replacement_epsilon < best.distance_3d) {
      best.valid = true;
      best.progress = candidate_progress;
      best.xy_error = std::hypot(dx, dy);
      best.z_error = std::abs(dz);
      best.distance_3d = distance;
      best.segment_index = index;
      best.point = projected;
    }
  }
  return best;
}

bool RouteTracker::initialize(
  const geometry_msgs::msg::Point & robot_position,
  bool allow_nearest_start,
  std::string * error)
{
  resetProgress();
  if (!ready()) {
    setError(error, "no valid route has been loaded");
    return false;
  }
  if (!std::isfinite(robot_position.x) || !std::isfinite(robot_position.y) ||
    !std::isfinite(robot_position.z))
  {
    setError(error, "robot position is not finite");
    return false;
  }

  RouteProjection start_projection;
  if (allow_nearest_start) {
    start_projection = project(robot_position, 0.0, length());
  } else {
    start_projection.valid = true;
    start_projection.point = route_.poses.front().pose.position;
    const double dx = robot_position.x - start_projection.point.x;
    const double dy = robot_position.y - start_projection.point.y;
    const double dz = robot_position.z - start_projection.point.z;
    start_projection.xy_error = std::hypot(dx, dy);
    start_projection.z_error = std::abs(dz);
    start_projection.distance_3d = std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  if (!start_projection.valid ||
    start_projection.xy_error > config_.start_max_xy_error ||
    start_projection.z_error > config_.start_max_z_error)
  {
    std::ostringstream message;
    message << "robot is outside the route start envelope: xy_error=" <<
      start_projection.xy_error << " (max " << config_.start_max_xy_error <<
      "), z_error=" << start_projection.z_error << " (max " <<
      config_.start_max_z_error << ")";
    setError(error, message.str());
    return false;
  }

  progress_ = allow_nearest_start ? start_projection.progress : 0.0;
  initialized_ = true;
  setError(error, "");
  return true;
}

RouteProjection RouteTracker::update(const geometry_msgs::msg::Point & robot_position)
{
  if (!initialized_) {
    return RouteProjection();
  }
  const RouteProjection projection = project(
    robot_position,
    progress_ - config_.progress_search_backward,
    progress_ + config_.progress_search_forward);
  if (projection.valid) {
    progress_ = std::max(progress_, projection.progress);
  }
  return projection;
}

void RouteTracker::resetProgress()
{
  progress_ = 0.0;
  initialized_ = false;
}

bool RouteTracker::ready() const
{
  return route_.poses.size() >= 3U && arc_lengths_.size() == route_.poses.size();
}

bool RouteTracker::initialized() const
{
  return initialized_;
}

bool RouteTracker::insideCorridor(const RouteProjection & projection) const
{
  return projection.valid &&
         projection.xy_error <= config_.corridor_max_xy_error &&
         projection.z_error <= config_.corridor_max_z_error;
}

bool RouteTracker::goalReached(const geometry_msgs::msg::Point & robot_position) const
{
  if (!ready()) {
    return false;
  }
  const auto & goal = route_.poses.back().pose.position;
  const double xy_error = std::hypot(robot_position.x - goal.x, robot_position.y - goal.y);
  const double z_error = std::abs(robot_position.z - goal.z);
  return xy_error <= config_.goal_max_xy_error && z_error <= config_.goal_max_z_error;
}

double RouteTracker::progress() const
{
  return progress_;
}

double RouteTracker::length() const
{
  return arc_lengths_.empty() ? 0.0 : arc_lengths_.back();
}

double RouteTracker::progressRatio() const
{
  return length() > 0.0 ? std::max(0.0, std::min(1.0, progress_ / length())) : 0.0;
}

const nav_msgs::msg::Path & RouteTracker::route() const
{
  return route_;
}

geometry_msgs::msg::PoseStamped RouteTracker::poseAt(double route_progress) const
{
  if (!ready()) {
    return geometry_msgs::msg::PoseStamped();
  }
  route_progress = std::max(0.0, std::min(length(), route_progress));
  const auto upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), route_progress);
  std::size_t end_index = static_cast<std::size_t>(std::distance(arc_lengths_.begin(), upper));
  if (end_index == 0U) {
    return route_.poses.front();
  }
  if (end_index >= route_.poses.size()) {
    return route_.poses.back();
  }
  const std::size_t start_index = end_index - 1U;
  const double segment_length = arc_lengths_[end_index] - arc_lengths_[start_index];
  const double ratio = segment_length > 0.0 ?
    (route_progress - arc_lengths_[start_index]) / segment_length : 0.0;
  geometry_msgs::msg::PoseStamped output;
  output.header.frame_id = route_.header.frame_id;
  output.pose = interpolatePose(route_.poses[start_index].pose, route_.poses[end_index].pose, ratio);
  return output;
}

nav_msgs::msg::Path RouteTracker::localPlan() const
{
  nav_msgs::msg::Path plan;
  plan.header = route_.header;
  if (!ready() || !initialized_) {
    return plan;
  }

  const double start_progress = std::max(0.0, progress_ - config_.local_plan_backward);
  const double end_progress = std::min(length(), progress_ + config_.local_plan_forward);
  plan.poses.push_back(poseAt(start_progress));

  auto first = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), start_progress);
  for (auto iterator = first; iterator != arc_lengths_.end() && *iterator < end_progress; ++iterator) {
    const std::size_t index = static_cast<std::size_t>(std::distance(arc_lengths_.begin(), iterator));
    plan.poses.push_back(route_.poses[index]);
  }
  plan.poses.push_back(poseAt(end_progress));

  while (plan.poses.size() < 3U) {
    plan.poses.insert(plan.poses.end() - 1, plan.poses.front());
  }
  return plan;
}

}  // namespace dddmr_route_navigation
