#include "dddmr_scan_planner/command_guard_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dddmr_scan_planner
{
namespace
{

bool finiteNonnegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool receiptIsFresh(const bool received, const double age, const double timeout)
{
  return received && finiteNonnegative(age) && age <= timeout;
}

bool headerIsFresh(
  const double age, const double maximum_age, const double maximum_future_skew)
{
  return std::isfinite(age) && age <= maximum_age && age >= -maximum_future_skew;
}

}  // namespace

CommandGuardPolicy::CommandGuardPolicy(const CommandGuardLimits & limits)
: limits_(limits)
{
  const bool valid =
    finiteNonnegative(limits_.max_x) &&
    finiteNonnegative(limits_.max_y) &&
    finiteNonnegative(limits_.max_yaw) &&
    std::isfinite(limits_.max_translation) && limits_.max_translation > 0.0 &&
    std::isfinite(limits_.raw_command_timeout) && limits_.raw_command_timeout > 0.0 &&
    std::isfinite(limits_.odom_timeout) && limits_.odom_timeout > 0.0 &&
    std::isfinite(limits_.cloud_timeout) && limits_.cloud_timeout > 0.0 &&
    std::isfinite(limits_.planner_timeout) && limits_.planner_timeout > 0.0 &&
    std::isfinite(limits_.odom_header_max_age) && limits_.odom_header_max_age > 0.0 &&
    std::isfinite(limits_.cloud_header_max_age) && limits_.cloud_header_max_age > 0.0 &&
    finiteNonnegative(limits_.max_future_skew) &&
    std::isfinite(limits_.zero_epsilon) && limits_.zero_epsilon > 0.0;
  if (!valid) {
    throw std::invalid_argument("SCAN command guard limits must be finite and safe");
  }
}

CommandGuardResult CommandGuardPolicy::evaluate(const CommandGuardInput & input) const
{
  CommandGuardResult result;

  if (!input.route_ready) {
    result.reason = "route_not_ready";
    return result;
  }
  if (!receiptIsFresh(
      input.raw_command_received, input.raw_command_age, limits_.raw_command_timeout))
  {
    result.reason = "raw_command_stale";
    return result;
  }
  if (!receiptIsFresh(input.odom_received, input.odom_age, limits_.odom_timeout)) {
    result.reason = "odom_stale";
    return result;
  }
  if (!receiptIsFresh(input.cloud_received, input.cloud_age, limits_.cloud_timeout)) {
    result.reason = "cloud_stale";
    return result;
  }
  if (!receiptIsFresh(
      input.planner_heartbeat_received, input.planner_heartbeat_age,
      limits_.planner_timeout))
  {
    result.reason = "planner_stale";
    return result;
  }
  if (!input.trajectory_received) {
    result.reason = "trajectory_missing";
    return result;
  }
  if (!headerIsFresh(
      input.odom_header_age, limits_.odom_header_max_age, limits_.max_future_skew))
  {
    result.reason = "odom_header_stale";
    return result;
  }
  if (!headerIsFresh(
      input.cloud_header_age, limits_.cloud_header_max_age, limits_.max_future_skew))
  {
    result.reason = "cloud_header_stale";
    return result;
  }
  if (
    !std::isfinite(input.raw_command.x) ||
    !std::isfinite(input.raw_command.y) ||
    !std::isfinite(input.raw_command.yaw))
  {
    result.reason = "nonfinite_command";
    return result;
  }

  result.command.x =
    std::clamp(input.raw_command.x, -limits_.max_x, limits_.max_x);
  result.command.y =
    std::clamp(input.raw_command.y, -limits_.max_y, limits_.max_y);
  result.command.yaw =
    std::clamp(input.raw_command.yaw, -limits_.max_yaw, limits_.max_yaw);

  const double translation = std::hypot(result.command.x, result.command.y);
  if (translation > limits_.max_translation) {
    const double scale = limits_.max_translation / translation;
    result.command.x *= scale;
    result.command.y *= scale;
  }

  result.allowed = true;
  result.moving =
    std::abs(result.command.x) > limits_.zero_epsilon ||
    std::abs(result.command.y) > limits_.zero_epsilon ||
    std::abs(result.command.yaw) > limits_.zero_epsilon;
  result.reason = "ok";
  return result;
}

}  // namespace dddmr_scan_planner
