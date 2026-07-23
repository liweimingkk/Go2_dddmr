#ifndef DDDMR_SCAN_PLANNER__COMMAND_GUARD_POLICY_HPP_
#define DDDMR_SCAN_PLANNER__COMMAND_GUARD_POLICY_HPP_

#include <string>

namespace dddmr_scan_planner
{

struct Command
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CommandGuardLimits
{
  double max_x{0.40};
  double max_y{0.0};
  double max_yaw{0.50};
  double max_translation{0.40};
  double raw_command_timeout{0.15};
  double odom_timeout{0.25};
  double cloud_timeout{0.35};
  double planner_timeout{0.50};
  double odom_header_max_age{0.35};
  double cloud_header_max_age{0.35};
  double max_future_skew{0.05};
  double zero_epsilon{0.001};
};

struct CommandGuardInput
{
  bool route_ready{false};
  bool raw_command_received{false};
  bool odom_received{false};
  bool cloud_received{false};
  bool trajectory_received{false};
  bool planner_heartbeat_received{false};
  double raw_command_age{0.0};
  double odom_age{0.0};
  double cloud_age{0.0};
  double planner_heartbeat_age{0.0};
  double odom_header_age{0.0};
  double cloud_header_age{0.0};
  Command raw_command;
};

struct CommandGuardResult
{
  bool allowed{false};
  bool moving{false};
  Command command;
  std::string reason{"uninitialized"};
};

class CommandGuardPolicy
{
public:
  explicit CommandGuardPolicy(const CommandGuardLimits & limits);

  CommandGuardResult evaluate(const CommandGuardInput & input) const;

private:
  CommandGuardLimits limits_;
};

}  // namespace dddmr_scan_planner

#endif  // DDDMR_SCAN_PLANNER__COMMAND_GUARD_POLICY_HPP_
