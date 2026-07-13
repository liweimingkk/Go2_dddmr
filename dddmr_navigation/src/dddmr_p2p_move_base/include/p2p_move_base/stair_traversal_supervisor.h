#ifndef P2P_MOVE_BASE__STAIR_TRAVERSAL_SUPERVISOR_H_
#define P2P_MOVE_BASE__STAIR_TRAVERSAL_SUPERVISOR_H_

#include <cstddef>
#include <cstdint>
#include <string>

namespace p2p_move_base
{

enum class StairTraversalState : std::uint8_t
{
  NORMAL = 0,
  PRECHECK = 1,
  APPROACH = 2,
  ALIGN = 3,
  COMMITTED = 4,
  LANDING_VERIFY = 5,
  FAULT_LATCH = 6,
};

struct StairSupervisorConfig
{
  bool enabled{false};
  std::size_t confirmation_cycles{5};
  double min_confidence{0.90};
  double min_support_ratio{0.80};
  double max_heading_error_rad{0.13962634015954636};
  double max_lateral_error_m{0.10};
  double align_max_forward_mps{0.0};
  double align_max_yaw_rps{0.25};
  double committed_max_yaw_rps{0.0};
};

struct StairObservation
{
  bool fresh{false};
  bool gait_fresh{false};
  bool stair_candidate{false};
  bool entry_valid{false};
  bool at_entry{false};
  bool on_stair{false};
  bool landing_valid{false};
  bool full_body_on_landing{false};
  bool terrain_accepted{false};
  bool allow_forward{false};
  bool drop_detected{false};
  bool dynamic_obstacle{false};
  bool gait_unchanged{false};
  std::int32_t staircase_id{-1};
  std::int32_t step_index{-1};
  std::int32_t step_count{0};
  std::uint64_t snapshot_version{0};
  std::uint64_t static_ground_generation{0};
  double confidence{0.0};
  double support_ratio{0.0};
  double heading_error_rad{0.0};
  double lateral_error_m{0.0};
};

struct StairCommandDecision
{
  bool allowed{false};
  bool latch_fault{false};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string reason;
};

class StairTraversalSupervisor
{
public:
  void configure(const StairSupervisorConfig & config);
  StairTraversalState update(const StairObservation & observation);
  StairTraversalState refreshInputHealth(
    bool terrain_fresh, bool gait_fresh, bool gait_unchanged);
  StairCommandDecision filterCommand(double x, double y, double yaw) const;
  StairTraversalState latchExternalFault(const std::string & reason);
  void reset();
  bool resetFault(bool command_stopped, const StairObservation & observation);

  StairTraversalState state() const {return state_;}
  const std::string & faultReason() const {return fault_reason_;}
  bool inputReady() const {return input_ready_;}
  const std::string & inputHoldReason() const {return input_hold_reason_;}
  bool recoveryAllowed() const;
  bool replanningAllowed() const;
  bool faultLatched() const {return state_ == StairTraversalState::FAULT_LATCH;}

private:
  bool finiteObservation(const StairObservation & observation) const;
  bool geometryWithinLimits(const StairObservation & observation) const;
  StairTraversalState latchFault(const std::string & reason);

  StairSupervisorConfig config_;
  StairTraversalState state_{StairTraversalState::NORMAL};
  std::size_t confirmation_count_{0};
  std::int32_t committed_staircase_id_{-1};
  std::uint64_t committed_static_ground_generation_{0};
  bool committed_seen_stair_{false};
  bool input_ready_{false};
  std::string input_hold_reason_{"terrain_no_status"};
  std::string fault_reason_;
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__STAIR_TRAVERSAL_SUPERVISOR_H_
