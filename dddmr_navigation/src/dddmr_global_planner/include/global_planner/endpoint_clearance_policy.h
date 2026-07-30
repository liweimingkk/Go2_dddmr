#ifndef GLOBAL_PLANNER__ENDPOINT_CLEARANCE_POLICY_H_
#define GLOBAL_PLANNER__ENDPOINT_CLEARANCE_POLICY_H_

namespace global_planner
{

struct EndpointClearanceDecision
{
  bool traversable{false};
  bool uses_dynamic_start_exception{false};
};

inline EndpointClearanceDecision evaluateEndpointClearance(
  bool project_to_traversable_ground,
  bool is_start_endpoint,
  bool allow_start_in_dynamic_inflation,
  double aggregate_clearance,
  double inscribed_radius,
  bool static_clearance_available,
  double static_clearance)
{
  if (!project_to_traversable_ground || aggregate_clearance >= inscribed_radius) {
    return {true, false};
  }

  // A live obstacle beside the robot can place its current graph node inside
  // the dynamic inflation radius even though the saved map says that the
  // floor itself is safe.  Only the start may use this exception.  The goal
  // and every successor expanded by A* still require aggregate clearance.
  if (
    is_start_endpoint &&
    allow_start_in_dynamic_inflation &&
    static_clearance_available &&
    static_clearance >= inscribed_radius)
  {
    return {true, true};
  }

  return {false, false};
}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__ENDPOINT_CLEARANCE_POLICY_H_
