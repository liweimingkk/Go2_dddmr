#ifndef LOCAL_PLANNER__GOAL_TOLERANCE_H_
#define LOCAL_PLANNER__GOAL_TOLERANCE_H_

#include <cmath>

#include <perception_3d/terrain_model.h>

namespace local_planner
{

inline bool withinGoalTolerance(
  double dx, double dy, double dz,
  double xy_tolerance, double z_tolerance)
{
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
    !std::isfinite(xy_tolerance) || !std::isfinite(z_tolerance) ||
    xy_tolerance < 0.0 || z_tolerance < 0.0)
  {
    return false;
  }
  return std::hypot(dx, dy) <= xy_tolerance && std::fabs(dz) <= z_tolerance;
}

inline bool terrainGoalMatches(
  const perception_3d::TerrainNode * current,
  const perception_3d::TerrainNode * goal)
{
  using perception_3d::TerrainClass;
  if (current == nullptr || goal == nullptr) {
    return false;
  }
  const auto safe_support = [](const perception_3d::TerrainNode & node) {
      return node.terrain_class == TerrainClass::FLAT ||
             node.terrain_class == TerrainClass::RAMP ||
             node.terrain_class == TerrainClass::STAIR_TREAD;
    };
  if (!safe_support(*current) || !safe_support(*goal)) {
    return false;
  }
  const auto has_flag = [](const perception_3d::TerrainNode & node,
      perception_3d::TerrainNodeFlag flag) {
      return (node.flags & static_cast<std::uint32_t>(flag)) != 0U;
    };
  if (has_flag(*goal, perception_3d::TERRAIN_NODE_LANDING)) {
    if (!has_flag(*current, perception_3d::TERRAIN_NODE_LANDING) ||
      current->staircase_id < 0 || current->staircase_id != goal->staircase_id)
    {
      return false;
    }
    const bool goal_lower =
      has_flag(*goal, perception_3d::TERRAIN_NODE_LOWER_LANDING);
    const bool goal_upper =
      has_flag(*goal, perception_3d::TERRAIN_NODE_UPPER_LANDING);
    return goal_lower != goal_upper &&
           goal_lower == has_flag(*current, perception_3d::TERRAIN_NODE_LOWER_LANDING) &&
           goal_upper == has_flag(*current, perception_3d::TERRAIN_NODE_UPPER_LANDING);
  }
  if (goal->terrain_class == TerrainClass::STAIR_TREAD) {
    return current->terrain_class == TerrainClass::STAIR_TREAD &&
           current->staircase_id == goal->staircase_id &&
           current->step_index == goal->step_index;
  }
  return current->surface_id >= 0 && current->surface_id == goal->surface_id;
}

}  // namespace local_planner

#endif  // LOCAL_PLANNER__GOAL_TOLERANCE_H_
