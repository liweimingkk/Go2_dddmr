#include <exception>
#include <memory>

#include <local_planner/local_planner.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <p2p_move_base/p2p_global_plan_manager.h>
#include <p2p_move_base/p2p_move_base.h>
#include <perception_3d/perception_3d_ros.h>
#include <rclcpp/rclcpp.hpp>
#include <recovery_behaviors/recovery_behaviors_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>

#include "dddmr_route_navigation/recorded_route_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::MultiThreadedExecutor executor;
    auto trajectory_generators =
      std::make_shared<trajectory_generators::Trajectory_Generators_ROS>(
      "trajectory_generators");
    auto critics = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
    auto perception =
      std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
    auto recovery = std::make_shared<recovery_behaviors::Recovery_Behaviors_ROS>(
      "recovery_behaviors", perception, critics, trajectory_generators);
    auto local_planner = std::make_shared<local_planner::Local_Planner>("local_planner");
    auto global_plan_manager =
      std::make_shared<p2p_move_base::P2PGlobalPlanManager>("global_plan_manager");
    auto point_to_point = std::make_shared<p2p_move_base::P2PMoveBase>("p2p_move_base");
    auto recorded_route =
      std::make_shared<dddmr_route_navigation::RecordedRouteController>();

    executor.add_node(trajectory_generators);
    executor.add_node(critics);
    executor.add_node(perception);
    executor.add_node(recovery);
    executor.add_node(local_planner);
    executor.add_node(global_plan_manager);
    executor.add_node(point_to_point);
    executor.add_node(recorded_route);

    trajectory_generators->initial();
    critics->initial();
    perception->initial();
    recovery->initial();
    local_planner->initial(perception, critics, trajectory_generators);
    global_plan_manager->initial();
    point_to_point->initial(local_planner, global_plan_manager);
    recorded_route->initialize(local_planner);

    RCLCPP_WARN(
      rclcpp::get_logger("outdoor_indoor_move_base_node"),
      "Outdoor and indoor controllers share one local planner. Their outputs must pass "
      "through the mission command mux; both direct controller topics are unsafe outputs.");
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("outdoor_indoor_move_base_node"),
      "Outdoor/indoor stack initialization failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
