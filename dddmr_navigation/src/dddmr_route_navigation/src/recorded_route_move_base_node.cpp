#include <memory>

#include <local_planner/local_planner.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <perception_3d/perception_3d_ros.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_generators/trajectory_generators_ros.h>

#include "dddmr_route_navigation/recorded_route_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::MultiThreadedExecutor executor;
    auto trajectory_generators =
      std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
    auto critics = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
    auto perception = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
    auto local_planner = std::make_shared<local_planner::Local_Planner>("local_planner");
    auto route_controller =
      std::make_shared<dddmr_route_navigation::RecordedRouteController>();

    executor.add_node(trajectory_generators);
    executor.add_node(critics);
    executor.add_node(perception);
    executor.add_node(local_planner);
    executor.add_node(route_controller);

    trajectory_generators->initial();
    critics->initial();
    perception->initial();
    local_planner->initial(perception, critics, trajectory_generators);
    route_controller->initialize(local_planner);

    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("recorded_route_move_base_node"),
      "Recorded-route stack initialization failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
