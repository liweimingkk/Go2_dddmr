cmake_minimum_required(VERSION 3.8)
project(dddmr_route_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(local_planner REQUIRED)
find_package(perception_3d REQUIRED)
find_package(mpc_critics REQUIRED)
find_package(trajectory_generators REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(base_trajectory REQUIRED)
find_package(p2p_move_base REQUIRED)
find_package(recovery_behaviors REQUIRED)

set(route_dependencies
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  local_planner
  perception_3d
  mpc_critics
  trajectory_generators
  dddmr_sys_core
  base_trajectory
  p2p_move_base
  recovery_behaviors
)

add_library(route_tracker SHARED
  src/route_tracker.cpp
)
target_include_directories(route_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(route_tracker
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_library(recorded_route_controller SHARED
  src/recorded_route_controller.cpp
)
target_include_directories(recorded_route_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${dddmr_sys_core_INCLUDE_DIRS}
)
target_link_libraries(recorded_route_controller route_tracker)
ament_target_dependencies(recorded_route_controller ${route_dependencies})

add_executable(recorded_route_move_base_node
  src/recorded_route_move_base_node.cpp
)
target_link_libraries(recorded_route_move_base_node recorded_route_controller)
ament_target_dependencies(recorded_route_move_base_node ${route_dependencies})

add_executable(outdoor_indoor_move_base_node
  src/outdoor_indoor_move_base_node.cpp
)
target_link_libraries(outdoor_indoor_move_base_node
  recorded_route_controller
)
ament_target_dependencies(outdoor_indoor_move_base_node ${route_dependencies})

install(TARGETS
  route_tracker
  recorded_route_controller
  recorded_route_move_base_node
  outdoor_indoor_move_base_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/go2_mission_cmd_mux.py
  scripts/mission_command_mux_policy.py
  scripts/outdoor_indoor_mission.py
  scripts/outdoor_indoor_mission_client.py
  scripts/outdoor_indoor_mission_lib.py
  scripts/outdoor_indoor_mission_no_motion.py
  scripts/route_tool.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_route_tracker
    test/test_route_tracker.cpp
  )
  target_link_libraries(test_route_tracker route_tracker)
  ament_target_dependencies(test_route_tracker geometry_msgs nav_msgs)

  add_test(
    NAME test_route_tool
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_route_tool.py
  )
  add_test(
    NAME test_outdoor_indoor_mission_lib
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_outdoor_indoor_mission_lib.py
  )
  add_test(
    NAME test_mission_command_mux_policy
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_command_mux_policy.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(route_tracker recorded_route_controller)
ament_export_dependencies(${route_dependencies})
ament_package()
