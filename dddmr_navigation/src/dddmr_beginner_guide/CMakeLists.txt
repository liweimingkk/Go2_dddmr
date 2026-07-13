cmake_minimum_required(VERSION 3.5)
project(dddmr_beginner_guide)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dddmr_sys_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  pluginlib
  dddmr_sys_core
)

include_directories(
  include
)


#[[
install(TARGETS
  parameters
  lidar_measurement_model_likelihood
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
#]]
#install node

install(PROGRAMS
  scripts/audit_go2_navigation_request_echo.py
  scripts/go2_gait_monitor_policy.py
  scripts/go2_gait_state_monitor.py
  scripts/go2_nav_cmd_gate.py
  scripts/go2_nav_gate_policy.py
  scripts/go2_sport_cmd_vel_dry_run.py
  scripts/go2_sport_cmd_vel_adapter.py
  scripts/go2_sport_request_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_go2_nav_gate_policy
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_go2_nav_gate_policy.py
  )
  add_test(
    NAME test_go2_gait_monitor_policy
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_go2_gait_monitor_policy.py
  )
  add_test(
    NAME test_go2_sport_request_policy
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_go2_sport_request_policy.py
  )
  add_test(
    NAME test_go2_xt16_terrain_site_profile
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_go2_xt16_terrain_site_profile.py
  )
endif()

install(DIRECTORY
 "launch"
 "config"
 "rviz"
 DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
