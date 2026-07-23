cmake_minimum_required(VERSION 3.8)
project(dddmr_scan_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

set(common_dependencies
  dddmr_sys_core
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  scan_planner_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

add_library(scan_command_guard_policy
  src/command_guard_policy.cpp
)
target_include_directories(scan_command_guard_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(scan_input_adapter_node
  src/scan_input_adapter_node.cpp
)
ament_target_dependencies(scan_input_adapter_node ${common_dependencies})

add_executable(scan_route_bridge_node
  src/scan_route_bridge_node.cpp
)
ament_target_dependencies(scan_route_bridge_node ${common_dependencies})

add_executable(scan_command_guard_node
  src/scan_command_guard_node.cpp
)
target_link_libraries(scan_command_guard_node scan_command_guard_policy)
ament_target_dependencies(scan_command_guard_node ${common_dependencies})

install(TARGETS
  scan_command_guard_policy
  scan_input_adapter_node
  scan_route_bridge_node
  scan_command_guard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_command_guard_policy
    test/test_command_guard_policy.cpp
  )
  target_link_libraries(test_command_guard_policy scan_command_guard_policy)
  target_include_directories(test_command_guard_policy PRIVATE include)

  add_test(
    NAME test_go2_xt16_scan_config
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scan_config.py
      ${CMAKE_CURRENT_SOURCE_DIR}/config/go2_xt16_scan_planner.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/launch/go2_xt16_scan_navigation.launch
      ${CMAKE_CURRENT_SOURCE_DIR}/../dddmr_beginner_guide/rviz/go2_xt16_navigation.rviz
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(scan_command_guard_policy)
ament_package()
