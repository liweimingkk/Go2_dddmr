cmake_minimum_required(VERSION 3.5)
project(dddmr_pg_map_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
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
find_package(tf2_sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL 1.15 REQUIRED QUIET)
find_package(OpenMP REQUIRED)

find_package(dddmr_sys_core REQUIRED)
find_package(dddmr_pcl REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
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
  tf2_sensor_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
  ${dddmr_sys_core_INCLUDE_DIRS}
  ${dddmr_pcl_INCLUDE_DIRS}
)

# Deterministic identity of the exact pose-graph artifacts.  This target has no
# ROS/PCL dependency so its integrity contract can be exercised by pure tests.
add_library(map_artifact_identity
  src/map_artifact_identity.cpp
)
target_include_directories(map_artifact_identity PUBLIC include)
set_target_properties(map_artifact_identity PROPERTIES POSITION_INDEPENDENT_CODE ON)

# library
add_library(dddmr_pg_map_server
  src/dddmr_pg_map_server.cpp
)
target_link_libraries(dddmr_pg_map_server
  map_artifact_identity
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  ${dddmr_pcl_LIBRARIES}
)
ament_target_dependencies(dddmr_pg_map_server
  ${dependencies}
)

#node
add_executable(dddmr_pg_map_server_node
  src/dddmr_pg_map_server_node.cpp
)
target_link_libraries(dddmr_pg_map_server_node 
  dddmr_pg_map_server
)
ament_target_dependencies(dddmr_pg_map_server_node
  ${dependencies}
)

install(TARGETS
  map_artifact_identity
  dddmr_pg_map_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install node
install(TARGETS
  dddmr_pg_map_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
 launch
 config
 rviz
 DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_roi_config
    test/test_terrain_roi_config.cpp
  )
  target_include_directories(test_terrain_roi_config PRIVATE include)

  ament_add_gtest(test_map_artifact_identity
    test/test_map_artifact_identity.cpp
  )
  target_link_libraries(test_map_artifact_identity map_artifact_identity)
endif()

ament_export_include_directories(include)
ament_export_libraries(dddmr_pg_map_server map_artifact_identity)
ament_export_dependencies(${dependencies})
ament_package()
