cmake_minimum_required(VERSION 3.22)
project(hermes_navigate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to release build if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# ──────────────────────────────────────────────────────────────────────────────
# Shared dependency list for BT plugin targets
# ──────────────────────────────────────────────────────────────────────────────
set(BT_PLUGIN_DEPS
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  nav2_costmap_2d
  nav_msgs
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  pluginlib
)

# ──────────────────────────────────────────────────────────────────────────────
# BT Plugin shared library
# All five BT nodes compiled into a single plugin .so that Nav2 BT Navigator
# loads at runtime via pluginlib / class_loader.
# ──────────────────────────────────────────────────────────────────────────────
add_library(hermes_navigate_bt_plugins SHARED
  src/compute_frontiers.cpp
  src/score_frontiers.cpp
  src/select_next_frontier.cpp
  src/should_do_coverage.cpp
  src/select_coverage_waypoint.cpp
  src/bt_node_registration.cpp
)
target_include_directories(hermes_navigate_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hermes_navigate_bt_plugins ${BT_PLUGIN_DEPS})
# Mark the library so pluginlib knows it's a plugin
pluginlib_export_plugin_description_file(nav2_behavior_tree plugins/hermes_navigate_bt_plugins.xml)

# ──────────────────────────────────────────────────────────────────────────────
# Standalone coverage tracker node executable
# ──────────────────────────────────────────────────────────────────────────────
add_executable(coverage_tracker_node
  src/coverage_tracker_node.cpp
)
target_include_directories(coverage_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(coverage_tracker_node
  rclcpp
  rclcpp_lifecycle
  nav2_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  nav2_costmap_2d
)

# ──────────────────────────────────────────────────────────────────────────────
# Install targets
# ──────────────────────────────────────────────────────────────────────────────
install(TARGETS
  hermes_navigate_bt_plugins
  coverage_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  behavior_trees
  launch
  plugins
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# ──────────────────────────────────────────────────────────────────────────────
# Testing
# ──────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
