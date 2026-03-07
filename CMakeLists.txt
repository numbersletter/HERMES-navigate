cmake_minimum_required(VERSION 3.22)
project(hermes_navigate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to release build if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# ──────────────────────────────────────────────────────────────────────────────
# Shared dependency list re-used across multiple targets
# ──────────────────────────────────────────────────────────────────────────────
set(COMMON_DEPS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
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
  std_srvs
  visualization_msgs
  pluginlib
  ament_index_cpp
)

# ──────────────────────────────────────────────────────────────────────────────
# hermes_navigate_plugins — pluginlib shared library
#   Contains BaseFrontierSearch and BaseCostPlugin implementations:
#     WavefrontFrontierDetector, InformationGainPlugin, EuclideanPlugin
# ──────────────────────────────────────────────────────────────────────────────
add_library(hermes_navigate_plugins SHARED
  src/plugins/wavefront_frontier_detector.cpp
  src/plugins/information_gain_plugin.cpp
  src/plugins/euclidean_plugin.cpp
)
target_include_directories(hermes_navigate_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hermes_navigate_plugins ${COMMON_DEPS})
pluginlib_export_plugin_description_file(
  hermes_navigate plugins/hermes_navigate_plugins.xml)

# ──────────────────────────────────────────────────────────────────────────────
# hermes_navigate_bt_plugins — BT node shared library
#   Loaded at runtime by HermesNavigateNode.
#   Each node registers itself via its static registerWithFactory() method.
# ──────────────────────────────────────────────────────────────────────────────
add_library(hermes_navigate_bt_plugins SHARED
  # BT nodes
  src/bt_plugins/return_to_start_condition.cpp
  src/bt_plugins/search_frontiers_node.cpp
  src/bt_plugins/assign_costs_node.cpp
  src/bt_plugins/select_frontier_node.cpp
  src/bt_plugins/return_to_start_node.cpp
  src/bt_plugins/navigate_to_frontier_node.cpp
)
target_include_directories(hermes_navigate_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hermes_navigate_bt_plugins ${COMMON_DEPS})
target_link_libraries(hermes_navigate_bt_plugins hermes_navigate_plugins)

# ──────────────────────────────────────────────────────────────────────────────
# hermes_navigate_node — main lifecycle node executable
# ──────────────────────────────────────────────────────────────────────────────
add_executable(hermes_navigate_node
  src/hermes_navigate_node.cpp
)
target_include_directories(hermes_navigate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hermes_navigate_node ${COMMON_DEPS})
target_link_libraries(hermes_navigate_node
  hermes_navigate_bt_plugins
  hermes_navigate_plugins
)

# ──────────────────────────────────────────────────────────────────────────────
# coverage_tracker_node — standalone lifecycle node executable
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
  hermes_navigate_plugins
  hermes_navigate_bt_plugins
  hermes_navigate_node
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
  params
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
