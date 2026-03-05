#!/usr/bin/env python3
# Copyright 2026 HERMES Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
hermes_navigate_launch.py
=========================
Main entry-point launch file for the HERMES autonomous exploration suite.

Launches:
  1. robot_state_publisher  — publishes /tf from the robot SDF description
  2. slam_toolbox (online async)  — simultaneous mapping + localisation
  3. Nav2 (full stack)            — local / global planning + costmaps
  4. hermes_navigate_node         — main lifecycle node (BT-driven exploration)
  5. coverage_tracker_node        — camera-coverage tracking (lifecycle)
  6. Lifecycle manager            — auto-configures and activates lifecycle nodes

Launch arguments
----------------
  use_sim_time  : use /clock topic (default: false)
  log_level     : ROS log level (default: info)
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_hermes_navigate = get_package_share_directory("hermes_navigate")
    pkg_nav2_bringup    = get_package_share_directory("nav2_bringup")

    # ── Paths to config / resource files ─────────────────────────────────────
    sdf_xacro_file   = os.path.join(pkg_hermes_navigate, "urdf", "hermes_sim.sdf.xacro")
    nav2_params_file = os.path.join(pkg_hermes_navigate, "config", "nav2_params.yaml")
    slam_params_file = os.path.join(pkg_hermes_navigate, "config",
                                    "slam_toolbox_params.yaml")
    plugin_params    = os.path.join(pkg_hermes_navigate, "params", "plugin_params.yaml")
    explore_bt_file  = os.path.join(pkg_hermes_navigate, "behavior_trees",
                                    "hermes_exploration_bt.xml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock (/clock topic)")

    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Robot description (process SDF xacro) ────────────────────────────────
    robot_description_content = xacro.process_file(sdf_xacro_file).toxml()

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time":      use_sim_time,
        }],
    )

    # ── SLAM Toolbox (online async) ───────────────────────────────────────────
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── Nav2 Bringup ──────────────────────────────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file":  nav2_params_file,
        }.items(),
    )

    # ── HermesNavigateNode (main lifecycle node) ──────────────────────────────
    hermes_navigate = LifecycleNode(
        package="hermes_navigate",
        executable="hermes_navigate_node",
        name="hermes_navigate_node",
        namespace="",
        output="screen",
        parameters=[
            plugin_params,
            {
                "bt_xml_file":  explore_bt_file,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # ── Coverage Tracker Node (lifecycle) ─────────────────────────────────────
    coverage_tracker = LifecycleNode(
        package="hermes_navigate",
        executable="coverage_tracker_node",
        name="coverage_tracker_node",
        namespace="",
        output="screen",
        parameters=[{
            "camera_fov_deg":     60.0,
            "camera_max_range_m": 5.0,
            "update_rate_hz":     10.0,
            "base_frame":         "base_link",
            "map_frame":          "map",
            "camera_yaw_offset":  0.0,
            "use_sim_time":       use_sim_time,
        }],
    )

    # ── Lifecycle manager ─────────────────────────────────────────────────────
    # Manages hermes_navigate_node and coverage_tracker_node.
    # autostart=True: automatically calls configure → activate on startup.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_hermes",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart":    True,
            "node_names": [
                "hermes_navigate_node",
                "coverage_tracker_node",
            ],
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_log_level,

        robot_state_pub,
        slam_toolbox_node,
        nav2_bringup,

        hermes_navigate,
        coverage_tracker,
        lifecycle_manager,

        LogInfo(msg="HERMES navigate stack launched."),
    ])
