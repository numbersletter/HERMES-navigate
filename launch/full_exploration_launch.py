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
full_exploration_launch.py
==========================
Launches the complete HERMES autonomous exploration stack:
  - robot_state_publisher  — publishes /tf from the robot SDF description
  - slam_toolbox (online async)
  - Nav2 (full stack with custom exploration BT)
  - CoverageTrackerNode (lifecycle node)

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

    # ── Paths to config files ─────────────────────────────────────────────────
    sdf_xacro_file   = os.path.join(pkg_hermes_navigate, "urdf", "hermes_sim.sdf.xacro")
    nav2_params_file = os.path.join(pkg_hermes_navigate, "config", "nav2_params.yaml")
    slam_params_file = os.path.join(pkg_hermes_navigate, "config",
                                    "slam_toolbox_params.yaml")
    explore_bt_file  = os.path.join(pkg_hermes_navigate, "behavior_trees",
                                    "explore_and_cover.xml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock")

    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Robot description (process SDF xacro) ────────────────────────────────
    #   xacro expands the wheel macro then hands clean SDF to RSP.
    #   robot_state_publisher accepts SDF content via sdformat_urdf.
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

    # ── Nav2 Bringup (navigation2 full stack) ─────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time":            use_sim_time,
            "params_file":             nav2_params_file,
            "default_bt_xml_filename": explore_bt_file,
        }.items(),
    )

    # ── Coverage Tracker Node (lifecycle) ─────────────────────────────────────
    coverage_tracker = LifecycleNode(
        package="hermes_navigate",
        executable="coverage_tracker_node",
        name="coverage_tracker_node",
        namespace="",
        output="screen",
        parameters=[
            {
                "camera_fov_deg":     60.0,
                "camera_max_range_m": 5.0,
                "update_rate_hz":     10.0,
                "base_frame":         "base_link",
                "map_frame":          "map",
                "camera_yaw_offset":  0.0,
                "use_sim_time":       use_sim_time,
            }
        ],
    )

    # ── Lifecycle manager for coverage tracker ────────────────────────────────
    # Without a lifecycle manager the node stays in 'unconfigured' and never
    # creates its subscription, publisher, or timer.
    lifecycle_manager_coverage = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_coverage",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart":    True,
            "node_names":   ["coverage_tracker_node"],
        }],
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_log_level,

        # Robot description
        robot_state_pub,

        # SLAM
        slam_toolbox_node,

        # Nav2
        nav2_bringup,

        # Coverage tracker + its lifecycle manager
        coverage_tracker,
        lifecycle_manager_coverage,

        LogInfo(msg="HERMES full exploration stack launched."),
    ])

