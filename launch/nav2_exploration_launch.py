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
nav2_exploration_launch.py
===========================
Launches Nav2 + the HERMES exploration/coverage stack only.
Assumes SLAM Toolbox is already running and providing the map → odom TF
and the /map occupancy grid.

Launch arguments
----------------
  use_sim_time  : use /clock topic (default: false)
  params_file   : path to nav2_params.yaml
                  (default: hermes_navigate/config/nav2_params.yaml)
  bt_xml_file   : path to the exploration BT XML
                  (default: hermes_navigate/behavior_trees/explore_and_cover.xml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    pkg_hermes_navigate = get_package_share_directory("hermes_navigate")
    pkg_nav2_bringup    = get_package_share_directory("nav2_bringup")

    default_nav2_params = os.path.join(
        pkg_hermes_navigate, "config", "nav2_params.yaml")
    default_bt_xml = os.path.join(
        pkg_hermes_navigate, "behavior_trees", "explore_and_cover.xml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock")

    declare_params_file = DeclareLaunchArgument(
        "params_file", default_value=default_nav2_params,
        description="Full path to nav2_params.yaml")

    declare_bt_xml = DeclareLaunchArgument(
        "bt_xml_file", default_value=default_bt_xml,
        description="Full path to the exploration behavior tree XML")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file  = LaunchConfiguration("params_file")
    bt_xml_file  = LaunchConfiguration("bt_xml_file")

    # ── Nav2 stack ────────────────────────────────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time":              use_sim_time,
            "params_file":               params_file,
            "default_bt_xml_filename":   bt_xml_file,
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_bt_xml,

        nav2_bringup,
        coverage_tracker,

        LogInfo(msg="HERMES Nav2 exploration stack launched (SLAM assumed running)."),
    ])
