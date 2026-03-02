#!/usr/bin/env python3
# Copyright 2024 HERMES Team
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
  - slam_toolbox (online async)
  - Nav2 (full stack with custom exploration BT)
  - CoverageTrackerNode (lifecycle node)
  - Static TF publishers: base_link → laser_frame, base_link → camera_frame

Launch arguments
----------------
  laser_x, laser_y, laser_z          : laser frame offset from base_link (m)
  laser_roll, laser_pitch, laser_yaw : laser frame RPY offset (rad)
  camera_x, camera_y, camera_z       : camera frame offset from base_link (m)
  camera_roll, camera_pitch,          : camera frame RPY offset (rad)
  camera_yaw
  use_sim_time                        : use /clock topic (default: false)
  log_level                           : ROS log level (default: info)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_hermes_navigate = get_package_share_directory("hermes_navigate")
    pkg_nav2_bringup    = get_package_share_directory("nav2_bringup")
    pkg_slam_toolbox    = get_package_share_directory("slam_toolbox")

    # ── Paths to config files ─────────────────────────────────────────────────
    nav2_params_file    = os.path.join(pkg_hermes_navigate, "config", "nav2_params.yaml")
    slam_params_file    = os.path.join(pkg_hermes_navigate, "config",
                                       "slam_toolbox_params.yaml")
    explore_bt_file     = os.path.join(pkg_hermes_navigate, "behavior_trees",
                                       "explore_and_cover.xml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock")

    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level")

    # Laser static TF
    declare_laser_x   = DeclareLaunchArgument("laser_x",   default_value="0.0")
    declare_laser_y   = DeclareLaunchArgument("laser_y",   default_value="0.0")
    declare_laser_z   = DeclareLaunchArgument("laser_z",   default_value="0.18")
    declare_laser_roll  = DeclareLaunchArgument("laser_roll",  default_value="0.0")
    declare_laser_pitch = DeclareLaunchArgument("laser_pitch", default_value="0.0")
    declare_laser_yaw   = DeclareLaunchArgument("laser_yaw",   default_value="0.0")

    # Camera static TF
    declare_camera_x     = DeclareLaunchArgument("camera_x",     default_value="0.05")
    declare_camera_y     = DeclareLaunchArgument("camera_y",     default_value="0.0")
    declare_camera_z     = DeclareLaunchArgument("camera_z",     default_value="0.20")
    declare_camera_roll  = DeclareLaunchArgument("camera_roll",  default_value="0.0")
    declare_camera_pitch = DeclareLaunchArgument("camera_pitch", default_value="0.0")
    declare_camera_yaw   = DeclareLaunchArgument("camera_yaw",   default_value="0.0")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Static TF: base_link → laser_frame ───────────────────────────────────
    laser_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            LaunchConfiguration("laser_x"),
            LaunchConfiguration("laser_y"),
            LaunchConfiguration("laser_z"),
            LaunchConfiguration("laser_yaw"),
            LaunchConfiguration("laser_pitch"),
            LaunchConfiguration("laser_roll"),
            "base_link",
            "laser_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── Static TF: base_link → camera_frame ──────────────────────────────────
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            LaunchConfiguration("camera_x"),
            LaunchConfiguration("camera_y"),
            LaunchConfiguration("camera_z"),
            LaunchConfiguration("camera_yaw"),
            LaunchConfiguration("camera_pitch"),
            LaunchConfiguration("camera_roll"),
            "base_link",
            "camera_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
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
            "use_sim_time":    use_sim_time,
            "params_file":     nav2_params_file,
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

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_log_level,
        declare_laser_x, declare_laser_y, declare_laser_z,
        declare_laser_roll, declare_laser_pitch, declare_laser_yaw,
        declare_camera_x, declare_camera_y, declare_camera_z,
        declare_camera_roll, declare_camera_pitch, declare_camera_yaw,

        # Static TF publishers
        laser_tf_node,
        camera_tf_node,

        # SLAM
        slam_toolbox_node,

        # Nav2
        nav2_bringup,

        # Coverage tracker
        coverage_tracker,

        LogInfo(msg="HERMES full exploration stack launched."),
    ])
