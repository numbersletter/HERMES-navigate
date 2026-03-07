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
simulation_launch.py
====================
Launches a complete Gazebo Harmonic simulation environment for testing
the HERMES autonomous exploration and navigation stack.

What is started
---------------
  1. Gazebo Harmonic (gz-sim) with a simple indoor test world
  2. robot_state_publisher  — publishes /tf from the robot SDF description
  3. ros_gz_bridge          — bridges ROS 2 ↔ Gazebo topics (YAML config)
  4. Gazebo entity spawner  — spawns the HERMES robot in Gazebo from SDF
  5. slam_toolbox           — online async SLAM (map → odom TF + /map)
  6. Nav2 full stack        — planner / controller / BT navigator
  7. hermes_navigate_node   — lifecycle frontier-exploration BT controller
  8. lifecycle_manager      — auto-configures and activates hermes_navigate_node

All nodes run with use_sim_time=true so they synchronise on /clock.

Launch arguments
----------------
  world    : SDF world file path
             (default: hermes_navigate/worlds/hermes_world.sdf)
  gz_gui   : launch the Gazebo GUI client (default: true)
             set to false for headless / CI runs
  log_level: ROS 2 log level (default: info)

Usage
-----
  ros2 launch hermes_navigate simulation_launch.py
  ros2 launch hermes_navigate simulation_launch.py gz_gui:=false
  ros2 launch hermes_navigate simulation_launch.py world:=/path/to/my.sdf
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
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_hermes_navigate = get_package_share_directory("hermes_navigate")
    pkg_ros_gz_sim      = get_package_share_directory("ros_gz_sim")
    pkg_nav2_bringup    = get_package_share_directory("nav2_bringup")

    # ── Paths to key files ────────────────────────────────────────────────────
    default_world   = os.path.join(pkg_hermes_navigate, "worlds",
                                   "hermes_world.sdf")
    sdf_xacro_file  = os.path.join(pkg_hermes_navigate, "urdf",
                                   "hermes_sim.sdf.xacro")
    bridge_params   = os.path.join(pkg_hermes_navigate, "config",
                                   "ros_gz_bridge.yaml")
    slam_params     = os.path.join(pkg_hermes_navigate, "config",
                                   "slam_toolbox_params.yaml")
    nav2_params     = os.path.join(pkg_hermes_navigate, "config",
                                   "sim_nav2_params.yaml")
    plugin_params   = os.path.join(pkg_hermes_navigate, "params",
                                   "plugin_params.yaml")
    explore_bt      = os.path.join(pkg_hermes_navigate, "behavior_trees",
                                   "hermes_exploration_bt.xml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        "world", default_value=default_world,
        description="Full path to the Gazebo SDF world file")

    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui", default_value="true",
        description="Launch the Gazebo GUI client (false for headless)")

    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level")

    world     = LaunchConfiguration("world")
    gz_gui    = LaunchConfiguration("gz_gui")

    # ── Robot description (process SDF xacro) ────────────────────────────────
    #   xacro expands the wheel macro then hands clean SDF to both RSP and the
    #   Gazebo spawner.  robot_state_publisher accepts SDF via sdformat_urdf.
    robot_description_content = xacro.process_file(sdf_xacro_file).toxml()

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────────
    #   gz_args:
    #     -r          : run immediately (don't pause at startup)
    #     -s          : server-only (no GUI) — added when gz_gui=false
    gz_args = PythonExpression([
        '"-r " + "' , world, '"'
        ' if "', gz_gui, '" == "true" else '
        '"-r -s " + "', world, '"'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args":         gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time":      True,
        }],
    )

    # ── 3. ros_gz_bridge ─────────────────────────────────────────────────────
    #   Bridges Gazebo ↔ ROS 2 topics needed by the HERMES stack.
    #   Topic mapping is configured via config/ros_gz_bridge.yaml.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[
            {"config_file": bridge_params},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ── 4. Spawn HERMES robot into Gazebo ─────────────────────────────────────
    #   Pass the already-processed SDF string directly via -string so no
    #   separate plain-SDF file needs to exist on disk.
    #   body_z_spawn = wheel_radius + body_size/2 = 0.033 + 0.0699 = 0.1029 m
    #   Spawned on the left (west) side of the room, x = -0.40, clear of the
    #   central divider wall which sits at x = 0.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_hermes",
        arguments=[
            "-name",   "hermes",
            "-string", robot_description_content,
            "-x",      "-0.40",
            "-y",      "0.0",
            "-z",      "0.1029",
            "-Y",      "0.0",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── 5. SLAM Toolbox (online async) ────────────────────────────────────────
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": True},
        ],
    )

    # ── 6. Nav2 (navigation stack) ────────────────────────────────────────────
    #   Nav2 handles NavigateToPose requests from hermes_navigate_node.
    #   It uses its built-in navigate_to_pose BT for each individual navigation
    #   action; high-level exploration is controlled by hermes_navigate_node.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file":  nav2_params,
        }.items(),
    )



    return LaunchDescription([
        # Arguments
        declare_world,
        declare_gz_gui,
        declare_log_level,

        LogInfo(msg="Starting HERMES simulation environment (Gazebo Harmonic) …"),

        # Infrastructure
        gz_sim,
        robot_state_pub,
        bridge,
        spawn_robot,

        # Autonomy stack

        LogInfo(msg="HERMES simulation ready — all nodes launched."),
    ])
