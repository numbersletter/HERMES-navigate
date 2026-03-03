# HERMES-navigate

Nav2-based autonomous exploration and camera-coverage navigation system for the HERMES platform.

## Overview

This package provides frontier-based autonomous exploration with camera-coverage-aware scoring, implemented as custom Nav2 Behavior Tree plugins for ROS 2 Jazzy.

## Simulation

A Gazebo Harmonic simulation environment is included so you can test the full exploration/navigation stack without physical hardware.

### Prerequisites

```bash
# Gazebo Harmonic + ROS 2 Jazzy integration
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
                 ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup \
                 ros-jazzy-xacro ros-jazzy-robot-state-publisher
```

### Launch the simulation

```bash
# Full simulation (Gazebo GUI + SLAM + Nav2 exploration)
ros2 launch hermes_navigate simulation_launch.py

# Headless (no GUI — useful for CI or SSH sessions)
ros2 launch hermes_navigate simulation_launch.py gz_gui:=false

# Custom world
ros2 launch hermes_navigate simulation_launch.py world:=/path/to/my_world.sdf
```

The launch file starts:
1. **Gazebo Harmonic** with `worlds/hermes_world.sdf` — a walled 10 m × 8 m indoor room with obstacles
2. **robot_state_publisher** — publishes TF from `urdf/hermes_sim.urdf.xacro`
3. **ros_gz_bridge** — bridges `/clock`, `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/joint_states`
4. **slam_toolbox** — online async SLAM (provides `map → odom` TF and `/map`)
5. **Nav2** — full stack with the HERMES custom exploration BT
6. **coverage_tracker_node** — lifecycle camera-coverage tracker

### Visualise with RViz

```bash
rviz2
```

Add displays for: `/map`, `/scan`, TF, Nav2 costmaps, and the HERMES BT status.

### Robot model

`urdf/hermes_sim.urdf.xacro` — differential-drive robot matching the real HERMES geometry:
- `base_link` cylinder, radius 0.18 m
- `laser_frame` at z = 0.18 m (360° LiDAR, 12 m range)
- `camera_frame` at x = 0.05 m, z = 0.20 m
- Wheel separation 0.30 m, wheel radius 0.05 m

### World layout

`worlds/hermes_world.sdf` — simple indoor test environment:
- 10 m × 8 m room with 2.5 m walls
- 2 column obstacles, 1 partial divider wall, 2 box obstacles

---

## Real-robot launch

### Full stack (SLAM + Nav2 + exploration)

```bash
ros2 launch hermes_navigate full_exploration_launch.py
```

### Nav2 only (SLAM assumed running)

```bash
ros2 launch hermes_navigate nav2_exploration_launch.py
```

## Part of the HERMES system

- [HERMES](https://github.com/bcchau1/HERMES) — main umbrella repo
- [HERMES-driver](https://github.com/numbersletter/HERMES-driver) — motor/base control
- [HERMES-bringup](https://github.com/numbersletter/HERMES-bringup) — LIDAR + odometry bringup
- [HERMES-percept](https://github.com/numbersletter/HERMES-percept) — camera + face detection