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
                 ros-jazzy-sdformat-urdf ros-jazzy-robot-state-publisher
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
1. **Gazebo Harmonic** with `worlds/hermes_world.sdf` — a walled 4 ft × 4 ft indoor room (inner) with obstacles
2. **robot_state_publisher** — publishes TF from `urdf/hermes_sim.sdf` (parsed via `sdformat_urdf`)
3. **ros_gz_bridge** — bridges `/clock`, `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/joint_states` (configured via `config/ros_gz_bridge.yaml`)
4. **slam_toolbox** — online async SLAM (provides `map → odom` TF and `/map`)
5. **Nav2** — full stack with the HERMES custom exploration BT
6. **coverage_tracker_node** — lifecycle camera-coverage tracker

### Visualise with RViz

```bash
rviz2
```

Add displays for: `/map`, `/scan`, TF, Nav2 costmaps, and the HERMES BT status.

### Robot model

`urdf/hermes_sim.sdf` — differential-drive robot in SDF format matching the real HERMES geometry:

| Part | Dimension |
|---|---|
| Body | cube, 5.5 × 5.5 × 5.5 in (0.1397 m per side) |
| Drive wheels | diameter 2.6 in (r = 0.033 m), width 0.7 in (0.018 m) |
| Wheel separation | 0.1575 m centre-to-centre |
| Caster | sphere, r = 0.0165 m, at **front** of body |
| LiDAR (`laser_frame`) | 360°, 12 m range, on top of body (~0.173 m AGL) |
| Camera (`camera_frame`) | at front-top corner of body |

**TF tree:**
```
odom
└── base_link
      ├── base_footprint  (virtual ground plane, child of base_link)
      ├── wheel_left_link
      ├── wheel_right_link
      ├── caster_link     (front)
      ├── laser_frame
      └── camera_frame
```

Both drive wheels share identical geometry parameterized by a `y_reflect` sign:
- Left wheel:  `y_reflect = +1` → `y = +0.0788 m`
- Right wheel: `y_reflect = -1` → `y = -0.0788 m`

### Bridge configuration

`config/ros_gz_bridge.yaml` — declarative YAML config for `ros_gz_bridge`:

| ROS 2 topic | Direction | Type |
|---|---|---|
| `/clock` | GZ → ROS | `rosgraph_msgs/msg/Clock` |
| `/cmd_vel` | ROS → GZ | `geometry_msgs/msg/Twist` |
| `/odom` | GZ → ROS | `nav_msgs/msg/Odometry` |
| `/scan` | GZ → ROS | `sensor_msgs/msg/LaserScan` |
| `/tf` | GZ → ROS | `tf2_msgs/msg/TFMessage` |
| `/joint_states` | GZ → ROS | `sensor_msgs/msg/JointState` |

### World layout

`worlds/hermes_world.sdf` — simple indoor test environment:
- **4 ft × 4 ft** inner room (1.2192 m × 1.2192 m) with **7.5 in** (0.191 m) walls
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
