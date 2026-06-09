# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble + Gazebo Fortress simulation of an autonomous differential-drive robot (BB8-style) that explores an arena, detects a flag via semantic segmentation, and navigates to collect it. University course project (SSC0712 – USP São Carlos).

## Packages

| Package | Type | Purpose |
|---|---|---|
| `prm_2026` | Python (ament_python) | Simulation base: Gazebo world files, robot URDF, basic launch infrastructure |
| `bb8_control` | Python (ament_python) | Main autonomous control: FSM, explore_lite integration, Nav2, vision |
| `bb8_visual_slam` | Python (ament_python) | Visual SLAM alternative using RTAB-Map (experimental) |
| `bb8_slam` | C++ (ament_cmake) | LIDAR scan filter node: replaces infinite readings with 3.5 m max |

## Build & Setup

```bash
# Install ROS dependencies (includes explore_lite and topic_tools)
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-explore-lite ros-humble-topic-tools

# Build all packages
colcon build --symlink-install --packages-select prm_2026 bb8_slam bb8_control bb8_visual_slam

# Source the workspace (required in every new terminal)
source install/setup.bash
```

Build a single package: `colcon build --symlink-install --packages-select bb8_control`

## Running the System

### All-in-one (recommended)

```bash
ros2 launch bb8_control main_exploration.launch.py
# With debug logging:
ros2 launch bb8_control main_exploration.launch.py verbose:=true
# Different world:
ros2 launch bb8_control main_exploration.launch.py world:=arena_paredes.sdf
```

This starts the full stack in the correct boot order: Gazebo → robot spawn → SLAM → Nav2 → explore_lite → perception → FSM controller.

### Modular (for debugging individual layers)

```bash
# Terminal 1 – simulation + SLAM only
ros2 launch bb8_control sim_slam.launch.py

# Terminal 2 – Nav2
ros2 launch bb8_control nav2.launch.py

# Terminal 3 – explore_lite + FSM controller
ros2 launch bb8_control explore_control.launch.py

# Terminal 4 – vision/perception
ros2 launch bb8_control perception.launch.py
```

### Visual SLAM alternative (RTAB-Map)

```bash
ros2 launch bb8_visual_slam visual_slam.launch.py
ros2 launch bb8_control nav2.launch.py
ros2 launch bb8_control explore_control.launch.py
ros2 launch bb8_control perception.launch.py
```

Requires: `sudo apt install ros-humble-rtabmap-ros`

## Linting / Tests

```bash
colcon test --packages-select bb8_control
colcon test-result --verbose
```

Tests are ament linters only (flake8, pep257, copyright) — there are no functional unit tests.

## Architecture

### Control flow

```
Gazebo simulation
  ├─ /scan (LaserScan)
  ├─ /robot_cam/labels_map (Image)  ──► vision_processor.py
  │     └─ /vision/flag_detection (Pose2D)
  │     └─ /vision/flag_bearing   (Float32)
  ├─ /odom (Odometry)
  └─ /map (OccupancyGrid, from SLAM Toolbox)

explore_node (explore_lite, plain rclcpp::Node from m-explore-ros2)
  └─ sends NavigateToPose goals to Nav2 during EXPLORANDO state

controle_robo.py (ControleRoboFSM)
  ├─ pauses/resumes explore_node via /explore/resume (std_msgs/Bool)
  ├─ drives Nav2 ActionClient (NavigateToPose) for flag approach
  └─ publishes /diff_drive_base_controller/cmd_vel_unstamped

Nav2 stack (nav2.launch.py)
  └─ publishes /cmd_vel  ──► topic_tools relay ──► /diff_drive_base_controller/cmd_vel_unstamped
```

### FSM states (`controle_robo.py`)

- **EXPLORANDO** – explore_lite drives Nav2 autonomously. FSM only runs a watchdog: if the robot doesn't move ≥ 0.20 m for 12 s (frontier exhaustion), deactivate explore_lite, spin in place for 6 s, then reactivate.
- **BANDEIRA_DETECTADA** – flag confirmed by vision; explore_lite is deactivated via lifecycle `TRANSITION_DEACTIVATE`.
- **NAVIGANDO_PARA_BANDEIRA** – FSM sends `NavigateToPose` goals directly to Nav2 toward a point just in front of the flag.
- **PROCURANDO_BANDEIRA** – flag lost mid-nav; robot spins to reacquire. After 30 ticks (~6 s) with no flag, returns to EXPLORANDO.
- **POSICIONANDO_PARA_COLETA** – proportional bearing alignment; stops when bearing < 5° and announces victory.

### explore_lite control

`explore_node` (from local `src/m-explore-ros2/`) is a **plain `rclcpp::Node`**, not a lifecycle node. The FSM pauses and resumes it by publishing `std_msgs/Bool` to `/explore/resume`:
- `False` → `stop()` — cancels all Nav2 goals and halts the planning timer
- `True` → `resume()` — restarts the planning timer and picks the next frontier

### Key files

- `src/bb8_control/bb8_control/controle_robo.py` – main FSM node (explore_lite + Nav2 + vision)
- `src/bb8_control/bb8_control/vision_processor.py` – semantic segmentation → flag detection
- `src/bb8_control/config/explore.yaml` – explore_lite tuning (planner_frequency, potential_scale, min_frontier_size)
- `src/bb8_control/config/nav2_params.yaml` – Nav2 planner/controller tuning
- `src/bb8_control/config/slam_params.yaml` – SLAM Toolbox overrides
- `src/prm_2026/description/robot.urdf.xacro` – robot model
- `src/prm_2026/world/` – Gazebo SDF world files

### Topic mismatch: Nav2 ↔ robot

Nav2 writes to `/cmd_vel`; the Gazebo diff-drive controller reads `/diff_drive_base_controller/cmd_vel_unstamped`. `nav2.launch.py` bridges these with a `topic_tools/relay` node. `carrega_robo.launch.py` (from `prm_2026`) also spawns this relay, so the bridge is redundant but harmless when both are running.
