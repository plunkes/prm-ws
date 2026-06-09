# BB8 — Autonomous Exploration & Flag Capture

**SSC0712 — Programação de Robôs Móveis · USP São Carlos**

Autonomous differential-drive robot (BB8-style) built with **ROS 2 Humble** and **Gazebo Fortress**. The robot explores an arena using frontier-based exploration, detects a target flag via semantic segmentation, and navigates to capture it.

---

## Packages

| Package | Purpose |
|---|---|
| [`bb8_control`](src/bb8_control/) | **Main package** — FSM controller, Nav2 integration, explore_lite, vision |
| [`prm_2026`](#prm_2026) | Simulation base: Gazebo world files, robot URDF/Xacro, bridge launch |
| [`m-explore-ros2`](#m-explore-ros2) | Frontier exploration node (`explore_lite`) — modified fork |
| `bb8_slam` | LIDAR scan filter (C++) — replaces infinite readings with 3.5 m max |
| `bb8_visual_slam` | Visual SLAM alternative using RTAB-Map (experimental) |

### prm_2026

Simulation infrastructure package. Contains the robot URDF, Gazebo world SDF files, and the `carrega_robo.launch.py` entry point that spawns the robot and bridges Gazebo topics to ROS 2.

<!-- Link: https://github.com/... -->

### m-explore-ros2

Frontier-based exploration node. This is a **modified fork** — the upstream node gained:
- Pause/resume via `std_msgs/Bool` on `/explore/resume` (the FSM relies on this)
- Fixed goal blacklisting: preempted goals (error_code=0) are no longer blacklisted
- `ExploreStatus` publisher on `/explore/status`

<!-- Link: https://github.com/... -->

---

## Quick Start

### Install dependencies

```bash
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-topic-tools \
  ros-humble-explore-lite
```

### Build

```bash
cd ~/prm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select prm_2026 bb8_slam bb8_control explore_lite_msgs explore_lite
source install/setup.bash
```

Build a single package: `colcon build --symlink-install --packages-select bb8_control`

### Launch the full stack

```bash
ros2 launch bb8_control main_exploration.launch.py
# With DEBUG logging:
ros2 launch bb8_control main_exploration.launch.py verbose:=true
# Different world:
ros2 launch bb8_control main_exploration.launch.py world:=arena_paredes.sdf
```

This starts the full stack in boot order: Gazebo → robot spawn → SLAM → Nav2 → explore_lite → perception → FSM.

---

## Architecture

### Control flow

```
Gazebo simulation
  ├─ /scan (LaserScan)
  ├─ /robot_cam/labels_map (Image)  ──► vision_processor.py
  │     └─ /vision/flag_detection (Pose2D)    label 25 = blue_flag (target)
  │     └─ /vision/flag_bearing   (Float32)
  ├─ /odom (Odometry, from odom_gt_publisher)
  └─ /map (OccupancyGrid, from SLAM Toolbox)

explore_node (from m-explore-ros2, paused/resumed via /explore/resume)
  └─ sends NavigateToPose goals to Nav2 during EXPLORANDO state

controle_robo.py (ControleRoboFSM)
  ├─ pauses/resumes explore_node via /explore/resume (std_msgs/Bool)
  ├─ drives Nav2 ActionClient (NavigateToPose) for flag approach
  └─ publishes /diff_drive_base_controller/cmd_vel_unstamped

Nav2 stack (nav2.launch.py)
  └─ publishes /cmd_vel  ──► topic_tools relay ──► /diff_drive_base_controller/cmd_vel_unstamped
```

### FSM states

```
┌─────────────────────────────────────────────────────────────────┐
│                         EXPLORANDO                              │
│  explore_lite ACTIVE → sends NavigateToPose goals to Nav2       │
│  Watchdog: if no movement ≥ 0.20 m for 12 s →                   │
│    1. Deactivate explore_lite + clear local costmap              │
│    2. Back up 1.6 s, then spin 6 s  ─── back to explore ACTIVE ─┤
│                                                                  │
│  Flag detected by /vision/flag_detection? ─────────────────────►│
└──────────────────────────────────┬──────────────────────────────┘
                                   │  flag seen
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                    BANDEIRA_DETECTADA                           │
│  Deactivate explore_lite (lifecycle TRANSITION_DEACTIVATE)      │
│  Flag map position estimated from LIDAR + bearing               │
└──────────────────────────────────┬──────────────────────────────┘
                                   │  position known
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                  NAVIGANDO_PARA_BANDEIRA                        │
│  FSM sends NavigateToPose goal to Nav2                          │
│  Goal = point COLETA_DISTANCE + 0.2 m in front of flag          │
│  Refreshes goal every 20 ticks if flag position updates         │
│                                                                  │
│  front_dist ≤ 0.45 m ──────────────────────────────────────────►│
│  flag lost ─────────────────────────────────────────────────────┤
└────────────────────────┬─────────────────────┬──────────────────┘
              flag lost  │                     │  close enough
                         ▼                     ▼
┌──────────────────────────────┐  ┌────────────────────────────────┐
│     PROCURANDO_BANDEIRA      │  │   POSICIONANDO_PARA_COLETA     │
│  Spin in place at 1.5 rad/s  │  │  Proportional bearing align    │
│  Up to 6 s (30 ticks)        │  │  Stop when |bearing| < 5°      │
│                              │  │  Announce: VITORIA!            │
│  flag reacquired? ──────────►│  │  (stay stopped)                │
│  timeout? → EXPLORANDO       │  │  flag lost? → PROCURANDO       │
└──────────────────────────────┘  └────────────────────────────────┘
```

**Key FSM parameters (`controle_robo.py`):**

| Constant | Value | Meaning |
|---|---|---|
| `COLETA_DISTANCE` | 0.45 m | LIDAR front distance that triggers POSICIONANDO |
| `WATCHDOG_DIST` | 0.20 m | Minimum movement to reset the idle timer |
| `WATCHDOG_TIMEOUT_S` | 12.0 s | Idle time before recovery starts |
| `BACKUP_SPEED` | -0.15 m/s | Reverse speed during recovery phase 1 |
| `BACKUP_TICKS` | 8 | Backup duration (8 × 0.2 s = 1.6 s) |
| `RECOVERY_SPIN_S` | 6.0 s | Spin duration after backup (phase 2) |
| `HARD_RECOVERY_COUNT` | 3 | Cycles before also clearing global costmap |
| `FLAG_CONFIRM_FRAMES` | 3 | Consecutive detections before trusting the flag |
| `ROTATE_SPEED` | 1.5 rad/s | Spin rate for PROCURANDO and POSICIONANDO |
| `SEARCH_TICKS_360` | 30 ticks | ~6 s for full 360° search in PROCURANDO |

### Topic mismatch: Nav2 ↔ robot

Nav2 writes to `/cmd_vel`; the Gazebo diff-drive controller reads `/diff_drive_base_controller/cmd_vel_unstamped`. `nav2.launch.py` bridges these with a `topic_tools/relay` node.

### Ground-truth odometry

`odom_gt_publisher.py` subscribes to `/model/prm_robot/pose` (Gazebo bridge, `PoseStamped`) and publishes the `odom→base_link` TF and `/odom`. Positions are relative to the spawn pose so the odom origin is (0, 0). Using `PoseStamped` preserves the Ignition sim-time header, keeping TF timestamps aligned with LIDAR scan timestamps so SLAM Toolbox processes scans immediately on startup.

### Semantic labels (arena_cilindros.sdf)

| Label | Object |
|---|---|
| 5, 10, 15 | Walls / arena borders |
| 20 | `red_flag` — robot's own team flag (ignore) |
| 25 | `blue_flag` — **capture target** |
| 28 | Flag deploy zone |
| 30 | Cylindrical obstacles |

---

## Modular Launch

Each sub-launch accepts `verbose:=true` for DEBUG logging.

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
sudo apt install ros-humble-rtabmap-ros
ros2 launch bb8_visual_slam visual_slam.launch.py
ros2 launch bb8_control nav2.launch.py
ros2 launch bb8_control explore_control.launch.py
ros2 launch bb8_control perception.launch.py
```

---

## Diagnostics

```bash
ros2 launch bb8_control diagnostics.launch.py
```

Prints a health summary every 10 s. Monitors LIDAR collision proximity, stuck detection, zero-velocity episodes, map update rate, and more.

---

## RViz Configuration

RViz2 launches automatically. Useful displays to add:

| Display | Topic | Notes |
|---|---|---|
| Map | `/map` | SLAM occupancy grid |
| MarkerArray | `/explore/frontiers` | Current frontier list |
| LaserScan | `/scan` | LIDAR readings |
| Path | `/plan` | Nav2 planned path |
| TF | — | Verify `map→odom→base_link` |
| Image (Raw) | `/robot_cam/colored_map` | Segmentation overlay |

Set **Fixed Frame** to `map`.

---

## Testing & Debugging

```bash
# Monitor FSM state transitions
ros2 topic echo /rosout | grep "\[FSM\]"

# Verify explore_lite
ros2 topic hz /explore/frontiers       # ~1 Hz
ros2 topic echo /explore/status

# Pause / resume exploration manually
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: false"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: true"

# Nav2 goal traffic
ros2 topic echo /navigate_to_pose/_action/status

# TF tree (must show map → odom → base_link)
ros2 run tf2_tools view_frames

# Verify flag detection
ros2 topic echo /vision/flag_detection   # theta=1.0 means detected
ros2 topic echo /vision/flag_bearing
```

### Expected log sequence (healthy run)

```
[controle_robo]: ControleRoboFSM started – EXPLORANDO
[explore_node]:  Received costmap, now running
[controle_robo]: [MAP] First map: 384×384 @ 0.050 m/cell
... (robot explores for N minutes) ...
[vision_processor]: Flag @ (320, 240)px  bearing=2.3°  area=480px
[controle_robo]: [FSM] EXPLORANDO → BANDEIRA_DETECTADA
[controle_robo]: [FSM] BANDEIRA_DETECTADA → NAVIGANDO_PARA_BANDEIRA
[controle_robo]: [Nav2] goal SUCCEEDED
[controle_robo]: [FSM] NAVIGANDO_PARA_BANDEIRA → POSICIONANDO_PARA_COLETA
[controle_robo]:     VITORIA!  BANDEIRA CAPTURADA!
```

### Linting

```bash
colcon test --packages-select bb8_control
colcon test-result --verbose
```

---

## Known Issues

**1. Watchdog / Nav2 recovery overlap**
The FSM watchdog fires after 12 s of no movement. Nav2's own recovery (backup + spin) can take 5–8 s. Both can run simultaneously, causing erratic motion.

**2. Nav2 may not reach within `COLETA_DISTANCE`**
Nav2's `xy_goal_tolerance: 0.15 m` means it stops ~0.70 m from the flag. If the flag is not in the front LIDAR cone at that distance, the robot resends the same goal indefinitely without triggering POSICIONANDO.

**3. Flag position drifts during approach**
`_flag_map_x/y` is updated every FSM tick while the flag is visible. As the robot approaches, the bearing changes, shifting the estimated flag map position — Nav2 replans every 20 ticks.

**4. No obstacle check during direct velocity control**
When the FSM publishes velocity directly (PROCURANDO, POSICIONANDO, recovery spin) it does not consult the LIDAR. The robot can spin into walls at 1.5 rad/s with no avoidance.
