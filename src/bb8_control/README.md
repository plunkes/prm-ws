# bb8_control — Autonomous Exploration & Flag Capture

Autonomous robot package for ROS 2 Humble. The robot explores an arena using
`explore_lite` frontier exploration until it detects a flag via semantic
segmentation, then navigates directly to it using Nav2.

---

## Quick Start

### Install dependencies

```bash
# ROS navigation / SLAM
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-nav2-bringup \
                 ros-humble-nav2-msgs \
                 ros-humble-topic-tools

# Frontier exploration
sudo apt install ros-humble-explore-lite
```

### Build

```bash
cd ~/prm_ws
colcon build --symlink-install --packages-select bb8_control
source install/setup.bash
```

### Launch the full stack

```bash
ros2 launch bb8_control main_exploration.launch.py
```

**With verbose (DEBUG) logging:**

```bash
ros2 launch bb8_control main_exploration.launch.py verbose:=true
```

**Different world:**

```bash
ros2 launch bb8_control main_exploration.launch.py world:=arena_paredes.sdf
```

---

## Behavior Control Flow

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
│  explore_lite cancels its current Nav2 goal                     │
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

**Key FSM parameters (in `controle_robo.py`):**

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
| `EXPLORE_HEARTBEAT_TICKS` | 25 | Ticks between explore_lite keepalive (~5 s) |
| `RESEND_TICKS` | 20 | FSM ticks between Nav2 goal refreshes |
| `ROTATE_SPEED` | 1.5 rad/s | Spin rate for PROCURANDO and POSICIONANDO |
| `SEARCH_TICKS_360` | 30 ticks | ~6 s for full 360° search in PROCURANDO |

---

## Known Issues & Limitations

**1. Watchdog fires before Nav2 recovery completes**
The FSM watchdog triggers after 12 s of no movement (`WATCHDOG_TIMEOUT_S`).
Nav2's own recovery behaviors (backup + spin) can take 5–8 s. If the robot gets
stuck in an obstacle, Nav2 starts recovering, but at 12 s the FSM also fires its
own deactivate + spin recovery. The two recovery mechanisms overlap, which can
cause erratic motion. The `progress_timeout` in `explore.yaml` is set to 15 s
and is effectively never reached because the FSM watchdog always preempts it.

**2. Nav2 may not reach within `COLETA_DISTANCE` of the flag**
The Nav2 goal is offset 0.55 m from the estimated flag position. Nav2's
`xy_goal_tolerance` is 0.15 m, so it considers the goal "reached" when within
0.15 m of the offset point (≈0.70 m from flag). The LIDAR transition to
POSICIONANDO requires `front_dist ≤ 0.45 m`. If Nav2 stops the robot at
0.70 m and the flag is not in the front ±30° LIDAR cone, the robot can get
stuck resending the same Nav2 goal indefinitely without triggering POSICIONANDO.

**4. Flag position drifts while approaching**
`_flag_map_x/y` is updated every FSM tick while the flag is visible. As the
robot approaches, the bearing angle and LIDAR range at that angle change, so
the estimated flag map position shifts. This causes the Nav2 goal to be resent
to a different location every 20 ticks (4 s), forcing Nav2 to replan frequently.

**5. No obstacle check during direct velocity control**
When the FSM publishes velocity directly (PROCURANDO, POSICIONANDO, recovery
spin), it does not check `_front_dist`. The robot can spin into nearby walls
at 1.5 rad/s with no avoidance.

**6. Flag position drifts while approaching (continued)**
If the flag goes out of camera view during the final approach (e.g., robot turns
slightly), the FSM falls back to PROCURANDO_BANDEIRA and spins to reacquire.

---

## Sub-Launch Manual

Each sub-launch can be run independently for isolated testing.
All accept `verbose:=true` to enable DEBUG logging.

### `slam.launch.py` — SLAM only

Starts `async_slam_toolbox_node` with the project's parameter overrides.
Run this when the simulation and robot are already up.

```bash
ros2 launch bb8_control slam.launch.py
# Then verify:
ros2 topic echo /map --once
```

### `nav2.launch.py` — Nav2 navigation stack only

Starts the Nav2 planner, controller, behavior server, bt_navigator, and
velocity smoother. Assumes `/map` and the `map→odom` TF are already available.

```bash
ros2 launch bb8_control nav2.launch.py
# Check all lifecycle nodes are active:
ros2 lifecycle get /bt_navigator
```

### `explore.launch.py` — explore_lite only

Starts `explore_node` (plain ROS 2 node from m-explore-ros2).
The node blocks until Nav2's `navigate_to_pose` action server is ready, then
begins frontier planning automatically.

Exploration is paused/resumed via topic:
```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: false"  # stop
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: true"   # resume
```

```bash
ros2 launch bb8_control explore.launch.py
# Watch frontier markers in RViz (/explore/frontiers)
# Check exploration status:
ros2 topic echo /explore/status
```

### `perception.launch.py` — Vision processor only

Starts the semantic segmentation decoder node.
Requires the Gazebo camera bridge (`/robot_cam/labels_map`) to be active.

```bash
ros2 launch bb8_control perception.launch.py
# Verify flag detection:
ros2 topic echo /vision/flag_detection
ros2 topic echo /vision/flag_bearing
```

### `main_exploration.launch.py` — Full stack

Aggregates all sub-launches plus Gazebo simulation and the master FSM node.
This is the primary entry point.

```bash
ros2 launch bb8_control main_exploration.launch.py
ros2 launch bb8_control main_exploration.launch.py verbose:=true
```

---

## Diagnostics

Run the diagnostic node in a second terminal **while the robot is running**:

```bash
ros2 launch bb8_control diagnostics.launch.py
```

Every 10 seconds it prints a health summary. WARN/ERROR events are also
printed immediately when thresholds are crossed.

### What it monitors

| Check | Threshold | Failure mode it catches |
|---|---|---|
| LIDAR near-miss | < 0.18 m | Robot approaching obstacle dangerously |
| LIDAR collision | < 0.12 m | Robot has likely made contact |
| Stuck detection | < 0.10 m movement in 10 s | Robot stopped in front of obstacle |
| Zero velocity | cmd_vel ≈ 0 for > 8 s | explore_lite/Nav2 not driving the robot |
| Map update rate | < 0.3 Hz | SLAM Toolbox stalled |
| LIDAR rate | < 5 Hz | LIDAR bridge broken |
| Odometry rate | < 5 Hz | Odometry bridge broken |
| Map unknown % | > 80 % after 60 s | Exploration not making progress |

### Example healthy output

```
--------------------------------------------------------------
  BB8 Diagnostics  (runtime 45 s)
--------------------------------------------------------------
  TOPIC RATES
    /scan    20.0 Hz  OK
    /odom    50.0 Hz  OK
    /map      0.8 Hz  OK
  COLLISION PROXIMITY
    Current min LIDAR:  0.412 m
    Session min LIDAR:  0.312 m
    Near-miss events (< 0.18 m):  0
    Collision events  (< 0.12 m):  0
  MOTION
    Total dist travelled:  3.42 m
    Avg linear velocity:   0.521 m/s
    Stuck episodes (>25s idle):  0
    Zero-vel episodes (>8s stopped):  1
  MAP / SLAM
    Unknown cells:  71.3 %
    Explored area:  4.6 m²
  VISION
    /vision/flag_bearing received:  NO — perception may be down
--------------------------------------------------------------
```

---

## RViz Configuration

RViz2 launches automatically via `carrega_robo.launch.py`. Add these displays
manually for full debugging visibility:

| Display type | Topic / Source | Notes |
|---|---|---|
| Map | `/map` | SLAM occupancy grid |
| MarkerArray | `/explore/frontiers` | Current frontier list |
| LaserScan | `/scan` | LIDAR readings |
| Path | `/plan` | Nav2 planned path |
| TF | — | Check `map→odom→base_link` chain |
| Image (Raw) | `/robot_cam/colored_map` | Segmentation colour overlay |

Set **Fixed Frame** to `map`.

---

## Testing & Debugging

### Monitor FSM state transitions

```bash
ros2 topic echo /rosout | grep "\[FSM\]"
```

### Verify explore_lite is running

```bash
ros2 topic hz /explore/frontiers          # should publish at ~1 Hz
ros2 topic echo /explore/status           # EXPLORATION_STARTED or EXPLORATION_COMPLETE
```

### Force a pause/resume manually

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: false"  # pause
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: true"   # resume
```

### Inspect Nav2 goal traffic

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

### Check the TF tree

```bash
ros2 run tf2_tools view_frames
# Must see: world → map → odom → base_link
```

### Record a run for offline analysis

```bash
ros2 bag record /map /scan /tf /tf_static \
    /vision/flag_detection /vision/flag_bearing \
    /explore/frontiers /plan /rosout \
    /diff_drive_base_controller/cmd_vel_unstamped \
    -o exploration_run
```

### Expected log sequence (healthy run)

```
[controle_robo]: ControleRoboFSM started – EXPLORANDO
[controle_robo]: [DIAG] All primary systems GO
[explore_node]:  Received costmap, now running
[controle_robo]: [MAP] First map: 384×384 @ 0.050 m/cell
... (robot explores for N minutes) ...
[vision_processor]: Flag @ (320, 240)px  bearing=2.3°  area=480px
[controle_robo]: [FSM] EXPLORANDO → BANDEIRA_DETECTADA
[controle_robo]: [EXPLORE] Stop sent to explore_lite
[controle_robo]: [FSM] BANDEIRA_DETECTADA → NAVIGANDO_PARA_BANDEIRA
[controle_robo]: [Nav2] goal → (3.45, -1.20)
[controle_robo]: [Nav2] goal SUCCEEDED
[controle_robo]: [FSM] NAVIGANDO_PARA_BANDEIRA → POSICIONANDO_PARA_COLETA
[controle_robo]: ============================================================
[controle_robo]:     VITORIA!  BANDEIRA CAPTURADA!
[controle_robo]:     BB8 completou a missao com sucesso!
```

### Troubleshooting

**Robot doesn't move at all**

- Check: `ros2 topic echo /map --once` (SLAM running?)
- Check: `ros2 topic echo /explore/status` (explore_lite running?)
- Check: `ros2 topic echo /navigate_to_pose/_action/status` (Nav2 receiving goals?)

**explore_lite logs "All frontiers traversed/tried out, stopping"**

- The FSM watchdog detects the stall after 12 s and triggers a recovery spin
  (6 s), then reactivates explore_lite.
- If no new frontiers appear after several cycles, the arena is fully mapped.

**Flag never detected**

- Check: `ros2 topic echo /vision/flag_detection` (theta=1.0 means detected)
- Check: `ros2 topic hz /robot_cam/labels_map` (camera bridge running?)
- Manually inspect: `ros2 run image_view image_view image:=/robot_cam/colored_map`

**Nav2 rejects goals**

- Ensure Nav2 lifecycle nodes are active: `ros2 lifecycle get /bt_navigator`
- Check costmap inflation: reduce `inflation_radius` in `nav2_params.yaml` if
  the robot is in a tight arena and goals fall inside inflated obstacles.

**Robot stops in front of obstacle and does not recover**

- This is the primary known issue (see Known Issues #1 and #2 above).
- Run the diagnostic node to capture stuck-episode counts and zero-vel duration.
- Check `/rosout` for `[EXPLORE] No movement for … Starting recovery spin`.
- If the watchdog is firing but the robot still doesn't recover, the recovery
  spin direction may be pushing it further into the obstacle.
