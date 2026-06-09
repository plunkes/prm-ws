#!/usr/bin/env python3
"""
BB8 runtime diagnostic node.

Subscribes to key topics and prints a periodic health summary covering:
  - FSM state tracking
  - Flag detection events
  - Collision proximity  (nearest LIDAR obstacle)
  - Stuck detection      (robot not moving)
  - SLAM / map health    (update rate, % unknown cells)
  - Velocity profile     (average speed, zero-velocity episodes)
  - Topic presence       (are all required topics publishing?)

All findings are printed every REPORT_INTERVAL_S seconds and also whenever a
WARN/ERROR threshold is crossed.

Launch:
  ros2 run bb8_control diagnostics
  ros2 launch bb8_control diagnostics.launch.py
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# ── Thresholds ────────────────────────────────────────────────────────────────
COLLISION_ALERT_M = 0.18        # WARN: obstacle within this range
COLLISION_CRITICAL_M = 0.12     # ERROR: likely contact
STUCK_THRESHOLD_M = 0.10        # metres to reset idle timer
STUCK_WARN_S = 10.0             # seconds idle before WARN
STUCK_ERROR_S = 25.0            # seconds idle before ERROR
ZERO_VEL_WARN_S = 8.0           # seconds of zero cmd_vel before WARN
MAP_RATE_WARN_HZ = 0.3          # SLAM stall threshold
SCAN_RATE_WARN_HZ = 2.0         # LIDAR bridge broken threshold
ODOM_RATE_WARN_HZ = 1.5         # odometry bridge broken threshold (relay ~2 Hz)
UNKNOWN_CELL_WARN_PCT = 80.0    # % unknown after 60 s → barely explored
REPORT_INTERVAL_S = 10.0


class BB8Diagnostics(Node):
    """Periodic health monitor for the BB8 exploration stack."""

    def __init__(self):
        """Initialise subscriptions, counters, and timers."""
        super().__init__("bb8_diagnostics")

        self._start_time = time.monotonic()

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, "/scan", self._cb_scan, qos_sensor)
        # ros2_control publishes odometry here; relay_odom bridges to /odom
        self.create_subscription(
            Odometry,
            "/diff_drive_base_controller/odom",
            self._cb_odom,
            qos_sensor,
        )
        self.create_subscription(OccupancyGrid, "/map", self._cb_map, qos_map)
        self.create_subscription(
            Twist,
            "/diff_drive_base_controller/cmd_vel_unstamped",
            self._cb_cmdvel,
            10,
        )
        self.create_subscription(
            Pose2D, "/vision/flag_detection", self._cb_flag_detection, 10
        )
        self.create_subscription(String, "/bb8/fsm_state", self._cb_fsm_state, 10)

        # ── LIDAR ─────────────────────────────────────────────────────────────
        self._scan_count = 0
        self._scan_window_start = time.monotonic()
        self._scan_hz = 0.0
        self._min_lidar = float("inf")
        self._near_events = 0           # < COLLISION_ALERT_M
        self._collision_events = 0      # < COLLISION_CRITICAL_M
        self._current_min = float("inf")

        # ── Odometry / stuck detection ─────────────────────────────────────────
        self._odom_count = 0
        self._odom_window_start = time.monotonic()
        self._odom_hz = 0.0
        self._last_x: float = None
        self._last_y: float = None
        self._stuck_since: float = None
        self._stuck_ref_x: float = None
        self._stuck_ref_y: float = None
        self._stuck_episodes = 0
        self._total_dist_m = 0.0

        # ── cmd_vel / velocity profile ─────────────────────────────────────────
        self._zero_vel_since: float = None
        self._zero_vel_episodes = 0
        self._lin_vel_sum = 0.0
        self._lin_vel_n = 0

        # ── Map / SLAM ─────────────────────────────────────────────────────────
        self._map_count = 0
        self._map_window_start = time.monotonic()
        self._map_hz = 0.0
        self._unknown_pct = 100.0
        self._explored_m2 = 0.0

        # ── Vision / FSM ──────────────────────────────────────────────────────
        self._flag_detections = 0       # frames where theta > 0.5
        self._current_fsm_state = "unknown"
        self._fsm_state_counts: dict = {}

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0, self._check_stuck)
        self.create_timer(REPORT_INTERVAL_S, self._report)

        self.get_logger().info(
            f"BB8Diagnostics started — report every {REPORT_INTERVAL_S:.0f} s"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_scan(self, msg: LaserScan):
        """Update LIDAR rate, minimum distance, and collision counters."""
        self._scan_count += 1
        now = time.monotonic()
        elapsed = now - self._scan_window_start
        if elapsed >= 2.0:
            self._scan_hz = self._scan_count / elapsed
            self._scan_count = 0
            self._scan_window_start = now

        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = msg.range_max
        cur_min = float(np.min(ranges)) if len(ranges) > 0 else msg.range_max
        self._current_min = cur_min

        if cur_min < self._min_lidar:
            self._min_lidar = cur_min

        if cur_min < COLLISION_ALERT_M:
            self._near_events += 1
            if cur_min < COLLISION_CRITICAL_M:
                self._collision_events += 1
                self.get_logger().error(
                    f"[COLLISION] obstacle at {cur_min:.3f} m "
                    f"(threshold {COLLISION_CRITICAL_M} m)",
                    throttle_duration_sec=1.0,
                )

    def _cb_odom(self, msg: Odometry):
        """Update odometry rate and accumulate distance travelled."""
        self._odom_count += 1
        now = time.monotonic()
        elapsed = now - self._odom_window_start
        if elapsed >= 2.0:
            self._odom_hz = self._odom_count / elapsed
            self._odom_count = 0
            self._odom_window_start = now

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_x is not None:
            self._total_dist_m += math.hypot(x - self._last_x, y - self._last_y)
        self._last_x = x
        self._last_y = y

    def _cb_map(self, msg: OccupancyGrid):
        """Update map rate, unknown cell percentage, and explored area."""
        self._map_count += 1
        now = time.monotonic()
        elapsed = now - self._map_window_start
        if elapsed >= 5.0:
            self._map_hz = self._map_count / elapsed
            self._map_count = 0
            self._map_window_start = now

        data = np.array(msg.data, dtype=np.int8)
        total = len(data)
        unknown = int(np.sum(data == -1))
        free = int(np.sum(data == 0))
        self._unknown_pct = 100.0 * unknown / total if total > 0 else 100.0
        self._explored_m2 = free * msg.info.resolution ** 2

    def _cb_cmdvel(self, msg: Twist):
        """Track cmd_vel: detect zero-velocity episodes and average speed."""
        lin = abs(msg.linear.x)
        self._lin_vel_sum += lin
        self._lin_vel_n += 1
        now = time.monotonic()
        if lin < 0.02 and abs(msg.angular.z) < 0.05:
            if self._zero_vel_since is None:
                self._zero_vel_since = now
        else:
            if self._zero_vel_since is not None:
                if now - self._zero_vel_since > ZERO_VEL_WARN_S:
                    self._zero_vel_episodes += 1
            self._zero_vel_since = None

    def _cb_flag_detection(self, msg: Pose2D):
        """Count frames where the flag is actually visible (theta > 0.5)."""
        if msg.theta > 0.5:
            self._flag_detections += 1

    def _cb_fsm_state(self, msg: String):
        """Track current FSM state and count ticks per state."""
        self._current_fsm_state = msg.data
        self._fsm_state_counts[msg.data] = (
            self._fsm_state_counts.get(msg.data, 0) + 1
        )

    # ─────────────────────────────────────────────────────────────────────────
    # PERIODIC CHECKS
    # ─────────────────────────────────────────────────────────────────────────

    def _check_stuck(self):
        """1 Hz check: alert when the robot has been idle too long."""
        if self._last_x is None:
            return
        now = time.monotonic()

        if self._stuck_since is None:
            self._stuck_since = now
            self._stuck_ref_x = self._last_x
            self._stuck_ref_y = self._last_y
            return

        dist = math.hypot(
            self._last_x - self._stuck_ref_x,
            self._last_y - self._stuck_ref_y,
        )
        if dist >= STUCK_THRESHOLD_M:
            self._stuck_since = now
            self._stuck_ref_x = self._last_x
            self._stuck_ref_y = self._last_y
        else:
            idle = now - self._stuck_since
            if idle > STUCK_ERROR_S:
                self.get_logger().error(
                    f"[STUCK] No movement in {idle:.0f} s",
                    throttle_duration_sec=5.0,
                )
                # Count each new crossing of the ERROR threshold once
                if idle - 1.0 < STUCK_ERROR_S:
                    self._stuck_episodes += 1
            elif idle > STUCK_WARN_S:
                self.get_logger().warn(
                    f"[STUCK] No movement in {idle:.0f} s",
                    throttle_duration_sec=5.0,
                )

    def _report(self):
        """Print the full diagnostic summary."""
        runtime = time.monotonic() - self._start_time
        avg_vel = self._lin_vel_sum / self._lin_vel_n if self._lin_vel_n > 0 else 0.0

        # Time each state was active (approximate ticks × 0.2 s)
        state_times = {
            k: v * 0.2 for k, v in self._fsm_state_counts.items()
        }
        state_str = "  ".join(
            f"{k}={v:.0f}s" for k, v in sorted(state_times.items())
        ) or "no data"

        sep = "-" * 62
        lines = [
            "",
            sep,
            f"  BB8 Diagnostics  (runtime {runtime:.0f} s)",
            sep,
            "  TOPIC RATES",
            f"    /scan   {self._scan_hz:5.1f} Hz  "
            + ("OK" if self._scan_hz >= SCAN_RATE_WARN_HZ or runtime < 5
               else "WARN — LIDAR bridge may be down"),
            f"    /odom   {self._odom_hz:5.1f} Hz  "
            + ("OK" if self._odom_hz >= ODOM_RATE_WARN_HZ
               else "WARN — check relay_odom in carrega_robo.launch.py"),
            f"    /map    {self._map_hz:5.2f} Hz  "
            + ("OK" if self._map_hz >= MAP_RATE_WARN_HZ
               else ("WARN — SLAM may be stalled" if runtime > 15 else "...")),
            "",
            "  FSM",
            f"    Current state:  {self._current_fsm_state}",
            f"    Time per state: {state_str}",
            f"    Flag detections (confirmed frames):  {self._flag_detections}",
            "",
            "  COLLISION PROXIMITY",
            f"    Current min LIDAR:  {self._current_min:.3f} m",
            f"    Session min LIDAR:  {self._min_lidar:.3f} m",
            f"    Near-miss events (< {COLLISION_ALERT_M} m):  {self._near_events}",
            f"    Collision events  (< {COLLISION_CRITICAL_M} m):  "
            + f"{self._collision_events}"
            + ("  ERROR" if self._collision_events else ""),
            "",
            "  MOTION",
            f"    Total dist travelled:  {self._total_dist_m:.2f} m",
            f"    Avg linear velocity:   {avg_vel:.3f} m/s",
            f"    Stuck episodes (>{STUCK_ERROR_S:.0f}s idle):  {self._stuck_episodes}",
            f"    Zero-vel episodes (>{ZERO_VEL_WARN_S:.0f}s stopped):  "
            f"{self._zero_vel_episodes}",
            "",
            "  MAP / SLAM",
            f"    Unknown cells:  {self._unknown_pct:.1f} %"
            + ("  WARN — barely explored"
               if self._unknown_pct > UNKNOWN_CELL_WARN_PCT and runtime > 60 else ""),
            f"    Explored area:  {self._explored_m2:.1f} m²",
            sep,
        ]
        for line in lines:
            self.get_logger().info(line)


# ─────────────────────────────────────────────────────────────────────────────


def main(args=None):
    """Entry point for ros2 run."""
    rclpy.init(args=args)
    node = BB8Diagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
