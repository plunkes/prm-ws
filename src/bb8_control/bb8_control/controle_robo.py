#!/usr/bin/env python3
"""
Master FSM control node — explore_lite edition.

States:
  EXPLORANDO              – explore_lite drives Nav2 autonomously.
  BANDEIRA_DETECTADA      – flag confirmed (3 consecutive frames); explore_lite stopped.
  NAVIGANDO_PARA_BANDEIRA – FSM drives Nav2 toward the flag.
  PROCURANDO_BANDEIRA     – flag lost; robot spins to reacquire.
  POSICIONANDO_PARA_COLETA– final bearing alignment; stop when centred.

Recovery (inside EXPLORANDO):
  If the robot does not move >= WATCHDOG_DIST for WATCHDOG_TIMEOUT_S the FSM:
    1. Cancels the active Nav2 goal.
    2. Clears both Nav2 costmaps (removes phantom obstacles).
    3. Backs up for BACKUP_TICKS * 0.2 s.
    4. Spins for RECOVERY_SPIN_S seconds.
    5. Re-activates explore_lite.
  After HARD_RECOVERY_COUNT cycles without progress the global costmap is also
  force-cleared to dislodge persistent phantom obstacles.
"""

import math

import numpy as np
import rclpy
import rclpy.executors
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Pose2D, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64MultiArray, String


class ControleRoboFSM(Node):
    """FSM-based autonomous controller: exploration, flag detection, approach."""

    # ── Flag-approach parameters ──────────────────────────────────────────────
    COLETA_DISTANCE = 0.45      # m  — LIDAR front threshold → POSICIONANDO
    RESEND_TICKS = 20           # FSM ticks between Nav2 goal refreshes (~4 s)
    ROTATE_SPEED = 1.5          # rad/s for PROCURANDO and POSICIONANDO
    ALIGN_THRESHOLD = math.radians(5)
    SEARCH_TICKS_360 = 30       # ticks (~6 s) for a full 360° search
    FLAG_CONFIRM_FRAMES = 3     # consecutive detections before trusting the flag

    # ── Frontier-exhaustion watchdog ──────────────────────────────────────────
    WATCHDOG_DIST = 0.20        # m  — minimum movement to reset the idle timer
    WATCHDOG_TIMEOUT_S = 12.0   # s  — idle time before recovery starts
    BACKUP_SPEED = -0.15        # m/s (negative = reverse)
    BACKUP_TICKS = 8            # 8 × 0.2 s = 1.6 s backward
    RECOVERY_SPIN_S = 6.0       # s  — spin duration after backup
    HARD_RECOVERY_COUNT = 3     # recovery cycles before clearing global costmap

    def __init__(self):
        """Initialise publishers, subscribers, service clients, and timers."""
        super().__init__("controle_robo")

        self._cb_timer = ReentrantCallbackGroup()
        self._cb_subs = ReentrantCallbackGroup()

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

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, "/map", self._cb_map, qos_map,
            callback_group=self._cb_subs,
        )
        self.create_subscription(
            LaserScan, "/scan", self._cb_scan, qos_sensor,
            callback_group=self._cb_subs,
        )
        self.create_subscription(
            Pose2D, "/vision/flag_detection", self._cb_vision_pose, 10,
            callback_group=self._cb_subs,
        )
        self.create_subscription(
            Float32, "/vision/flag_bearing", self._cb_vision_bearing, 10,
            callback_group=self._cb_subs,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Twist, "/diff_drive_base_controller/cmd_vel_unstamped", 10
        )
        self._pub_gripper = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self._state_pub = self.create_publisher(String, "/bb8/fsm_state", 10)

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav2 = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self._cb_timer,
        )

        # ── explore_lite lifecycle service ────────────────────────────────────
        self._explore_lc = self.create_client(
            ChangeState, "/explore_node/change_state",
            callback_group=self._cb_subs,
        )

        # ── Costmap clear services (called during recovery) ───────────────────
        self._local_costmap_clear = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self._cb_subs,
        )
        self._global_costmap_clear = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self._cb_subs,
        )

        # ── Sensor state ──────────────────────────────────────────────────────
        self._mapa: np.ndarray = None
        self._map_info = None
        self._scan: LaserScan = None
        self._front_dist = float("inf")
        self._flag_detected = False
        self._flag_consec = 0       # consecutive positive vision frames
        self._flag_bearing = 0.0
        self._flag_map_x: float = None
        self._flag_map_y: float = None

        # ── Nav2 bookkeeping ──────────────────────────────────────────────────
        self._nav2_handle = None
        self._nav2_active = False
        self._nav2_last_xy = None
        self._nav2_ticks = 0

        # ── Arm retraction ────────────────────────────────────────────────────
        self._arm_retract_sent = 0

        # ── Recovery state ────────────────────────────────────────────────────
        self._watchdog_pos: tuple = None
        self._watchdog_idle_s = 0.0
        self._recovery_backing = False
        self._recovery_back_ticks = 0
        self._recovery_spinning = False
        self._recovery_ticks = 0
        self._recovery_count = 0    # cycles since last map progress

        # ── FSM ───────────────────────────────────────────────────────────────
        self._estado = "EXPLORANDO"
        self._procurando_ticks = 0
        self._victory_announced = False

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.2, self._fsm_tick, callback_group=self._cb_timer)
        self._diag_timer = self.create_timer(
            8.0, self._startup_diagnostic, callback_group=self._cb_timer
        )
        self._arm_timer = self.create_timer(
            0.5, self._fold_arm, callback_group=self._cb_timer
        )

        self.get_logger().info(
            "ControleRoboFSM started – EXPLORANDO (explore_lite in control)"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # ARM RETRACTION
    # ─────────────────────────────────────────────────────────────────────────

    def _fold_arm(self):
        """Send arm-retraction commands for the first few seconds after startup."""
        msg = Float64MultiArray()
        msg.data = [-1.5708, -1.5708, 0.0, 0.0]
        self._pub_gripper.publish(msg)
        self._arm_retract_sent += 1
        if self._arm_retract_sent >= 20:
            self._arm_timer.cancel()
            self.get_logger().info("[INIT] Arm retraction complete")

    # ─────────────────────────────────────────────────────────────────────────
    # STARTUP DIAGNOSTIC
    # ─────────────────────────────────────────────────────────────────────────

    def _startup_diagnostic(self):
        """Log one-time health check 8 s after startup, then cancel itself."""
        self._diag_timer.cancel()
        ok = True

        if self._mapa is None:
            self.get_logger().error(
                "[DIAG] /map never received – check slam_toolbox. "
                "Try: ros2 topic echo /map --once"
            )
            ok = False
        else:
            self.get_logger().info(
                f"[DIAG] /map OK – {self._mapa.shape}, "
                f"res={self._map_info.resolution:.3f} m/cell"
            )

        pose = self._get_map_pose()
        if pose is None:
            self.get_logger().error(
                "[DIAG] TF map→base_link unavailable – check slam_toolbox. "
                "Try: ros2 run tf2_tools view_frames"
            )
            ok = False
        else:
            self.get_logger().info(
                f"[DIAG] TF OK – robot at map ({pose[0]:.2f}, {pose[1]:.2f})"
            )

        if self._scan is None:
            self.get_logger().error("[DIAG] /scan never received!")
            ok = False
        else:
            self.get_logger().info("[DIAG] /scan OK")

        if not self._nav2.server_is_ready():
            self.get_logger().warn(
                "[DIAG] Nav2 not ready yet – exploration works without it, "
                "but flag navigation will fail until Nav2 is active."
            )
        else:
            self.get_logger().info("[DIAG] Nav2 action server OK")

        if not self._explore_lc.service_is_ready():
            self.get_logger().warn(
                "[DIAG] /explore_node/change_state not available – "
                "explore_lite may still be starting up."
            )
        else:
            self.get_logger().info("[DIAG] explore_lite lifecycle service OK")

        if ok:
            self.get_logger().info("[DIAG] All primary systems GO")

    # ─────────────────────────────────────────────────────────────────────────
    # SENSOR CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_map(self, msg: OccupancyGrid):
        """Store the latest occupancy grid."""
        first = self._mapa is None
        self._map_info = msg.info
        self._mapa = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        if first:
            self.get_logger().info(
                f"[MAP] First map: {msg.info.width}×{msg.info.height} "
                f"@ {msg.info.resolution:.3f} m/cell"
            )

    def _cb_scan(self, msg: LaserScan):
        """Cache latest scan and compute the minimum distance in the forward ±30° cone."""
        self._scan = msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = msg.range_max
        n = len(ranges)
        half = int(math.radians(30) / msg.angle_increment)
        c = n // 2
        front = ranges[max(0, c - half): min(n, c + half + 1)]
        self._front_dist = float(np.min(front)) if len(front) > 0 else msg.range_max

    def _cb_vision_pose(self, msg: Pose2D):
        """Count consecutive positive detections; require FLAG_CONFIRM_FRAMES before flagging."""
        if msg.theta > 0.5:
            self._flag_consec += 1
            self._flag_detected = self._flag_consec >= self.FLAG_CONFIRM_FRAMES
        else:
            self._flag_consec = 0
            self._flag_detected = False

    def _cb_vision_bearing(self, msg: Float32):
        """Store the bearing (rad) to the flag centre in the camera frame."""
        self._flag_bearing = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    # COORDINATE HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _get_map_pose(self):
        """Return (x, y, yaw) of base_link in map frame, or None on TF error."""
        try:
            tf = self._tf.lookup_transform("map", "base_link", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y**2 + q.z**2),
            )
            return t.x, t.y, yaw
        except Exception:
            return None

    def _estimate_flag_map_pos(self, rx, ry, ryaw):
        """Project camera bearing + LIDAR range into map-frame flag coordinates."""
        if self._scan is None:
            return None, None
        s = self._scan
        angle = self._flag_bearing
        idx = int(round((angle - s.angle_min) / s.angle_increment))
        idx = max(0, min(len(s.ranges) - 1, idx))
        dist = s.ranges[idx]
        if not math.isfinite(dist):
            dist = min(s.range_max, 2.0)
        dist = max(dist, 0.3)
        world_angle = ryaw + angle
        return rx + dist * math.cos(world_angle), ry + dist * math.sin(world_angle)

    # ─────────────────────────────────────────────────────────────────────────
    # EXPLORE_LITE LIFECYCLE
    # ─────────────────────────────────────────────────────────────────────────

    def _deactivate_explore(self):
        """Send ACTIVE → INACTIVE transition to explore_lite (async)."""
        if not self._explore_lc.service_is_ready():
            self.get_logger().warn(
                "[EXPLORE] Lifecycle service not ready; cannot deactivate."
            )
            return
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self._explore_lc.call_async(req)
        self.get_logger().info("[EXPLORE] Deactivate request sent to explore_lite")

    def _activate_explore(self):
        """Send INACTIVE → ACTIVE transition to explore_lite (async)."""
        if not self._explore_lc.service_is_ready():
            self.get_logger().warn(
                "[EXPLORE] Lifecycle service not ready; cannot activate."
            )
            return
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self._explore_lc.call_async(req)
        self.get_logger().info("[EXPLORE] Activate request sent to explore_lite")

    # ─────────────────────────────────────────────────────────────────────────
    # COSTMAP CLEARING
    # ─────────────────────────────────────────────────────────────────────────

    def _clear_local_costmap(self):
        """Async request to clear the local costmap obstacle layer."""
        if self._local_costmap_clear.service_is_ready():
            self._local_costmap_clear.call_async(ClearEntireCostmap.Request())
            self.get_logger().info("[RECOVERY] Local costmap cleared")

    def _clear_global_costmap(self):
        """Async request to clear the global costmap obstacle layer."""
        if self._global_costmap_clear.service_is_ready():
            self._global_costmap_clear.call_async(ClearEntireCostmap.Request())
            self.get_logger().warn("[RECOVERY] Global costmap cleared (hard recovery)")

    # ─────────────────────────────────────────────────────────────────────────
    # NAV2 HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _send_nav2_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send or refresh a NavigateToPose goal; skip if same goal is already active."""
        if not self._nav2.server_is_ready():
            self.get_logger().warn(
                "[Nav2] Action server not ready", throttle_duration_sec=3.0
            )
            return False

        goal_key = (round(x, 1), round(y, 1))
        if self._nav2_active and self._nav2_last_xy == goal_key:
            return True

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)

        self._nav2_active = True
        self._nav2_last_xy = goal_key
        self._nav2_ticks = 0
        future = self._nav2.send_goal_async(goal)
        future.add_done_callback(self._on_nav2_accepted)
        self.get_logger().info(f"[Nav2] goal → ({x:.2f}, {y:.2f})")
        return True

    def _on_nav2_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("[Nav2] goal REJECTED")
            self._nav2_active = False
            self._nav2_last_xy = None
            return
        self._nav2_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav2_done)

    def _on_nav2_done(self, future):
        status = future.result().status
        names = {4: "SUCCEEDED", 5: "CANCELLED"}
        label = names.get(status, f"ABORTED (status={status})")
        self.get_logger().info(f"[Nav2] goal {label}")
        self._nav2_active = False
        self._nav2_handle = None
        self._nav2_last_xy = None

    def _cancel_nav2(self):
        """Cancel any in-flight Nav2 goal."""
        if self._nav2_handle is not None:
            self._nav2_handle.cancel_goal_async()
            self._nav2_handle = None
        self._nav2_active = False
        self._nav2_last_xy = None

    # ─────────────────────────────────────────────────────────────────────────
    # FSM
    # ─────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        """Change FSM state and manage explore_lite lifecycle at boundaries."""
        if self._estado == new_state:
            return
        old = self._estado
        self._estado = new_state
        self.get_logger().info(f"[FSM] {old} → {new_state}")

        if old == "EXPLORANDO" and new_state != "EXPLORANDO":
            self._recovery_backing = False
            self._recovery_spinning = False
            self._deactivate_explore()

        elif new_state == "EXPLORANDO" and old != "EXPLORANDO":
            self._activate_explore()
            self._watchdog_pos = None
            self._watchdog_idle_s = 0.0

    def _fsm_tick(self):
        """Run one FSM tick: evaluate transitions then execute the current state action."""
        pose = self._get_map_pose()
        if pose is None:
            self.get_logger().warn(
                "Waiting for TF map→base_link…", throttle_duration_sec=3.0
            )
            return
        rx, ry, ryaw = pose

        self._nav2_ticks += 1

        self._state_pub.publish(String(data=self._estado))

        if self._flag_detected:
            fx, fy = self._estimate_flag_map_pos(rx, ry, ryaw)
            if fx is not None:
                self._flag_map_x = fx
                self._flag_map_y = fy

        # ── State transitions ─────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            if self._flag_detected:
                self._transition("BANDEIRA_DETECTADA")

        elif self._estado == "BANDEIRA_DETECTADA":
            if self._flag_map_x is not None:
                self._transition("NAVIGANDO_PARA_BANDEIRA")
            else:
                self.get_logger().warn(
                    "[FSM] Flag seen but no position estimate – retrying"
                )
                self._transition("EXPLORANDO")

        elif self._estado == "NAVIGANDO_PARA_BANDEIRA":
            if not self._flag_detected:
                self._cancel_nav2()
                self._procurando_ticks = 0
                self._transition("PROCURANDO_BANDEIRA")
            elif self._front_dist <= self.COLETA_DISTANCE:
                self._cancel_nav2()
                self._transition("POSICIONANDO_PARA_COLETA")

        elif self._estado == "PROCURANDO_BANDEIRA":
            if self._flag_detected:
                self._transition("NAVIGANDO_PARA_BANDEIRA")
            else:
                self._procurando_ticks += 1
                if self._procurando_ticks >= self.SEARCH_TICKS_360:
                    self._procurando_ticks = 0
                    self._transition("EXPLORANDO")

        elif self._estado == "POSICIONANDO_PARA_COLETA":
            if not self._flag_detected:
                self._procurando_ticks = 0
                self._transition("PROCURANDO_BANDEIRA")

        # ── State actions ─────────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            self._do_explorando(rx, ry)
        elif self._estado == "NAVIGANDO_PARA_BANDEIRA":
            self._do_navegando(rx, ry)
        elif self._estado == "PROCURANDO_BANDEIRA":
            self._do_procurando()
        elif self._estado == "POSICIONANDO_PARA_COLETA":
            self._do_posicionando()

    # ─────────────────────────────────────────────────────────────────────────
    # STATE ACTIONS
    # ─────────────────────────────────────────────────────────────────────────

    def _do_explorando(self, rx: float, ry: float):
        """Run the watchdog and manage the 3-phase stuck recovery.

        Phases: (1) cancel + clear costmaps, (2) back up, (3) spin.
        explore_lite drives the robot between recovery cycles.
        """
        # ── Phase 2: backing up ───────────────────────────────────────────────
        if self._recovery_backing:
            twist = Twist()
            twist.linear.x = self.BACKUP_SPEED
            self._cmd_pub.publish(twist)
            self._recovery_back_ticks -= 1
            if self._recovery_back_ticks <= 0:
                self._recovery_backing = False
                self._recovery_spinning = True
                self._recovery_ticks = int(self.RECOVERY_SPIN_S / 0.2)
                self._cmd_pub.publish(Twist())
            return

        # ── Phase 3: spinning ─────────────────────────────────────────────────
        if self._recovery_spinning:
            twist = Twist()
            twist.angular.z = self.ROTATE_SPEED
            self._cmd_pub.publish(twist)
            self._recovery_ticks -= 1
            if self._recovery_ticks <= 0:
                self._recovery_spinning = False
                self._cmd_pub.publish(Twist())
                self._activate_explore()
                self._watchdog_pos = None
                self._watchdog_idle_s = 0.0
                self.get_logger().info(
                    "[EXPLORE] Recovery complete – explore_lite reactivated"
                )
            return

        # ── Phase 1: watchdog ─────────────────────────────────────────────────
        if self._watchdog_pos is None:
            self._watchdog_pos = (rx, ry)
            return

        dist = math.hypot(rx - self._watchdog_pos[0], ry - self._watchdog_pos[1])
        if dist >= self.WATCHDOG_DIST:
            self._watchdog_pos = (rx, ry)
            self._watchdog_idle_s = 0.0
        else:
            self._watchdog_idle_s += 0.2
            if self._watchdog_idle_s >= self.WATCHDOG_TIMEOUT_S:
                self.get_logger().warn(
                    f"[EXPLORE] No movement for {self.WATCHDOG_TIMEOUT_S:.0f} s "
                    "– starting recovery."
                )
                self._cancel_nav2()
                self._deactivate_explore()
                self._clear_local_costmap()

                self._recovery_count += 1
                if self._recovery_count >= self.HARD_RECOVERY_COUNT:
                    self._clear_global_costmap()
                    self._recovery_count = 0

                self._recovery_backing = True
                self._recovery_back_ticks = self.BACKUP_TICKS
                self._watchdog_idle_s = 0.0

    def _do_navegando(self, rx: float, ry: float):
        """Send / refresh a Nav2 goal toward a stopping point in front of the flag."""
        if self._flag_map_x is None:
            return
        if self._nav2_active and self._nav2_ticks < self.RESEND_TICKS:
            return

        dx = self._flag_map_x - rx
        dy = self._flag_map_y - ry
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return

        face_yaw = math.atan2(dy, dx)
        # Goal is placed slightly closer than COLETA_DISTANCE so Nav2 goal
        # tolerance still lands within the LIDAR trigger range.
        offset = self.COLETA_DISTANCE + 0.1
        if dist > offset:
            gx = self._flag_map_x - (dx / dist) * offset
            gy = self._flag_map_y - (dy / dist) * offset
        else:
            gx, gy = rx, ry

        self._send_nav2_goal(gx, gy, yaw=face_yaw)

    def _do_procurando(self):
        """Spin in place to reacquire the lost flag."""
        twist = Twist()
        twist.angular.z = self.ROTATE_SPEED
        self._cmd_pub.publish(twist)

    def _do_posicionando(self):
        """
        Proportional bearing alignment toward the flag centre.

        Stops and announces victory once the bearing is within ALIGN_THRESHOLD.
        """
        twist = Twist()
        if abs(self._flag_bearing) > self.ALIGN_THRESHOLD:
            twist.angular.z = self.ROTATE_SPEED * math.copysign(
                1.0, self._flag_bearing
            )
        else:
            if not self._victory_announced:
                self._victory_announced = True
                b = "=" * 60
                self.get_logger().info(b)
                self.get_logger().info("    VITORIA!  BANDEIRA CAPTURADA!")
                self.get_logger().info("    BB8 completou a missao com sucesso!")
                self.get_logger().info(b)
        self._cmd_pub.publish(twist)


# ─────────────────────────────────────────────────────────────────────────────


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = ControleRoboFSM()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
