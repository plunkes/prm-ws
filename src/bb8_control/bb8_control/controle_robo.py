#!/usr/bin/env python3
"""
Master FSM control node — explore_lite edition.

States:
  EXPLORANDO              – explore_lite drives Nav2 autonomously.
  BANDEIRA_DETECTADA      – flag confirmed (3 consecutive frames); explore_lite stopped.
  NAVEGANDO_BANDEIRA      – FSM drives Nav2 toward the flag.
  PROCURANDO_BANDEIRA     – flag lost; explore_lite reactivated to reacquire.
  POSICIONANDO_PARA_COLETA– final bearing alignment; stops and extends arm when centred.

Global watchdog:
  In any state except post-victory POSICIONANDO, if the robot has not moved
  WATCHDOG_DIST metres for WATCHDOG_REAL_TIMEOUT wall-clock seconds the FSM
  cancels the active Nav2 goal, clears the local costmap, and transitions to
  EXPLORANDO.  The watchdog only activates after the first valid map arrives.
"""

import math
import time

import numpy as np
import rclpy
import rclpy.executors
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Pose2D, Twist
from visualization_msgs.msg import Marker
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
from std_msgs.msg import Bool, Float32, Float64MultiArray, String


class ControleRoboFSM(Node):
    """FSM-based autonomous controller: exploration, flag detection, approach."""

    # ── Flag-approach parameters ──────────────────────────────────────────────
    FLAG_STOP_DISTANCE = 1.0    # m  — Euclidean distance to flag → stop + victory
    RESEND_TICKS = 20           # FSM ticks between Nav2 goal refreshes (~4 s)
    SEARCH_TICKS_360 = 30       # ticks before abandoning PROCURANDO (~6 sim-s)
    FLAG_CONFIRM_FRAMES = 3     # consecutive detections before trusting the flag

    # ── Global movement watchdog ──────────────────────────────────────────────
    WATCHDOG_DIST = 0.20           # m  — minimum movement to reset the idle timer
    WATCHDOG_REAL_TIMEOUT = 20.0   # wall-clock seconds of no movement → EXPLORANDO

    # ── Heartbeat ─────────────────────────────────────────────────────────────
    EXPLORE_HEARTBEAT_TICKS = 25   # ticks between keepalive resumes (~5 s at 5 Hz)

    # ── Arm poses ─────────────────────────────────────────────────────────────
    # joint order: [gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint]
    ARM_RETRACT = [-1.5708, -1.5708, 0.0, 0.0]     # folded away from robot front
    ARM_CAPTURE = [0.2, 0.5, -0.06, 0.06]           # extended forward, gripper open

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
        # Latched so RViz shows the marker as soon as it subscribes
        self._flag_marker_pub = self.create_publisher(
            Marker, "/flag_marker",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav2 = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self._cb_timer,
        )

        # ── explore_lite control (Bool: False=stop, True=resume) ──────────────
        self._explore_resume_pub = self.create_publisher(Bool, "/explore/resume", 10)

        # ── Costmap clear services ─────────────────────────────────────────────
        # Global costmap is cleared ONCE on the first successful TF lookup so
        # Nav2 reinitialises from the real SLAM map instead of a stale 0×0 map.
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
        self._global_costmap_initialized = False

        # ── Sensor state ──────────────────────────────────────────────────────
        self._mapa: np.ndarray = None
        self._map_info = None
        self._map_valid = False
        self._scan: LaserScan = None
        self._front_dist = float("inf")
        self._flag_detected = False
        self._flag_consec = 0
        self._flag_bearing = 0.0
        self._flag_map_x: float = None
        self._flag_map_y: float = None

        # ── Nav2 bookkeeping ──────────────────────────────────────────────────
        self._nav2_handle = None
        self._nav2_active = False
        self._nav2_last_xy = None
        self._nav2_ticks = 0

        # ── Arm state ─────────────────────────────────────────────────────────
        self._arm_retract_sent = 0

        # ── Watchdog (wall-clock time) ─────────────────────────────────────────
        self._watchdog_pos: tuple = None
        self._watchdog_last_moved: float = None   # time.monotonic() timestamp

        # ── Heartbeat ─────────────────────────────────────────────────────────
        self._explore_heartbeat_ticks = 0

        # ── FSM ───────────────────────────────────────────────────────────────
        self._estado = "EXPLORANDO"
        self._procurando_ticks = 0
        self._victory_announced = False

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.2, self._fsm_tick, callback_group=self._cb_timer)
        self._diag_timer = self.create_timer(
            30.0, self._startup_diagnostic, callback_group=self._cb_timer
        )
        self._arm_timer = self.create_timer(
            0.5, self._fold_arm, callback_group=self._cb_timer
        )

        self.get_logger().info(
            "ControleRoboFSM started – EXPLORANDO (explore_lite in control)"
        )
        self._state_pub.publish(String(data=self._estado))

    # ─────────────────────────────────────────────────────────────────────────
    # ARM CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def _fold_arm(self):
        """Send arm-retraction commands for the first few seconds after startup."""
        msg = Float64MultiArray()
        msg.data = self.ARM_RETRACT
        self._pub_gripper.publish(msg)
        self._arm_retract_sent += 1
        if self._arm_retract_sent >= 20:
            self._arm_timer.cancel()
            self.get_logger().info("[INIT] Arm retraction complete")

    def _extend_arm_capture(self):
        """Extend arm forward with gripper open — capture pose."""
        msg = Float64MultiArray()
        msg.data = self.ARM_CAPTURE
        self._pub_gripper.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # STARTUP DIAGNOSTIC
    # ─────────────────────────────────────────────────────────────────────────

    def _startup_diagnostic(self):
        """Log one-time health check 30 s after startup, then cancel itself."""
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

        if ok:
            self.get_logger().info("[DIAG] All primary systems GO")

    # ─────────────────────────────────────────────────────────────────────────
    # SENSOR CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_map(self, msg: OccupancyGrid):
        """Store the latest occupancy grid."""
        self._map_info = msg.info
        self._mapa = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        if msg.info.width > 0 and msg.info.height > 0 and not self._map_valid:
            self._map_valid = True
            self.get_logger().info(
                f"[MAP] First valid map: {msg.info.width}×{msg.info.height} "
                f"@ {msg.info.resolution:.3f} m/cell — watchdog active"
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
        """Count consecutive positive detections; require FLAG_CONFIRM_FRAMES."""
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
    # EXPLORE_LITE CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def _deactivate_explore(self):
        self._explore_resume_pub.publish(Bool(data=False))
        self.get_logger().info("[EXPLORE] Stop sent to explore_lite")

    def _activate_explore(self):
        self._explore_resume_pub.publish(Bool(data=True))
        self.get_logger().info("[EXPLORE] Resume sent to explore_lite")

    # ─────────────────────────────────────────────────────────────────────────
    # COSTMAP CLEARING
    # ─────────────────────────────────────────────────────────────────────────

    def _clear_local_costmap(self):
        """Async request to clear the local costmap obstacle layer."""
        if self._local_costmap_clear.service_is_ready():
            self._local_costmap_clear.call_async(ClearEntireCostmap.Request())
            self.get_logger().info("[RECOVERY] Local costmap cleared")

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
        """Change FSM state, manage explore_lite lifecycle, publish for diagnostics."""
        if self._estado == new_state:
            return
        old = self._estado
        self._estado = new_state
        self.get_logger().info(f"[FSM] {old} → {new_state}")
        self._state_pub.publish(String(data=new_state))

        # Reset watchdog on every transition so the 20 s window starts fresh.
        self._watchdog_pos = None
        self._watchdog_last_moved = None

        # explore_lite is active during EXPLORANDO and PROCURANDO_BANDEIRA.
        explore_states = ("EXPLORANDO", "PROCURANDO_BANDEIRA")
        if new_state in explore_states:
            self._activate_explore()
        elif old in explore_states:
            self._deactivate_explore()

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

        # ── One-time global costmap clear on first TF availability ────────────
        # Clears any stale 0×0 SLAM map state so Nav2 accepts the real map.
        if not self._global_costmap_initialized:
            if self._global_costmap_clear.service_is_ready():
                self._global_costmap_initialized = True
                self._global_costmap_clear.call_async(ClearEntireCostmap.Request())
                self.get_logger().info(
                    "[STARTUP] Global costmap cleared — map→base_link TF available"
                )

        # ── Global movement watchdog ──────────────────────────────────────────
        # Any state except post-victory POSICIONANDO: no movement for
        # WATCHDOG_REAL_TIMEOUT wall-clock seconds → cancel Nav2 + EXPLORANDO.
        victory_hold = (
            self._estado == "POSICIONANDO_PARA_COLETA" and self._victory_announced
        )
        if self._map_valid and not victory_hold:
            if self._watchdog_pos is None:
                self._watchdog_pos = (rx, ry)
                self._watchdog_last_moved = time.monotonic()
            else:
                moved = math.hypot(
                    rx - self._watchdog_pos[0], ry - self._watchdog_pos[1]
                )
                if moved >= self.WATCHDOG_DIST:
                    self._watchdog_pos = (rx, ry)
                    self._watchdog_last_moved = time.monotonic()
                elif (
                    time.monotonic() - self._watchdog_last_moved
                    >= self.WATCHDOG_REAL_TIMEOUT
                ):
                    recovery = (
                        "PROCURANDO_BANDEIRA"
                        if self._estado == "POSICIONANDO_PARA_COLETA"
                        else "EXPLORANDO"
                    )
                    self.get_logger().warn(
                        f"[WATCHDOG] No movement for "
                        f"{self.WATCHDOG_REAL_TIMEOUT:.0f} s "
                        f"in state {self._estado} — going to {recovery}"
                    )
                    self._cancel_nav2()
                    self._clear_local_costmap()
                    self._transition(recovery)
                    return

        if self._flag_detected:
            fx, fy = self._estimate_flag_map_pos(rx, ry, ryaw)
            if fx is not None:
                self._flag_map_x = fx
                self._flag_map_y = fy
                self._publish_flag_marker()

        # ── State transitions ─────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            if self._flag_detected:
                self._transition("BANDEIRA_DETECTADA")

        elif self._estado == "BANDEIRA_DETECTADA":
            if self._flag_map_x is not None:
                self._transition("NAVEGANDO_BANDEIRA")
            else:
                self.get_logger().warn(
                    "[FSM] Flag seen but no position estimate – retrying"
                )
                self._transition("EXPLORANDO")

        elif self._estado == "NAVEGANDO_BANDEIRA":
            if not self._flag_detected:
                self._cancel_nav2()
                self._procurando_ticks = 0
                self._transition("PROCURANDO_BANDEIRA")
            elif self._flag_map_x is not None:
                dist_to_flag = math.hypot(
                    rx - self._flag_map_x, ry - self._flag_map_y
                )
                if dist_to_flag < self.FLAG_STOP_DISTANCE:
                    self._cancel_nav2()
                    self._transition("POSICIONANDO_PARA_COLETA")

        elif self._estado == "PROCURANDO_BANDEIRA":
            if self._flag_detected:
                self._transition("NAVEGANDO_BANDEIRA")
            else:
                self._procurando_ticks += 1
                if self._procurando_ticks >= self.SEARCH_TICKS_360:
                    self._procurando_ticks = 0
                    self._transition("EXPLORANDO")

        elif self._estado == "POSICIONANDO_PARA_COLETA":
            # Post-victory: never leave.  Pre-victory: Nav2 navigates to the
            # stored flag position — no need to keep the flag in camera view.
            # The watchdog handles genuine stuck cases → PROCURANDO_BANDEIRA.
            pass

        # ── State actions ─────────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            self._do_explorando()
        elif self._estado == "NAVEGANDO_BANDEIRA":
            self._do_navegando(rx, ry)
        elif self._estado == "PROCURANDO_BANDEIRA":
            self._do_procurando()
        elif self._estado == "POSICIONANDO_PARA_COLETA":
            self._do_posicionando(rx, ry)

    # ─────────────────────────────────────────────────────────────────────────
    # STATE ACTIONS
    # ─────────────────────────────────────────────────────────────────────────

    def _do_explorando(self):
        """Send periodic explore_lite keepalive heartbeat."""
        self._explore_heartbeat_ticks += 1
        if self._explore_heartbeat_ticks >= self.EXPLORE_HEARTBEAT_TICKS:
            self._explore_heartbeat_ticks = 0
            self._explore_resume_pub.publish(Bool(data=True))

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
        offset = self.FLAG_STOP_DISTANCE - 0.2   # aim 0.8 m from flag
        if dist > offset:
            gx = self._flag_map_x - (dx / dist) * offset
            gy = self._flag_map_y - (dy / dist) * offset
        else:
            gx, gy = rx, ry

        self._send_nav2_goal(gx, gy, yaw=face_yaw)

    def _do_procurando(self):
        """Keep explore_lite alive while watching for the flag to reappear.

        explore_lite is active in this state so the robot continues navigating;
        no manual cmd_vel is published here.  After SEARCH_TICKS_360 ticks
        without reacquiring the flag, the FSM transitions back to EXPLORANDO.
        """
        self._explore_heartbeat_ticks += 1
        if self._explore_heartbeat_ticks >= self.EXPLORE_HEARTBEAT_TICKS:
            self._explore_heartbeat_ticks = 0
            self._explore_resume_pub.publish(Bool(data=True))

    def _do_posicionando(self, rx: float, ry: float):
        """Stop and extend arm: entered only when < FLAG_STOP_DISTANCE from flag."""
        if self._victory_announced:
            self._extend_arm_capture()
            self._cmd_pub.publish(Twist())
            return

        # First tick in this state — robot is already within FLAG_STOP_DISTANCE.
        self._cancel_nav2()
        self._victory_announced = True
        self._extend_arm_capture()
        self._cmd_pub.publish(Twist())
        self._announce_victory()

    def _publish_flag_marker(self):
        """Publish a blue cylinder in RViz at the estimated flag map position."""
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "flag"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self._flag_map_x
        m.pose.position.y = self._flag_map_y
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = 0.30
        m.scale.y = 0.30
        m.scale.z = 1.0
        m.color.r = 0.0
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 0.9
        # lifetime 0,0 = permanent (never expires)
        self._flag_marker_pub.publish(m)

    def _announce_victory(self):
        sep  = "=" * 62
        sep2 = "*" * 62
        self.get_logger().info(sep2)
        self.get_logger().info(sep)
        self.get_logger().info("")
        self.get_logger().info("        VITORIA!   BANDEIRA CAPTURADA!")
        self.get_logger().info("        BB8 completou a missao com sucesso!")
        self.get_logger().info("")
        self.get_logger().info("        Braco estendido — aguardando coleta.")
        self.get_logger().info("")
        self.get_logger().info(sep)
        self.get_logger().info(sep2)


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
