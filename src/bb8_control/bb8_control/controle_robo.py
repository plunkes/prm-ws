#!/usr/bin/env python3
"""
Master FSM control node.

States (from rubric):
  EXPLORANDO              – reactive LIDAR-based exploration (no Nav2)
  BANDEIRA_DETECTADA      – flag confirmed, cancel exploration
  NAVIGANDO_PARA_BANDEIRA – Nav2 drives to projected flag position
  PROCURANDO_BANDEIRA     – lost flag mid-nav, rotate to reacquire
  POSICIONANDO_PARA_COLETA – final alignment, stop, announce victory

Exploration strategy – memory-infused reactivity:
  Every LIDAR beam is scored by  range × info_gain  where info_gain comes
  from the live SLAM occupancy grid:
    1.0  → unknown cell   (never seen – high exploration value)
    0.05 → free cell      (already mapped – strongly discourage revisit)
    0.0  → occupied cell  (wall – do not go there)
  The desired heading is the weighted-average direction of all beam scores.
  This prevents backtracking without any path planning or extra data structures.

Nav2 is kept active but used ONLY for deliberative flag navigation once the
objective is detected by the semantic segmentation camera.
"""

import math

import numpy as np
import rclpy
import rclpy.executors
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Pose2D, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
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
from std_msgs.msg import Float32, Float64MultiArray


class ControleRoboFSM(Node):

    # ── reactive exploration ──────────────────────────────────────────────────
    CRUISE_SPEED     = 0.7    # m/s  nominal forward speed during exploration
    MIN_SPEED        = 0.15   # m/s  minimum creep (always move if safe)
    KP_ANGULAR       = 2.0    # rad/s per rad – proportional heading-error gain
    MAX_ANGULAR      = 1.5    # rad/s hard cap on reactive angular output
    FRONT_STOP_DIST  = 0.28   # m    stop translating (keep turning) when closer
    EMERGENCY_DIST   = 0.14   # m    full stop + emergency spin if closer

    # ── shared ────────────────────────────────────────────────────────────────
    COLETA_DISTANCE  = 0.45   # m    NAVIGANDO → POSICIONANDO when front ≤ this
    RESEND_TICKS     = 20     # ticks before refreshing Nav2 flag goal
    SEARCH_TICKS_360 = 30     # ticks for ~360° search spin
    ALIGN_THRESHOLD  = math.radians(5)
    ROTATE_SPEED     = 1.5    # rad/s for flag search / POSICIONANDO spin

    def __init__(self):
        super().__init__("controle_robo")

        self._cb_group_timer = ReentrantCallbackGroup()
        self._cb_group_subs  = ReentrantCallbackGroup()

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

        self._tf          = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, "/map", self._cb_map, qos_map,
            callback_group=self._cb_group_subs,
        )
        self.create_subscription(
            Odometry, "/odom", self._cb_odom, qos_sensor,
            callback_group=self._cb_group_subs,
        )
        self.create_subscription(
            LaserScan, "/scan", self._cb_scan, qos_sensor,
            callback_group=self._cb_group_subs,
        )
        self.create_subscription(
            Pose2D, "/vision/flag_detection", self._cb_vision_pose, 10,
            callback_group=self._cb_group_subs,
        )
        self.create_subscription(
            Float32, "/vision/flag_bearing", self._cb_vision_bearing, 10,
            callback_group=self._cb_group_subs,
        )

        # ── publishers ────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Twist, "/diff_drive_base_controller/cmd_vel_unstamped", 10
        )
        self._pub_gripper = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )

        # ── Nav2 – ONLY used for deliberative flag navigation ─────────────────
        self._nav2 = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self._cb_group_timer,
        )

        # ── sensor state (written by callbacks, read by FSM timer) ─────────────
        self._mapa: np.ndarray  = None
        self._map_info          = None
        self._scan: LaserScan   = None
        self._front_dist        = float("inf")
        self._flag_detected     = False
        self._flag_bearing      = 0.0
        self._flag_map_x: float = None
        self._flag_map_y: float = None

        # ── Nav2 bookkeeping (flag navigation only) ────────────────────────────
        self._nav2_handle  = None
        self._nav2_active  = False
        self._nav2_last_xy = None
        self._nav2_ticks   = 0

        # ── arm retraction ─────────────────────────────────────────────────────
        self._arm_retract_sent = 0

        # ── FSM ────────────────────────────────────────────────────────────────
        self._estado            = "EXPLORANDO"
        self._procurando_ticks  = 0
        self._victory_announced = False

        # ── timers ─────────────────────────────────────────────────────────────
        self.create_timer(
            0.2, self._fsm_tick, callback_group=self._cb_group_timer
        )
        self._diag_timer = self.create_timer(
            8.0, self._startup_diagnostic, callback_group=self._cb_group_timer
        )
        # Retract arm every 0.5 s for 10 s; repeating because the gripper
        # controller may not be active at startup.
        self._arm_timer = self.create_timer(
            0.5, self._fold_arm, callback_group=self._cb_group_timer
        )

        self.get_logger().info(
            "ControleRoboFSM started – EXPLORANDO (reactive, map-biased)"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # ARM RETRACTION
    # ─────────────────────────────────────────────────────────────────────────

    def _fold_arm(self):
        msg = Float64MultiArray()
        # Joint order: gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint
        # Shoulder -π/2 (sweep sideways) + elbow -π/2 (fold back) → arm inside body bounds
        msg.data = [-1.5708, -1.5708, 0.0, 0.0]
        self._pub_gripper.publish(msg)
        self._arm_retract_sent += 1
        if self._arm_retract_sent >= 20:
            self._arm_timer.cancel()
            self.get_logger().info("[INIT] Arm retraction complete (20 × 0.5 s)")

    # ─────────────────────────────────────────────────────────────────────────
    # STARTUP DIAGNOSTIC
    # ─────────────────────────────────────────────────────────────────────────

    def _startup_diagnostic(self):
        self._diag_timer.cancel()
        ok = True

        if self._mapa is None:
            self.get_logger().error(
                "[DIAG] /map never received – check slam_toolbox is running "
                "and QoS is compatible.  Try: ros2 topic echo /map --once"
            )
            ok = False
        else:
            self.get_logger().info(
                f"[DIAG] /map OK – shape {self._mapa.shape}, "
                f"res={self._map_info.resolution:.3f} m/cell"
            )

        pose = self._get_map_pose()
        if pose is None:
            self.get_logger().error(
                "[DIAG] TF map→base_link unavailable – check slam_toolbox "
                "and diff_drive_base_controller.  Try: ros2 run tf2_tools view_frames"
            )
            ok = False
        else:
            self.get_logger().info(
                f"[DIAG] TF OK – robot at map ({pose[0]:.2f}, {pose[1]:.2f})"
            )

        if self._scan is None:
            self.get_logger().error(
                "[DIAG] /scan never received – reactive controller cannot run!"
            )
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
        first = self._mapa is None
        self._map_info = msg.info
        self._mapa = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        if first:
            self.get_logger().info(
                f"[MAP] First map received – {msg.info.width}×{msg.info.height} "
                f"@ {msg.info.resolution:.3f} m/cell"
            )

    def _cb_odom(self, msg: Odometry):
        pass  # TF is the primary pose source; odom kept as placeholder

    def _cb_scan(self, msg: LaserScan):
        self._scan = msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = msg.range_max
        n    = len(ranges)
        half = int(math.radians(30) / msg.angle_increment)
        c    = n // 2
        front = ranges[max(0, c - half): min(n, c + half + 1)]
        self._front_dist = float(np.min(front)) if len(front) > 0 else msg.range_max

    def _cb_vision_pose(self, msg: Pose2D):
        # theta > 0.5 is the "flag visible" sentinel from vision_processor
        self._flag_detected = msg.theta > 0.5

    def _cb_vision_bearing(self, msg: Float32):
        self._flag_bearing = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    # COORDINATE HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _get_map_pose(self):
        """Return (x, y, yaw) of base_link in map frame, or None on error."""
        try:
            tf = self._tf.lookup_transform("map", "base_link", rclpy.time.Time())
            t  = tf.transform.translation
            q  = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
            )
            return t.x, t.y, yaw
        except Exception:
            return None

    def _world_to_grid(self, wx, wy):
        if self._map_info is None or self._mapa is None:
            return None
        res = self._map_info.resolution
        gx  = int((wx - self._map_info.origin.position.x) / res)
        gy  = int((wy - self._map_info.origin.position.y) / res)
        h, w = self._mapa.shape
        if 0 <= gx < w and 0 <= gy < h:
            return gx, gy
        return None

    def _grid_to_world(self, gx, gy):
        if self._map_info is None:
            return None
        res = self._map_info.resolution
        ox  = self._map_info.origin.position.x
        oy  = self._map_info.origin.position.y
        return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res

    def _estimate_flag_map_pos(self, rx, ry, ryaw):
        """Project camera bearing + LIDAR range into map-frame coordinates."""
        if self._scan is None:
            return None, None
        s     = self._scan
        angle = self._flag_bearing
        idx   = int(round((angle - s.angle_min) / s.angle_increment))
        idx   = max(0, min(len(s.ranges) - 1, idx))
        dist  = s.ranges[idx]
        if not math.isfinite(dist):
            dist = min(s.range_max, 2.0)
        dist = max(dist, 0.3)
        world_angle = ryaw + angle
        return rx + dist * math.cos(world_angle), ry + dist * math.sin(world_angle)

    # ─────────────────────────────────────────────────────────────────────────
    # NAV2 HELPERS  (flag navigation only)
    # ─────────────────────────────────────────────────────────────────────────

    def _send_nav2_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if not self._nav2.server_is_ready():
            self.get_logger().warn(
                "[Nav2] Action server not ready", throttle_duration_sec=3.0
            )
            return False

        goal_key = (round(x, 1), round(y, 1))
        if self._nav2_active and self._nav2_last_xy == goal_key:
            return True  # same goal already in flight

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id        = "map"
        goal.pose.header.stamp           = self.get_clock().now().to_msg()
        goal.pose.pose.position.x        = float(x)
        goal.pose.pose.position.y        = float(y)
        goal.pose.pose.orientation.w     = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z     = math.sin(yaw / 2.0)

        self._nav2_active  = True
        self._nav2_last_xy = goal_key
        self._nav2_ticks   = 0
        future = self._nav2.send_goal_async(goal)
        future.add_done_callback(self._on_nav2_accepted)
        self.get_logger().info(f"[Nav2] goal → ({x:.2f}, {y:.2f})")
        return True

    def _on_nav2_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("[Nav2] goal REJECTED")
            self._nav2_active  = False
            self._nav2_last_xy = None
            return
        self._nav2_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav2_done)

    def _on_nav2_done(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("[Nav2] goal SUCCEEDED")
        elif status == 5:
            self.get_logger().info("[Nav2] goal cancelled")
        else:
            self.get_logger().warn(f"[Nav2] goal ABORTED (status={status})")
        self._nav2_active  = False
        self._nav2_handle  = None
        self._nav2_last_xy = None

    def _cancel_nav2(self):
        if self._nav2_handle is not None:
            self._nav2_handle.cancel_goal_async()
            self._nav2_handle = None
        self._nav2_active  = False
        self._nav2_last_xy = None

    # ─────────────────────────────────────────────────────────────────────────
    # FSM
    # ─────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        if self._estado != new_state:
            self.get_logger().info(f"[FSM] {self._estado} → {new_state}")
            self._estado = new_state

    def _fsm_tick(self):
        pose = self._get_map_pose()
        if pose is None:
            self.get_logger().warn(
                "Waiting for TF map→base_link…", throttle_duration_sec=3.0
            )
            return
        rx, ry, ryaw = pose

        self._nav2_ticks += 1

        # Keep flag world-position estimate fresh whenever the camera sees it
        if self._flag_detected:
            fx, fy = self._estimate_flag_map_pos(rx, ry, ryaw)
            if fx is not None:
                self._flag_map_x = fx
                self._flag_map_y = fy

        # ── STATE TRANSITIONS ─────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            if self._flag_detected:
                self._transition("BANDEIRA_DETECTADA")

        elif self._estado == "BANDEIRA_DETECTADA":
            if self._flag_map_x is not None:
                self._transition("NAVIGANDO_PARA_BANDEIRA")
            else:
                self.get_logger().warn(
                    "[FSM] Flag detected but no position estimate yet – retrying"
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

        # ── STATE ACTIONS ─────────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            self._do_explorando(rx, ry, ryaw)

        elif self._estado == "NAVIGANDO_PARA_BANDEIRA":
            self._do_navegando(rx, ry)

        elif self._estado == "PROCURANDO_BANDEIRA":
            self._do_procurando()

        elif self._estado == "POSICIONANDO_PARA_COLETA":
            self._do_posicionando()

    # ─────────────────────────────────────────────────────────────────────────
    # STATE ACTIONS
    # ─────────────────────────────────────────────────────────────────────────

    def _do_explorando(self, rx: float, ry: float, ryaw: float):
        """Pure reactive LIDAR exploration – no Nav2, no path planning."""
        if self._nav2_active:
            self._cancel_nav2()

        v, w = self._compute_reactive_cmd(rx, ry, ryaw)
        twist = Twist()
        twist.linear.x  = v
        twist.angular.z = w
        self._cmd_pub.publish(twist)

    def _compute_reactive_cmd(self, rx: float, ry: float, ryaw: float):
        """
        Score every LIDAR beam by  range × info_gain  then steer toward the
        weighted-average direction.

        info_gain table (from SLAM occupancy grid):
          -1  (unknown / not yet seen) → 1.00  ← highest exploration value
          0–15 (free / already mapped) → 0.05  ← strongly discourage revisit
          >50  (occupied / wall)       → 0.00  ← never approach

        Desired heading = atan2(Σ score·sin θ, Σ score·cos θ) in robot frame.
        Angular control: proportional to heading error.
        Linear control: scale down with turn rate, floor at MIN_SPEED.
        """
        s = self._scan
        if s is None:
            return 0.0, self.ROTATE_SPEED

        ranges = np.array(s.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, s.range_max)
        ranges = np.clip(ranges, 0.0, s.range_max)

        n           = len(ranges)
        beam_angles = (
            s.angle_min + np.arange(n, dtype=np.float32) * s.angle_increment
        )

        # --- Info-gain from SLAM occupancy grid ---
        info_gain = np.ones(n, dtype=np.float32)  # safe default: no map yet

        if self._mapa is not None and self._map_info is not None:
            res = self._map_info.resolution
            ox  = self._map_info.origin.position.x
            oy  = self._map_info.origin.position.y
            H, W = self._mapa.shape

            # Probe 0.30 m beyond the beam endpoint (peek into the next cell).
            # Clamp to 97 % of range_max so we never probe past the sensor horizon.
            probe_dist   = np.minimum(ranges * 0.85 + 0.30, s.range_max * 0.97)
            world_angles = ryaw + beam_angles

            px = rx + probe_dist * np.cos(world_angles)
            py = ry + probe_dist * np.sin(world_angles)

            gx = np.floor((px - ox) / res).astype(np.int32)
            gy = np.floor((py - oy) / res).astype(np.int32)

            in_map = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
            gx_c   = np.clip(gx, 0, W - 1)
            gy_c   = np.clip(gy, 0, H - 1)

            # int8 occupancy: −1=unknown, 0–15=free, >50=occupied
            cell_vals = self._mapa[gy_c, gx_c].astype(np.int16)

            info_gain = np.where(
                ~in_map,            1.0,   # outside map bounds → treat as unknown
                np.where(cell_vals < 0,   1.0,    # unknown  → max info gain
                np.where(cell_vals <= 15, 0.05,   # free/visited → strongly discouraged
                                          0.0))   # occupied → avoid
            ).astype(np.float32)

        # --- Beam score ---
        scores = ranges * info_gain

        # Ignore the rear ±30° to prevent accidental reverse driving
        scores[np.abs(beam_angles) > math.radians(150)] = 0.0

        # Suppress beams that are essentially touching a surface
        scores[ranges < 0.12] = 0.0

        # --- Desired heading: weighted directional sum ---
        dx_sum = float(np.dot(scores, np.cos(beam_angles)))
        dy_sum = float(np.dot(scores, np.sin(beam_angles)))

        if dx_sum == 0.0 and dy_sum == 0.0:
            # Completely surrounded – spin in place until a direction opens up
            return 0.0, self.ROTATE_SPEED

        desired = math.atan2(dy_sum, dx_sum)  # heading error in robot frame

        # --- Angular control (proportional) ---
        angular_z = float(
            np.clip(self.KP_ANGULAR * desired, -self.MAX_ANGULAR, self.MAX_ANGULAR)
        )

        # --- Linear control ---
        if self._front_dist < self.EMERGENCY_DIST:
            # Imminent collision: stop everything and spin hard
            return 0.0, math.copysign(self.ROTATE_SPEED, angular_z)

        if self._front_dist < self.FRONT_STOP_DIST:
            # Close to wall: rotate only, do not advance
            return 0.0, angular_z

        # Normal motion: scale linear speed down with turn sharpness
        turn_fraction = abs(angular_z) / self.MAX_ANGULAR
        linear_x = max(self.MIN_SPEED, self.CRUISE_SPEED * (1.0 - turn_fraction))
        return float(linear_x), angular_z

    def _do_navegando(self, rx: float, ry: float):
        """Nav2 goal toward a point COLETA_DISTANCE + margin in front of the flag."""
        if self._flag_map_x is None:
            return
        if self._nav2_active and self._nav2_ticks < self.RESEND_TICKS:
            return

        dx   = self._flag_map_x - rx
        dy   = self._flag_map_y - ry
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return

        face_yaw = math.atan2(dy, dx)
        offset   = self.COLETA_DISTANCE + 0.2
        if dist > offset:
            gx = self._flag_map_x - (dx / dist) * offset
            gy = self._flag_map_y - (dy / dist) * offset
        else:
            gx, gy = rx, ry  # already close; LIDAR will trigger POSICIONANDO

        self._send_nav2_goal(gx, gy, yaw=face_yaw)

    def _do_procurando(self):
        """Rotate in place to reacquire the lost flag."""
        twist = Twist()
        twist.angular.z = self.ROTATE_SPEED
        self._cmd_pub.publish(twist)

    def _do_posicionando(self):
        """
        Proportional bearing alignment.  Once the flag is centred in the camera
        frame (bearing < ALIGN_THRESHOLD), stop and announce victory.
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
            # Robot stops: twist stays at zero

        self._cmd_pub.publish(twist)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
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
