#!/usr/bin/env python3
"""
Master FSM control node.

States (from rubric):
  EXPLORANDO              – frontier-based exploration via Nav2 goals
  BANDEIRA_DETECTADA      – flag confirmed, cancel exploration
  NAVIGANDO_PARA_BANDEIRA – Nav2 navigates to projected flag position
  PROCURANDO_BANDEIRA     – lost flag mid-nav, rotate to reacquire
  POSICIONANDO_PARA_COLETA – final alignment facing flag at safe distance

Uses MultiThreadedExecutor so subscription callbacks are never blocked by
the 5 Hz FSM timer (important: frontier search can take tens of ms).
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64MultiArray

from bb8_control.explorador import ExploradorFronteiras


class ControleRoboFSM(Node):

    # ── constants ────────────────────────────────────────────────────────────────
    COLETA_DISTANCE = 0.45       # m  – NAVIGANDO → POSICIONANDO when LIDAR ≤ this
    RESEND_TICKS = 20            # ticks before picking a new frontier (4 s @ 5 Hz)
    SEARCH_TICKS_360 = 30        # ticks for ~360° rotation (6 s @ 5 Hz ÷ 1.2 rad/s)
    ALIGN_THRESHOLD = math.radians(5)
    ROTATE_SPEED = 1.2           # rad/s – fast spin to reacquire flag / escape obstacles
    FRONT_OBSTACLE_DIST = 0.40   # m  – rotate in place when wall is closer than this
    STUCK_GOAL_FAILS = 3         # consecutive Nav2 aborts before forcing a turnaround
    TURNAROUND_TICKS = 20        # ticks of forced rotation after being stuck (~210° @ 1.2 rad/s)

    def __init__(self):
        super().__init__("controle_robo")

        # ── callback groups (needed for MultiThreadedExecutor) ────────────────────
        # Timer and action callbacks share the reentrant group so they don't deadlock.
        self._cb_group_timer = ReentrantCallbackGroup()
        self._cb_group_subs = ReentrantCallbackGroup()

        # ── QoS ──────────────────────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # /map: slam_toolbox publishes RELIABLE + TRANSIENT_LOCAL.
        # We match exactly so late subscribers get the last retained message.
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── TF ───────────────────────────────────────────────────────────────────
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        # ── subscribers ──────────────────────────────────────────────────────────
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

        # ── publisher (only for PROCURANDO + POSICIONANDO – no Nav2 in these states) ──
        self._cmd_pub = self.create_publisher(
            Twist, "/diff_drive_base_controller/cmd_vel_unstamped", 10
        )
        # Gripper controller – retract arm on startup so it doesn't hit walls
        self._pub_gripper = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )

        # ── Nav2 action client ───────────────────────────────────────────────────
        self._nav2 = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self._cb_group_timer,
        )

        # ── exploration ──────────────────────────────────────────────────────────
        self._explorador = ExploradorFronteiras()

        # ── sensor state (written by callbacks, read by timer) ────────────────────
        self._mapa: np.ndarray = None
        self._map_info = None
        self._scan: LaserScan = None
        self._front_dist = float("inf")
        self._flag_detected = False
        self._flag_bearing = 0.0     # rad, robot frame (+ = left)
        self._flag_map_x: float = None
        self._flag_map_y: float = None

        # ── Nav2 bookkeeping ─────────────────────────────────────────────────────
        self._nav2_handle = None
        self._nav2_active = False
        self._nav2_last_xy = None
        self._nav2_ticks = 0         # ticks since last goal send
        self._nav2_fail_count = 0    # consecutive aborted goals
        self._turnaround_remaining = 0  # ticks of forced rotation left

        # ── arm retraction bookkeeping ───────────────────────────────────────────
        self._arm_retract_sent = 0

        # ── FSM ──────────────────────────────────────────────────────────────────
        self._estado = "EXPLORANDO"
        self._procurando_ticks = 0

        # ── timers ───────────────────────────────────────────────────────────────
        self.create_timer(
            0.2, self._fsm_tick, callback_group=self._cb_group_timer
        )
        # One-shot diagnostic fires 8 s after startup
        self._diag_timer = self.create_timer(
            8.0, self._startup_diagnostic, callback_group=self._cb_group_timer
        )
        # Retract arm every 0.5 s for 10 s total (20 publishes).
        # Repeating is necessary because the gripper controller may not be active
        # at the exact moment of a one-shot publish.
        self._arm_timer = self.create_timer(
            0.5, self._fold_arm, callback_group=self._cb_group_timer
        )

        self.get_logger().info("ControleRoboFSM started – state: EXPLORANDO")

    # ─────────────────────────────────────────────────────────────────────────────
    # ARM RETRACTION
    # ─────────────────────────────────────────────────────────────────────────────

    def _fold_arm(self):
        """Retract arm sideways so it cannot block the LIDAR or hit walls."""
        msg = Float64MultiArray()
        # Joint order matches controller_config.yaml joints list:
        # gripper_extension, right_gripper_joint, left_gripper_joint
        msg.data = [-1.5708, 0.0, 0.0]
        self._pub_gripper.publish(msg)
        self._arm_retract_sent += 1
        if self._arm_retract_sent >= 20:
            self._arm_timer.cancel()
            self.get_logger().info("[INIT] Arm retraction complete (sent 20× over 10 s)")

    # ─────────────────────────────────────────────────────────────────────────────
    # ONE-SHOT STARTUP DIAGNOSTIC
    # ─────────────────────────────────────────────────────────────────────────────

    def _startup_diagnostic(self):
        """
        Fires once 8 seconds after startup.  Logs the exact reason the robot
        is not moving so the problem can be diagnosed without guessing.
        """
        self._diag_timer.cancel()
        ok = True
        if self._mapa is None:
            self.get_logger().error(
                "[DIAG] /map never received!  "
                "Check slam_toolbox is running and QoS is compatible.  "
                "Try: ros2 topic echo /map --once"
            )
            ok = False
        else:
            self.get_logger().info(
                f"[DIAG] /map OK – shape {self._mapa.shape}, "
                f"resolution {self._map_info.resolution:.3f} m/cell"
            )

        pose = self._get_map_pose()
        if pose is None:
            self.get_logger().error(
                "[DIAG] TF map→base_link unavailable!  "
                "Check slam_toolbox (provides map→odom) and "
                "diff_drive_base_controller (provides odom→base_link).  "
                "Try: ros2 run tf2_tools view_frames"
            )
            ok = False
        else:
            self.get_logger().info(f"[DIAG] TF OK – robot at map ({pose[0]:.2f}, {pose[1]:.2f})")

        if not self._nav2.server_is_ready():
            self.get_logger().error(
                "[DIAG] Nav2 navigate_to_pose action server NOT ready!  "
                "Check that nav2.launch.py is running and bt_navigator is ACTIVE.  "
                "Try: ros2 action list"
            )
            ok = False
        else:
            self.get_logger().info("[DIAG] Nav2 action server OK")

        if self._scan is None:
            self.get_logger().warn("[DIAG] /scan never received – LIDAR not publishing")
        else:
            self.get_logger().info("[DIAG] /scan OK")

        if ok:
            self.get_logger().info("[DIAG] All systems GO – robot should be moving")

    # ─────────────────────────────────────────────────────────────────────────────
    # SENSOR CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────────

    def _cb_map(self, msg: OccupancyGrid):
        first = self._mapa is None
        self._map_info = msg.info
        self._mapa = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        if first:
            self.get_logger().info(
                f"[MAP] First /map received – {msg.info.width}×{msg.info.height} cells, "
                f"res={msg.info.resolution:.3f} m"
            )

    def _cb_odom(self, msg: Odometry):
        pass  # only used as a fallback; TF is the primary pose source

    def _cb_scan(self, msg: LaserScan):
        self._scan = msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = msg.range_max
        n = len(ranges)
        half = int(math.radians(30) / msg.angle_increment)
        c = n // 2
        front = ranges[max(0, c - half): min(n, c + half + 1)]
        self._front_dist = float(np.min(front)) if len(front) > 0 else msg.range_max

    def _cb_vision_pose(self, msg: Pose2D):
        self._flag_detected = msg.theta > 0.5

    def _cb_vision_bearing(self, msg: Float32):
        self._flag_bearing = msg.data

    # ─────────────────────────────────────────────────────────────────────────────
    # COORDINATE HELPERS
    # ─────────────────────────────────────────────────────────────────────────────

    def _get_map_pose(self):
        """Return (x, y, yaw) of base_link in map frame, or None on any error."""
        try:
            tf = self._tf.lookup_transform("map", "base_link", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
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
        gx = int((wx - self._map_info.origin.position.x) / res)
        gy = int((wy - self._map_info.origin.position.y) / res)
        h, w = self._mapa.shape
        if 0 <= gx < w and 0 <= gy < h:
            return gx, gy
        return None

    def _grid_to_world(self, gx, gy):
        if self._map_info is None:
            return None
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res

    def _estimate_flag_map_pos(self, rx, ry, ryaw):
        """
        Project camera bearing + LIDAR range at that bearing into map frame.
        """
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

    # ─────────────────────────────────────────────────────────────────────────────
    # NAV2 HELPERS
    # ─────────────────────────────────────────────────────────────────────────────

    def _send_nav2_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        Send a NavigateToPose goal.  Returns True if the request was dispatched.
        Non-blocking: uses send_goal_async.
        """
        if not self._nav2.server_is_ready():
            self.get_logger().warn(
                "[Nav2] Action server not ready – waiting…",
                throttle_duration_sec=3.0,
            )
            return False

        goal_key = (round(x, 1), round(y, 1))
        if self._nav2_active and self._nav2_last_xy == goal_key:
            return True  # same goal already in flight, nothing to do

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
            self.get_logger().warn("[Nav2] goal REJECTED by bt_navigator")
            self._nav2_active = False
            self._nav2_last_xy = None
            return
        self._nav2_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav2_done)

    def _on_nav2_done(self, future):
        # status: 4=SUCCEEDED, 5=CANCELED (by us), 6=ABORTED (Nav2 gave up)
        status = future.result().status
        if status == 4:  # succeeded
            self.get_logger().info("[Nav2] goal SUCCEEDED")
            self._nav2_fail_count = 0
        elif status == 5:  # cancelled by us intentionally
            self.get_logger().info("[Nav2] goal cancelled")
        else:  # aborted – Nav2 could not reach the goal
            self.get_logger().warn(f"[Nav2] goal ABORTED (status={status})")
            if self._estado == "EXPLORANDO":
                self._nav2_fail_count += 1
                self.get_logger().warn(
                    f"[Nav2] consecutive failures: {self._nav2_fail_count}/{self.STUCK_GOAL_FAILS}"
                )
                if self._nav2_fail_count >= self.STUCK_GOAL_FAILS:
                    self._nav2_fail_count = 0
                    self._turnaround_remaining = self.TURNAROUND_TICKS
                    self.get_logger().warn(
                        "[Nav2] STUCK detected – forcing turnaround rotation"
                    )
        self._nav2_active = False
        self._nav2_handle = None
        self._nav2_last_xy = None

    def _cancel_nav2(self):
        if self._nav2_handle is not None:
            self._nav2_handle.cancel_goal_async()
            self._nav2_handle = None
        self._nav2_active = False
        self._nav2_last_xy = None

    # ─────────────────────────────────────────────────────────────────────────────
    # FSM
    # ─────────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        if self._estado != new_state:
            self.get_logger().info(f"[FSM] {self._estado} → {new_state}")
            self._estado = new_state

    def _fsm_tick(self):
        # ── pre-conditions ───────────────────────────────────────────────────────
        if self._mapa is None:
            self.get_logger().warn(
                "Waiting for /map…", throttle_duration_sec=3.0
            )
            return

        pose = self._get_map_pose()
        if pose is None:
            self.get_logger().warn(
                "Waiting for TF map→base_link…", throttle_duration_sec=3.0
            )
            return
        rx, ry, ryaw = pose

        robot_grid = self._world_to_grid(rx, ry)
        if robot_grid is None:
            self.get_logger().warn(
                f"Robot pose ({rx:.2f},{ry:.2f}) is outside map bounds",
                throttle_duration_sec=3.0,
            )
            return

        self._nav2_ticks += 1

        # Refresh flag world position whenever visible
        if self._flag_detected:
            fx, fy = self._estimate_flag_map_pos(rx, ry, ryaw)
            if fx is not None:
                self._flag_map_x = fx
                self._flag_map_y = fy

        # ── STATE TRANSITIONS ────────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            if self._flag_detected:
                self._cancel_nav2()
                self._transition("BANDEIRA_DETECTADA")

        elif self._estado == "BANDEIRA_DETECTADA":
            # One-tick transient: advance immediately
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

        # ── STATE ACTIONS ────────────────────────────────────────────────────────

        if self._estado == "EXPLORANDO":
            self._do_explorando(robot_grid, ryaw)

        elif self._estado == "NAVIGANDO_PARA_BANDEIRA":
            self._do_navegando(rx, ry)

        elif self._estado == "PROCURANDO_BANDEIRA":
            self._do_procurando()

        elif self._estado == "POSICIONANDO_PARA_COLETA":
            self._do_posicionando()

    # ── state actions ─────────────────────────────────────────────────────────────

    def _do_explorando(self, robot_grid, ryaw):
        """Pick the best frontier and forward it to Nav2."""
        # ── Forced turnaround after repeated Nav2 failures ────────────────────────
        if self._turnaround_remaining > 0:
            self._cancel_nav2()
            twist = Twist()
            twist.angular.z = self.ROTATE_SPEED
            self._cmd_pub.publish(twist)
            self._turnaround_remaining -= 1
            if self._turnaround_remaining == 0:
                self.get_logger().info("[EXPLORANDO] Turnaround done – resuming exploration")
            return

        # ── Wall directly in front and Nav2 has no active goal → rotate away ────
        # Only override when Nav2 is idle; while Nav2 is actively driving let it
        # handle the path – if it fails, the failure counter above will trigger.
        if not self._nav2_active and self._front_dist < self.FRONT_OBSTACLE_DIST:
            twist = Twist()
            twist.angular.z = self.ROTATE_SPEED
            self._cmd_pub.publish(twist)
            self.get_logger().warn(
                f"[EXPLORANDO] Wall at {self._front_dist:.2f} m – rotating away",
                throttle_duration_sec=1.0,
            )
            return

        # ── Normal frontier exploration ───────────────────────────────────────────
        if self._nav2_active and self._nav2_ticks < self.RESEND_TICKS:
            return

        frontier = self._explorador.encontrar_alvo_desconhecido(
            self._mapa, robot_grid, ryaw
        )
        if frontier is None:
            self.get_logger().warn(
                "[EXPLORANDO] No frontiers found – map may be fully explored",
                throttle_duration_sec=5.0,
            )
            return

        world = self._grid_to_world(frontier[0], frontier[1])
        if world is None:
            return

        self._send_nav2_goal(world[0], world[1])

    def _do_navegando(self, rx, ry):
        """Navigate to a point slightly in front of the flag."""
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
        # Stop COLETA_DISTANCE + margin in front of the flag,
        # not at the flag's exact cell (which may be an obstacle in the costmap)
        offset = self.COLETA_DISTANCE + 0.2
        if dist > offset:
            gx = self._flag_map_x - (dx / dist) * offset
            gy = self._flag_map_y - (dy / dist) * offset
        else:
            gx, gy = rx, ry  # already close; LIDAR will trigger transition

        self._send_nav2_goal(gx, gy, yaw=face_yaw)

    def _do_procurando(self):
        """Rotate in place searching for the lost flag."""
        twist = Twist()
        twist.angular.z = self.ROTATE_SPEED
        self._cmd_pub.publish(twist)

    def _do_posicionando(self):
        """Proportional alignment: centre the flag in the camera frame."""
        twist = Twist()
        if abs(self._flag_bearing) > self.ALIGN_THRESHOLD:
            twist.angular.z = self.ROTATE_SPEED * math.copysign(1.0, self._flag_bearing)
        else:
            self.get_logger().info(
                "[FSM] Flag centred – POSICIONANDO_PARA_COLETA complete",
                throttle_duration_sec=5.0,
            )
        self._cmd_pub.publish(twist)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ControleRoboFSM()
    # MultiThreadedExecutor: subscription callbacks never blocked by the FSM timer.
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
