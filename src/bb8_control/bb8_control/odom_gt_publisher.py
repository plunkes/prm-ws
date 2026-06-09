#!/usr/bin/env python3
"""
Ground-truth odometry publisher.

Converts Gazebo's model pose (world frame, perfect) into a nav_msgs/Odometry
message on /odom and the odom→base_link TF, replacing the encoder-based
diff_drive_controller odometry which accumulates drift over time.

Positions are published RELATIVE to the robot's spawn pose so the odom frame
origin is (0, 0) at startup — the same convention used by the wheel-encoder
diff_drive_controller.  SLAM Toolbox and Nav2 both assume the odom frame starts
at (0, 0); publishing raw Gazebo world coordinates (e.g. x=-8, y=-0.5) places
the robot outside the initial costmap bounds and breaks planning.

Requires:
  - /model/prm_robot/pose (geometry_msgs/Pose) from the ros_gz_bridge
  - publish_odom: false and enable_odom_tf: false in controller_config.yaml
    so the diff_drive_controller does not conflict with this node's TF.
"""

import math

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomGTPublisher(Node):
    def __init__(self):
        super().__init__("odom_gt_publisher")

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # First received pose becomes the odom frame origin.
        self._initial_pose: Pose | None = None
        self._prev_pose: Pose | None = None
        self._prev_stamp = None

        self.create_subscription(
            Pose, "/model/prm_robot/pose", self._cb_pose, 10
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # ── callback ──────────────────────────────────────────────────────────────

    def _cb_pose(self, msg: Pose) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Record the spawn pose so all subsequent positions are relative to it.
        if self._initial_pose is None:
            self._initial_pose = msg
            self.get_logger().info(
                f"[odom_gt] Origin set to world ({msg.position.x:.2f}, "
                f"{msg.position.y:.2f}) — odom frame starts at (0, 0)."
            )

        # ── Position relative to spawn (odom frame origin = spawn position) ───
        rel_x = msg.position.x - self._initial_pose.position.x
        rel_y = msg.position.y - self._initial_pose.position.y

        # ── Body-frame velocity from consecutive GT pose differences ──────────
        vx_body = vy_body = wz = 0.0
        if self._prev_pose is not None and self._prev_stamp is not None:
            dt = (now - self._prev_stamp).nanoseconds * 1e-9
            if dt > 1e-6:
                dx = msg.position.x - self._prev_pose.position.x
                dy = msg.position.y - self._prev_pose.position.y

                yaw = self._yaw_from_quaternion(msg.orientation)
                vx_body = (dx * math.cos(yaw) + dy * math.sin(yaw)) / dt
                vy_body = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / dt

                yaw_prev = self._yaw_from_quaternion(self._prev_pose.orientation)
                d_yaw = math.atan2(
                    math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev)
                )
                wz = d_yaw / dt

        self._prev_pose = msg
        self._prev_stamp = now

        # ── odom→base_link TF ─────────────────────────────────────────────────
        # z=0: project to ground plane for 2D SLAM/Nav2; prevents the
        # occupancy grid from appearing displaced vertically in RViz.
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = rel_x
        tf.transform.translation.y = rel_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = msg.orientation
        self._tf_broadcaster.sendTransform(tf)

        # ── /odom ─────────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = rel_x
        odom.pose.pose.position.y = rel_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = msg.orientation
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGTPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
