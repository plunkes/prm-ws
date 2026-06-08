#!/usr/bin/env python3
"""
Vision processing node – semantic segmentation → flag detection.

Subscribes:
  /robot_cam/labels_map  (sensor_msgs/Image)  – Gazebo semantic label map

Publishes:
  /vision/flag_detection  (geometry_msgs/Pose2D)
      .x     = centroid column (pixels)
      .y     = centroid row    (pixels)
      .theta = 1.0 if flag visible, 0.0 otherwise

  /vision/flag_bearing  (std_msgs/Float32)
      Bearing from robot forward axis to flag centre, in radians.
      Positive = flag is to robot's LEFT.
      Zero when flag is not detected.

Parameters:
  flag_label_id    (int,   default 1)    – semantic label of the flag
  camera_hfov_deg  (float, default 80.0) – horizontal field of view of the camera
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class VisionProcessorNode(Node):

    def __init__(self):
        super().__init__("vision_processor")

        self.declare_parameter("flag_label_id", 1)
        self.declare_parameter("camera_hfov_deg", 90.0)  # matches URDF horizontal_fov=1.57 rad

        self._flag_label = self.get_parameter("flag_label_id").value
        hfov_deg = self.get_parameter("camera_hfov_deg").value
        self._hfov = math.radians(hfov_deg)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Gazebo semantic segmentation publishes the label map here
        self.create_subscription(
            Image, "/robot_cam/labels_map", self._cb_labels, qos
        )

        self._pub_detection  = self.create_publisher(Pose2D,  "/vision/flag_detection", 10)
        self._pub_bearing    = self.create_publisher(Float32, "/vision/flag_bearing",   10)
        # Scene classification: "objective" | "obstacle" | "clear"
        self._pub_scene_class = self.create_publisher(String, "/vision/scene_class", 10)

        self._bridge = CvBridge()
        self.get_logger().info(
            f"VisionProcessor started | flag_label={self._flag_label} "
            f"HFOV={hfov_deg:.0f}°"
        )

    def _cb_labels(self, msg: Image):
        try:
            img = self._decode_label_image(msg)
        except Exception as exc:
            self.get_logger().error(f"Image decode error: {exc}", throttle_duration_sec=2.0)
            return

        if img is None:
            return

        flag_mask = img == self._flag_label
        detected = bool(np.any(flag_mask))

        pose_msg = Pose2D()
        bearing_msg = Float32()

        if detected:
            ys, xs = np.where(flag_mask)
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))
            img_w = float(img.shape[1])

            # Bearing: positive CCW (robot-left). Image x increases rightward,
            # so a centroid left of centre means flag is to the robot's left.
            bearing = (img_w / 2.0 - cx) / (img_w / 2.0) * (self._hfov / 2.0)

            pose_msg.x = cx
            pose_msg.y = cy
            pose_msg.theta = 1.0
            bearing_msg.data = float(bearing)

            area = int(np.sum(flag_mask))
            self.get_logger().info(
                f"Flag @ ({cx:.0f}, {cy:.0f})px  bearing={math.degrees(bearing):.1f}°  area={area}px",
                throttle_duration_sec=1.0,
            )
            scene_class = "objective"
        else:
            pose_msg.theta = 0.0
            bearing_msg.data = 0.0
            # Any labelled pixel that is not the flag is an obstacle
            obstacle_detected = bool(np.any((img > 0) & ~flag_mask))
            scene_class = "obstacle" if obstacle_detected else "clear"

        self._pub_detection.publish(pose_msg)
        self._pub_bearing.publish(bearing_msg)
        self._pub_scene_class.publish(String(data=scene_class))

    def _decode_label_image(self, msg: Image):
        """
        Decode the label map into a 2-D uint16/uint8 numpy array where each
        pixel holds its semantic class ID.

        Gazebo semantic cameras typically output one of:
          - mono8  / 8UC1  : 8-bit label IDs
          - mono16 / 16UC1 : 16-bit label IDs
          - rgb8           : class colour map (not label IDs directly)
        """
        enc = msg.encoding.lower()

        if enc in ("mono8", "8uc1"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        if enc in ("mono16", "16uc1"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")

        if enc in ("rgb8", "bgr8"):
            # Gazebo may encode labels as colours: map each unique colour → ID
            # by treating the R channel as the label byte.
            colour_img = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="rgb8" if enc == "rgb8" else "bgr8"
            )
            # Use the red channel (index 0 for rgb8, index 2 for bgr8)
            ch = 0 if enc == "rgb8" else 2
            return colour_img[:, :, ch].astype(np.uint16)

        # Fallback: let cv_bridge decide and collapse to single channel
        img = self._bridge.imgmsg_to_cv2(msg)
        if img.ndim == 3:
            img = img[:, :, 0]
        return img.astype(np.uint16)


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
