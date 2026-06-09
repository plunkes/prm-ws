#!/usr/bin/env python3
"""
Map padder relay.

Subscribes to /map (SLAM Toolbox) and republishes /map_padded — a fixed-size
OccupancyGrid that always covers the full arena with unknown cells beyond the
currently explored area.

Why this exists
---------------
SLAM Toolbox sizes its published map to the bounding-box of explored scan
points.  Nav2's StaticLayer calls layered_costmap_->resizeMap() to match every
incoming map, so the global costmap always equals the current SLAM coverage.
This prevents the NavFn planner from routing the robot PAST the explored area
into unmapped territory, breaking frontier exploration.

By serving a constant-size padded map, the static layer never needs to resize:
cells inside the SLAM coverage receive SLAM values; cells outside remain
NO_INFORMATION (-1).  The planner already has allow_unknown=true, so it plans
freely through the padded unknown region.

Pad dimensions (map / odom frame, origin ≈ robot spawn at world (-8, -0.5)):
  origin (-3.0, -5.0) + 24 m × 12 m
  → x: -3 to +21  covers arena walls at odom +17.1 with 3.9 m margin
  → y: -5 to  +7  covers arena walls at odom +4.6 / -3.6 with ample margins
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapPadder(Node):
    PAD_ORIGIN_X = -3.0
    PAD_ORIGIN_Y = -5.0
    PAD_WIDTH_M  = 24.0
    PAD_HEIGHT_M = 12.0
    RESOLUTION   = 0.05   # must match SLAM Toolbox resolution (default 0.05)

    def __init__(self):
        super().__init__("map_padder")
        self._pad_w = int(self.PAD_WIDTH_M  / self.RESOLUTION)   # 480 cells
        self._pad_h = int(self.PAD_HEIGHT_M / self.RESOLUTION)   # 240 cells

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(OccupancyGrid, "/map_padded", qos)
        self.create_subscription(OccupancyGrid, "/map", self._cb, qos)

        self.get_logger().info(
            f"[map_padder] pad {self._pad_w}×{self._pad_h} cells "
            f"@ origin ({self.PAD_ORIGIN_X}, {self.PAD_ORIGIN_Y})"
        )

    def _cb(self, msg: OccupancyGrid) -> None:
        out_arr = np.full((self._pad_h, self._pad_w), -1, dtype=np.int8)

        if msg.info.width > 0 and msg.info.height > 0:
            slam_res = msg.info.resolution
            if abs(slam_res - self.RESOLUTION) > 1e-4:
                self.get_logger().warn(
                    f"[map_padder] SLAM resolution {slam_res:.4f} != "
                    f"pad {self.RESOLUTION:.4f}; skipping SLAM overlay"
                )
            else:
                dx = int(round(
                    (msg.info.origin.position.x - self.PAD_ORIGIN_X) / self.RESOLUTION))
                dy = int(round(
                    (msg.info.origin.position.y - self.PAD_ORIGIN_Y) / self.RESOLUTION))

                slam_arr = np.array(msg.data, dtype=np.int8).reshape(
                    (msg.info.height, msg.info.width))

                cx0 = max(0, dx)
                cy0 = max(0, dy)
                cx1 = min(self._pad_w, dx + int(msg.info.width))
                cy1 = min(self._pad_h, dy + int(msg.info.height))

                if cx0 < cx1 and cy0 < cy1:
                    sx0, sy0 = cx0 - dx, cy0 - dy
                    out_arr[cy0:cy1, cx0:cx1] = slam_arr[sy0:sy0+(cy1-cy0),
                                                          sx0:sx0+(cx1-cx0)]

        out = OccupancyGrid()
        out.header = msg.header
        out.info.resolution = self.RESOLUTION
        out.info.width  = self._pad_w
        out.info.height = self._pad_h
        out.info.origin.position.x = self.PAD_ORIGIN_X
        out.info.origin.position.y = self.PAD_ORIGIN_Y
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = out_arr.flatten().tolist()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapPadder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
