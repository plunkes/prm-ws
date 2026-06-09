"""
SLAM-only launch.

Starts async_slam_toolbox_node with the project's tuned parameter overrides.
Assumes the Gazebo simulation and robot are already running.

Usage (standalone, sim already up):
  ros2 launch bb8_control slam.launch.py
  ros2 launch bb8_control slam.launch.py verbose:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging",
    )

    verbose    = LaunchConfiguration("verbose")
    log_level  = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    pkg_slam_toolbox = FindPackageShare("slam_toolbox")
    pkg_bb8_control  = FindPackageShare("bb8_control")

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            # Upstream defaults first …
            os.path.join(
                pkg_slam_toolbox.find("slam_toolbox"),
                "config",
                "mapper_params_online_async.yaml",
            ),
            # … then our tuned overrides (base_frame, travel thresholds, etc.)
            PathJoinSubstitution([pkg_bb8_control, "config", "slam_params.yaml"]),
            {"use_sim_time": True},
        ],
        ros_arguments=["--log-level", log_level],
    )

    # Ground-truth odometry: converts /model/prm_robot/pose (Gazebo world-frame
    # pose) into nav_msgs/Odometry on /odom and the odom→base_link TF.
    # Replaces the diff_drive_controller's encoder-based odometry (which drifts).
    odom_gt_node = Node(
        package="bb8_control",
        executable="odom_gt_publisher",
        name="odom_gt_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", log_level],
    )

    # Pads the SLAM /map to full-arena size so Nav2's global costmap is always
    # large enough to plan across the unexplored parts of the arena.
    map_padder_node = Node(
        package="bb8_control",
        executable="map_padder",
        name="map_padder",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, slam_node, odom_gt_node, map_padder_node])
