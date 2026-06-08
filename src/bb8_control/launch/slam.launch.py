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

    return LaunchDescription([declare_verbose, slam_node])
