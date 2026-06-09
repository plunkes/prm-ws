"""
explore_lite launch.

Starts the explore_node (regular rclcpp::Node from m-explore-ros2).
The node blocks in its constructor until Nav2's navigate_to_pose action
server is ready, then begins frontier planning automatically.

Exploration is paused/resumed by publishing to /explore/resume (Bool):
  False → stop()  – cancels active Nav2 goal, halts planning timer
  True  → resume() – restarts planning timer

controle_robo.py controls exploration via this topic.

Usage:
  ros2 launch bb8_control explore.launch.py
  ros2 launch bb8_control explore.launch.py verbose:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the explore_lite launch description."""
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging",
    )

    verbose = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    pkg_control = FindPackageShare("bb8_control")

    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        namespace="",
        parameters=[
            PathJoinSubstitution([pkg_control, "config", "explore.yaml"]),
            {"use_sim_time": True},
        ],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, explore_node])
