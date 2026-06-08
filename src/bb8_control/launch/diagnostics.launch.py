"""
Diagnostics launch.

Starts the bb8_diagnostics node which monitors topic rates, collision
proximity, stuck detection, and SLAM health while the full stack runs.

Run this in a separate terminal alongside main_exploration.launch.py:
  ros2 launch bb8_control diagnostics.launch.py

All diagnostic findings are printed at INFO level every 10 seconds.
WARN/ERROR events are also printed immediately when they occur.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the diagnostics launch description."""
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging",
    )

    verbose = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    diag_node = Node(
        package="bb8_control",
        executable="diagnostics",
        name="bb8_diagnostics",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, diag_node])
