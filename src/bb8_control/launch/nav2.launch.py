"""
Nav2 navigation-only launch.

Assumes slam_toolbox is already publishing /map and the map→odom TF.
Does NOT launch AMCL or a second SLAM instance.

Topic bridge: Nav2 outputs /cmd_vel; the robot's diff-drive controller
listens on /diff_drive_base_controller/cmd_vel_unstamped. carrega_robo.launch.py
(from prm_2026) already spawns this relay; the one below is a safety fallback
for when this file is launched in isolation.

Usage:
  ros2 launch bb8_control nav2.launch.py
  ros2 launch bb8_control nav2.launch.py verbose:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging",
    )

    verbose   = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    pkg_control      = FindPackageShare("bb8_control")
    pkg_nav2_bringup = FindPackageShare("nav2_bringup")

    nav2_params_file = PathJoinSubstitution(
        [pkg_control, "config", "nav2_params.yaml"]
    )

    # navigation_launch.py: planner, controller, behavior_server,
    # bt_navigator, velocity_smoother, lifecycle_manager.
    # Does NOT start slam_toolbox or AMCL.
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_nav2_bringup, "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file":  nav2_params_file,
            "autostart":    "true",
            "log_level":    PythonExpression(
                ["'debug' if '", verbose, "' == 'true' else 'info'"]
            ),
        }.items(),
    )

    # Relay /cmd_vel → /diff_drive_base_controller/cmd_vel_unstamped.
    # Required when running nav2.launch.py standalone (without carrega_robo).
    # Requires: sudo apt install ros-humble-topic-tools
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/cmd_vel",
            "/diff_drive_base_controller/cmd_vel_unstamped",
        ],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, nav2_navigation, cmd_vel_relay])
