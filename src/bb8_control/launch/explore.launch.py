"""
explore_lite launch.

Starts the explore_node lifecycle node and a dedicated lifecycle manager that
auto-activates it. Once active, explore_node continuously sends NavigateToPose
goals to Nav2 targeting the nearest unexplored frontier.

controle_robo.py intercepts exploration by calling:
  /explore_node/change_state  (TRANSITION_DEACTIVATE = 4)

and resumes it by calling:
  /explore_node/change_state  (TRANSITION_ACTIVATE = 3)

Prerequisites: Nav2 must be running so the global costmap is available.

Install:  sudo apt install ros-humble-explore-lite

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
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging",
    )

    verbose = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    pkg_control = FindPackageShare("bb8_control")

    explore_node = LifecycleNode(
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

    # Manages explore_node lifecycle: configure → activate on startup.
    # The controle_robo FSM calls change_state directly to deactivate/reactivate
    # without going through the manager, which is fine – the manager observes
    # but does not interfere with direct state transitions.
    lifecycle_manager_explore = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_explore",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["explore_node"],
                # Bond timeout: if explore_node crashes, restart it.
                "bond_timeout": 4.0,
                "attempt_respawn_reconnection": True,
            }
        ],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, explore_node, lifecycle_manager_explore])
