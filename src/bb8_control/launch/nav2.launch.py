"""
Nav2 navigation-only launch.

Assumes slam_toolbox is already publishing /map and the map→odom TF.
Does NOT launch AMCL or a second SLAM instance.

Topic bridge: Nav2 outputs /cmd_vel; this launch relays it to the
robot's actual command topic /diff_drive_base_controller/cmd_vel_unstamped.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_control = FindPackageShare("bb8_control")
    pkg_nav2_bringup = FindPackageShare("nav2_bringup")

    nav2_params_file = PathJoinSubstitution(
        [pkg_control, "config", "nav2_params.yaml"]
    )

    # navigation_launch.py starts only the navigation stack (planner, controller,
    # behavior_server, bt_navigator, velocity_smoother, lifecycle_manager).
    # It does NOT start slam_toolbox or AMCL — the map→odom TF from slam_toolbox
    # is sufficient for global planning.
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_nav2_bringup, "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
    )

    # Nav2 controller_server publishes to /cmd_vel.
    # The robot's diff_drive controller listens on /diff_drive_base_controller/cmd_vel_unstamped.
    # This relay bridges the mismatch without modifying Nav2 internals.
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
    )

    return LaunchDescription([
        nav2_navigation,
        cmd_vel_relay,
    ])
