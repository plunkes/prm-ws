"""
Simulation + SLAM launch.

Starts Gazebo, loads the robot, and runs async_slam_toolbox (online mapping).
Nav2 navigation nodes are kept separate in nav2.launch.py so they can be
restarted independently without reloading the simulator.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_simulacao = FindPackageShare("prm_2026")
    pkg_slam_toolbox = FindPackageShare("slam_toolbox")
    pkg_bb8_control = FindPackageShare("bb8_control")

    inclui_simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_simulacao, "launch", "inicia_simulacao.launch.py"]
            )
        )
    )

    inclui_carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_simulacao, "launch", "carrega_robo.launch.py"]
            )
        )
    )

    # Async SLAM Toolbox – publishes /map and the map→odom TF that Nav2 needs.
    # Load slam_toolbox defaults first, then override with our tuned params.
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(
                pkg_slam_toolbox.find("slam_toolbox"),
                "config",
                "mapper_params_online_async.yaml",
            ),
            PathJoinSubstitution([pkg_bb8_control, "config", "slam_params.yaml"]),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        inclui_simulacao,
        inclui_carrega_robo,
        slam_toolbox_node,
    ])
