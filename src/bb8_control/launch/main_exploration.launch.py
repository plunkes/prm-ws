"""
Master exploration launch.

Brings up the complete autonomous stack in the correct boot order:

  1. Gazebo simulation  (prm_2026/inicia_simulacao)
  2. Robot spawn + controllers + RViz  (prm_2026/carrega_robo)
  3. SLAM Toolbox  (bb8_control/slam.launch.py)        — starts immediately
  4. Perception  (bb8_control/perception.launch.py)    — t + 5 s
  5. Nav2 navigation stack  (bb8_control/nav2.launch.py) — t + 10 s
     (SLAM needs ~8-10 s to receive the first LIDAR scan and publish a valid
     map; if Nav2 starts earlier its global costmap defaults to a tiny 5 m²
     area centred at the world origin, placing the robot outside it)
  6. explore_lite + FSM controller  (bb8_control/explore_control.launch.py)
                                                       — t + 16 s
     (Nav2 lifecycle activation takes ~3-5 s after Nav2 starts)

Usage:
  ros2 launch bb8_control main_exploration.launch.py
  ros2 launch bb8_control main_exploration.launch.py verbose:=true
  ros2 launch bb8_control main_exploration.launch.py world:=arena_paredes.sdf
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Arguments ─────────────────────────────────────────────────────────────
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging on all nodes",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="arena_cilindros.sdf",
        description="SDF world file name (relative to prm_2026/world/)",
    )

    verbose   = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    pkg_sim     = FindPackageShare("prm_2026")
    pkg_control = FindPackageShare("bb8_control")

    # ── 1. Gazebo simulation ───────────────────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "inicia_simulacao.launch.py"])
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # ── 2. Robot spawn + controllers + RViz ───────────────────────────────────
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "carrega_robo.launch.py"])
        ),
    )

    # ── 3. SLAM (t = 0) ───────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "slam.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # ── 4. Perception (t = 5 s) ───────────────────────────────────────────────
    perception = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_control, "launch", "perception.launch.py"])
            ),
            launch_arguments={"verbose": verbose}.items(),
        )],
    )

    # ── 5. Nav2 (t = 8 s) ─────────────────────────────────────────────────────
    # Nav2 may briefly see malformed 0×0 SLAM maps during the startup race
    # (odom_gt TF vs LIDAR scan ordering).  The FSM controller calls
    # _clear_global_costmap() the moment a valid SLAM map arrives, which forces
    # Nav2 to reinitialise from the real map.  No need to delay Nav2 further.
    nav2 = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_control, "launch", "nav2.launch.py"])
            ),
            launch_arguments={"verbose": verbose}.items(),
        )],
    )

    # ── 6. explore_lite + FSM controller (t = 12 s) ───────────────────────────
    # Nav2 lifecycle nodes take ~3-4 s to become active after Nav2 starts.
    explore = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_control, "launch", "explore.launch.py"])
            ),
            launch_arguments={"verbose": verbose}.items(),
        )],
    )

    controle_node = TimerAction(
        period=12.0,
        actions=[Node(
            package="bb8_control",
            executable="controle_robo",
            name="controle_robo",
            output="screen",
            parameters=[{"use_sim_time": True}],
            ros_arguments=["--log-level", log_level],
        )],
    )

    return LaunchDescription([
        declare_verbose,
        declare_world,
        sim,
        robot,
        slam,
        perception,
        nav2,
        explore,
        controle_node,
    ])
