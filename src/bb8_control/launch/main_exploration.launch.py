"""
Master exploration launch.

Brings up the complete autonomous stack in the correct boot order:

  1. Gazebo simulation  (prm_2026/inicia_simulacao)
  2. Robot spawn + controllers + RViz  (prm_2026/carrega_robo)
  3. SLAM Toolbox  (bb8_control/slam.launch.py)
  4. Nav2 navigation stack  (bb8_control/nav2.launch.py)
  5. explore_lite + lifecycle manager  (bb8_control/explore.launch.py)
  6. Vision / perception  (bb8_control/perception.launch.py)
  7. Master FSM controller  (bb8_control/controle_robo)

Usage:
  ros2 launch bb8_control main_exploration.launch.py
  ros2 launch bb8_control main_exploration.launch.py verbose:=true
  ros2 launch bb8_control main_exploration.launch.py world:=arena_paredes.sdf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # ── 3. SLAM ────────────────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "slam.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # ── 4. Nav2 ────────────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "nav2.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # ── 5. explore_lite (starts exploring once Nav2 costmap is ready) ─────────
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "explore.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # ── 6. Perception ──────────────────────────────────────────────────────────
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "perception.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # ── 7. Master FSM controller ───────────────────────────────────────────────
    controle_node = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([
        declare_verbose,
        declare_world,
        sim,
        robot,
        slam,
        nav2,
        explore,
        perception,
        controle_node,
    ])
