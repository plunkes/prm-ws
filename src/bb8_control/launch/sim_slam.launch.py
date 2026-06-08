from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
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
    # Added explicit sim time argument defaulting to true for this sim launch
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    verbose = LaunchConfiguration("verbose")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_sim = FindPackageShare("prm_2026")
    pkg_control = FindPackageShare("bb8_control")

    # 1. Gazebo simulation
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "inicia_simulacao.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 2. Robot spawn + controllers + RViz
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "carrega_robo.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 3. SLAM Toolbox (Now passing use_sim_time!)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "slam.launch.py"])
        ),
        launch_arguments={
            "verbose": verbose,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [declare_verbose, declare_world, declare_use_sim_time, sim, robot, slam]
    )

