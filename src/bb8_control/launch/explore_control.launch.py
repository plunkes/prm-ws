from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging on all nodes",
    )

    verbose = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    pkg_control = FindPackageShare("bb8_control")

    # 5. explore_lite (starts exploring once Nav2 costmap is ready)
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "explore.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # 7. Master FSM controller
    controle_node = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, explore, controle_node])
