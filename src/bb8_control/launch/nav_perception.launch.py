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

    verbose = LaunchConfiguration("verbose")
    pkg_control = FindPackageShare("bb8_control")

    # 4. Nav2 navigation stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "nav2.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    # 6. Vision / perception
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, "launch", "perception.launch.py"])
        ),
        launch_arguments={"verbose": verbose}.items(),
    )

    return LaunchDescription([declare_verbose, nav2, perception])
