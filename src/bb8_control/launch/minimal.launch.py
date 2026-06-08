import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Packages
    pkg_simulacao = FindPackageShare("prm_2026")
    pkg_slam = FindPackageShare("bb8_slam")
    pkg_control = FindPackageShare("bb8_control")

    # Start Gazebo simulation with empty world
    inclui_simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_simulacao.find("prm_2026"), "launch", "inicia_simulacao.launch.py"
            )
        ),
        launch_arguments={
            "world": "empty.sdf"
        }.items(),
    )

    # Load robot into simulation
    inclui_carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_simulacao.find("prm_2026"), "launch", "carrega_robo.launch.py"
            )
        )
    )

    # Start SLAM (includes scan filter)
    inclui_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam.find("bb8_slam"), "launch", "slam_launch.py")
        )
    )

    # Start minimal control node
    controle_node = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        output="screen",
    )

    return LaunchDescription([
        inclui_simulacao,
        inclui_carrega_robo,
        inclui_slam,
        controle_node,
    ])
