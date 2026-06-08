import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Vision processor node for semantic segmentation
    vision_node = Node(
        package="bb8_control",
        executable="vision_processor",
        name="vision_processor",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", "info"],
    )

    # Main control node
    controle_node = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        output="screen",
        parameters=[{"use_sim_time": True}],
        ros_arguments=["--log-level", "info"],
    )

    return LaunchDescription([vision_node, controle_node])
