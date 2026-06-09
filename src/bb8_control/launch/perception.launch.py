"""
Perception launch.

Starts the vision_processor node which decodes the Gazebo semantic
segmentation camera and publishes flag detection data:
  /vision/flag_detection  (geometry_msgs/Pose2D)  – centroid + visibility flag
  /vision/flag_bearing    (std_msgs/Float32)       – bearing to flag centre (rad)
  /vision/scene_class     (std_msgs/String)        – "objective" | "obstacle" | "clear"

Prerequisites: Gazebo simulation must be running so the camera bridge topic
  /robot_cam/labels_map  is available.

Usage:
  ros2 launch bb8_control perception.launch.py
  ros2 launch bb8_control perception.launch.py verbose:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Set to true to enable DEBUG-level logging",
    )

    verbose   = LaunchConfiguration("verbose")
    log_level = PythonExpression(["'DEBUG' if '", verbose, "' == 'true' else 'INFO'"])

    vision_node = Node(
        package="bb8_control",
        executable="vision_processor",
        name="vision_processor",
        output="screen",
        parameters=[{
            "use_sim_time":     True,
            "flag_label_id":    25,   # label 25 = blue_flag (target); label 20 = red_flag (own team, ignore)
            "camera_hfov_deg":  90.0,
        }],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([declare_verbose, vision_node])
