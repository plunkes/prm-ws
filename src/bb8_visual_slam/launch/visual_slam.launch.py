"""
Visual SLAM launch — replaces simulation_slam.launch.py.

Uses RTAB-Map with:
  - Laser scan (slam_toolbox replaced by rtabmap for 2D occupancy map)
  - RGB camera for visual loop closure detection

To install rtabmap:
  sudo apt install ros-humble-rtabmap-ros

Run this INSTEAD of simulation_slam.launch.py:
  ros2 launch bb8_visual_slam visual_slam.launch.py
  ros2 launch bb8_control nav2.launch.py
  ros2 launch bb8_control control.launch.py
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

    # Bridge the RGB camera topic added to robot.urdf.xacro for visual SLAM.
    # The segmentation camera bridge lives in carrega_robo.launch.py (unchanged).
    rgb_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="rgb_camera_bridge",
        arguments=[
            "/robot_cam_rgb@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam_rgb/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
        output="screen",
    )

    # RTAB-Map node
    #
    # Strategy: use laser scan as the primary odometry + mapping sensor (Reg/Strategy=1,
    # ICP registration). The RGB camera is used ONLY for visual loop closure — this means
    # the occupancy map quality is driven by the laser, and the camera reduces drift over
    # long runs by recognising revisited places.
    #
    # With a monocular camera (no depth), we disable visual odometry; loop closure still
    # works through bag-of-words image matching.
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "subscribe_scan": True,
            "subscribe_rgb": True,
            "subscribe_depth": False,       # monocular – no depth sensor
            "subscribe_stereo": False,
            "approx_sync": True,
            "approx_sync_max_interval": 0.1,
            # 2-D mode: ICP registration from laser scan
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "true",
            # Occupancy grid from laser scan (not stereo/depth)
            "Grid/Sensor": "0",
            "Grid/RangeMax": "3.5",
            "Grid/CellSize": "0.05",
            # Visual loop closure settings
            "Kp/MaxFeatures": "300",
            "Vis/MinInliers": "15",
            "RGBD/ProximityBySpace": "true",
            "RGBD/LinearUpdate": "0.1",
            "RGBD/AngularUpdate": "0.05",
            # Map update behaviour
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            # Database – fresh map every launch
            "database_path": "",
        }],
        remappings=[
            ("scan", "/scan"),
            ("rgb/image", "/robot_cam_rgb"),
            ("rgb/camera_info", "/robot_cam_rgb/camera_info"),
            ("odom", "/odom"),
        ],
    )

    # rtabmap_viz is an optional visual debugger (shows loop closures, features, etc.)
    # Uncomment to enable:
    # rtabmap_viz = Node(
    #     package="rtabmap_viz",
    #     executable="rtabmap_viz",
    #     name="rtabmap_viz",
    #     output="screen",
    #     parameters=[{"use_sim_time": True, "frame_id": "base_link"}],
    #     remappings=[
    #         ("scan", "/scan"),
    #         ("rgb/image", "/robot_cam_rgb"),
    #         ("rgb/camera_info", "/robot_cam_rgb/camera_info"),
    #     ],
    # )

    return LaunchDescription([
        inclui_simulacao,
        inclui_carrega_robo,
        rgb_camera_bridge,
        rtabmap_node,
        # rtabmap_viz,
    ])
