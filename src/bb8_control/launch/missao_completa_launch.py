"""
Missão completa: Gazebo + robô + visão + controle FSM Wall Follower.

Ordem de inicialização:
  1. inicia_simulacao  – Gazebo com arena_cilindros.sdf
  2. carrega_robo      – robot_state_publisher, spawn, controladores, bridges, RViz
  3. ground_truth_odometry – converte /model/prm_robot/pose → /odom_gt
  4. vision_processor  – segmentação semântica → detecção da bandeira
  5. controle_robo     – FSM wall follower
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = FindPackageShare("prm_2026")

    # 1. Gazebo com o mundo da arena
    simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "inicia_simulacao.launch.py"])
        ),
        launch_arguments={"world": "arena_cilindros.sdf"}.items(),
    )

    # 2. Robot + controladores + bridge LIDAR/câmera/ground-truth
    robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "carrega_robo.launch.py"])
        ),
    )

    # 3. Odometria ground truth (/model/prm_robot/pose → /odom_gt)
    gt_odom = Node(
        package="prm_2026",
        executable="ground_truth_odometry",
        name="ground_truth_odometry",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 4. Processamento visual: /robot_cam/labels_map → /vision/flag_detection + /vision/flag_bearing
    visao = Node(
        package="bb8_control",
        executable="vision_processor",
        name="vision_processor",
        parameters=[
            {"use_sim_time": True},
            {"flag_label_ids": [25]},
        ],
        output="screen",
    )

    # 5. FSM de controle – Wall Follower
    controle = Node(
        package="bb8_control",
        executable="controle_robo",
        name="controle_robo",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            simulacao,
            robo,
            gt_odom,
            visao,
            controle,
        ]
    )
