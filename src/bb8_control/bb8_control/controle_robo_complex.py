#!/usr/bin/env python3
"""
Main Control Node for BB8 Autonomous Robot
- Integrates sensor data (LIDAR, camera, odometry, map)
- Runs FSM state machine for exploration and navigation
- Implements movement control with reactive obstacle avoidance
- Fallback exploration heuristic when D* Lite fails
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import cv2
import numpy as np

from bb8_control.maquina_estados import GerenciadorMissao


def clip(value, lower, upper):
    return max(lower, min(value, upper))


class ControleRobo(Node):
    """
    Main robot control node integrating all sensor data and FSM logic.
    """

    def __init__(self):
        super().__init__("controle_robo")

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # QoS for best effort sensors
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile=qos_sensor
        )
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos_sensor)
        self.create_subscription(
            Image,
            "/robot_cam/colored_map",
            self.camera_callback,
            qos_profile=qos_sensor,
        )
        self.create_subscription(
            Odometry,
            "/diff_drive_base_controller/odom",
            self.odom_callback,
            qos_profile=qos_sensor,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        # Vision data subscriber (flag detection from vision node)
        self.create_subscription(
            Pose2D, "/vision/flag_detection", self.vision_callback, 10
        )

        # Gerenciador de Missão e variáveis de ambiente
        self.gerenciador_missao = GerenciadorMissao()
        self.mapa_2d = None
        self.resolucao_mapa = 0.05
        self.origem_mapa_x = 0.0
        self.origem_mapa_y = 0.0
        self.posicao_robo_grid = None

        # Odometria real do robô
        self.x_real = 0.0
        self.y_real = 0.0
        self.yaw_robo = 0.0

        # Variáveis da Bandeira e Visão
        self.bandeira_detectada_vision = False
        self.flag_image_coords = None
        self.distancia_frente = 10.0  # Inicializa com o range máximo do laser

        self.bridge = CvBridge()

        # Timer de controle a 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.move_robot)

        # Estado dos sensores
        self.obstaculo_a_frente = False

        # Filtros e Rampas de Velocidade
        self.linear_atual = 0.0
        self.angular_atual = 0.0

        # Limites Físicos de Velocidade
        self.MAX_LINEAR_VEL = 0.8  # m/s
        self.MAX_ANGULAR_VEL = 1.5  # rad/s

        # Taxa de aceleração permitida por ciclo (Slew Rate)
        self.MAX_LINEAR_ACCEL = 0.04
        self.MAX_ANGULAR_ACCEL = 0.1

        # Velocidades desejadas que serão calculadas
        self.velocidade_linear_desejada = 0.0
        self.velocidade_angular_desejada = 0.0

        # Exploration fallback heuristic
        self.fallback_mode = False
        self.fallback_direction = None

        self.get_logger().info("ControleRobo initialized")

    def map_callback(self, msg: OccupancyGrid):
        largura = msg.info.width
        altura = msg.info.height
        self.resolucao_mapa = msg.info.resolution
        self.origem_mapa_x = msg.info.origin.position.x
        self.origem_mapa_y = msg.info.origin.position.y
        self.mapa_2d = np.array(msg.data).reshape((altura, largura))

    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        self.obstaculo_a_frente = False
        distancias_frente = []

        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # TRATAMENTO DO ESPAÇO ABERTO (INF/NAN):
            # Se o feixe retornou infinito ou nan, assumimos que o espaço está livre até o limite máximo do Xacro
            distancia_atual = msg.ranges[i]
            if np.isinf(distancia_atual) or np.isnan(distancia_atual):
                distancia_atual = msg.range_max

            # Verifica apenas a janela frontal do robô (± 0.5 rad)
            if -0.5 <= angle <= 0.5:
                # Se houver algo real obstruindo a menos de 0.6m (e ignora o próprio chassi < 0.12)
                if 0.12 < distancia_atual < 0.6:
                    self.obstaculo_a_frente = True
                    distancias_frente.append(distancia_atual)

        # Salva a menor distância lida à frente para cálculos de aproximação
        if distancias_frente:
            self.distancia_frente = min(distancias_frente)
        else:
            self.distancia_frente = msg.range_max

    def imu_callback(self, msg: Imu):
        pass

    def vision_callback(self, msg: Pose2D):
        """
        Callback for vision-based flag detection.
        Receives (x, y) image coordinates of detected flag.
        """
        if msg.x >= 0 and msg.y >= 0:  # Valid detection
            self.bandeira_detectada_vision = True
            self.flag_image_coords = (int(msg.x), int(msg.y))
            self.get_logger().debug(
                f"Vision: Flag detected at image coords {self.flag_image_coords}",
                throttle_duration_sec=2.0,
            )
        else:
            self.bandeira_detectada_vision = False
            self.flag_image_coords = None

    def odom_callback(self, msg: Odometry):
        self.x_real = msg.pose.pose.position.x
        self.y_real = msg.pose.pose.position.y

        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        self.yaw_robo = R.from_quat(quat).as_euler("xyz")[2]

        if self.mapa_2d is not None:
            # Converte Metros -> Grid
            coluna_x = int((self.x_real - self.origem_mapa_x) / self.resolucao_mapa)
            linha_y = int((self.y_real - self.origem_mapa_y) / self.resolucao_mapa)

            altura, largura = self.mapa_2d.shape
            self.posicao_robo_grid = (
                max(0, min(coluna_x, largura - 1)),
                max(0, min(linha_y, altura - 1)),
            )

    def compute_exploration_fallback(self):
        """
        Fallback exploration heuristic when D* Lite fails.
        Analyzes local map to find direction towards unknown cells.

        Returns:
            (linear_vel, angular_vel) tuple for reactive movement
        """
        if self.mapa_2d is None or self.posicao_robo_grid is None:
            return (0.0, 0.0)

        x, y = self.posicao_robo_grid
        altura, largura = self.mapa_2d.shape
        search_radius = 5  # cells to search

        # Find unknown cells in surrounding area
        x_min = max(0, x - search_radius)
        x_max = min(largura, x + search_radius)
        y_min = max(0, y - search_radius)
        y_max = min(altura, y + search_radius)

        # Extract local region
        local_map = self.mapa_2d[y_min:y_max, x_min:x_max]

        # Find unknown cells (value -1)
        unknown_mask = local_map == -1
        unknown_count = np.sum(unknown_mask)

        if unknown_count == 0:
            # No unknowns nearby, explore forward
            self.get_logger().info(
                "Fallback: No unknowns in local area, moving forward",
                throttle_duration_sec=2.0,
            )
            return (0.3, 0.0)  # Move forward

        # Find centroid of unknown cells
        y_unknown, x_unknown = np.where(unknown_mask)
        centroid_x = np.mean(x_unknown) + x_min
        centroid_y = np.mean(y_unknown) + y_min

        # Calculate direction to unknown centroid
        dx = centroid_x - x
        dy = centroid_y - y

        target_angle = np.arctan2(dy, dx)
        error_angle = target_angle - self.yaw_robo
        error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi

        self.get_logger().info(
            f"Fallback: Found unknowns at angle {np.degrees(target_angle):.1f}°, "
            f"error={np.degrees(error_angle):.1f}°",
            throttle_duration_sec=2.0,
        )

        # Simple proportional control
        angular_vel = clip(
            error_angle * 1.5, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL
        )

        # Move forward if aligned enough
        linear_vel = 0.2 if abs(error_angle) < 0.3 else 0.0

        return (linear_vel, angular_vel)

    def camera_callback(self, msg: Image):
        # DEBUG ZERO: O callback está rodando? Os tópicos essenciais chegaram?
        if self.mapa_2d is None:
            self.get_logger().info("DEBUG: Aguardando mapa_2d do /map...")
            return
        if self.posicao_robo_grid is None:
            self.get_logger().info(
                "DEBUG: Aguardando posicao_robo_grid da odometria..."
            )
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        target_color = np.array([171, 242, 0])
        mask = cv2.inRange(frame, target_color, target_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_vista = len(contours) > 0
        cx_bandeira = None

        if self.bandeira_vista:
            maior_contorno = max(contours, key=cv2.contourArea)
            M = cv2.moments(maior_contorno)
            if M["m00"] != 0:
                cx_bandeira = int(M["m10"] / M["m00"])

        coordenada_bandeira_grid = None
        if self.bandeira_vista:
            dist = self.distancia_frente
            x_band_real = self.x_real + (dist * np.cos(self.yaw_robo))
            y_band_real = self.y_real + (dist * np.sin(self.yaw_robo))

            c_band = int((x_band_real - self.origem_mapa_x) / self.resolucao_mapa)
            l_band = int((y_band_real - self.origem_mapa_y) / self.resolucao_mapa)

            altura, largura = self.mapa_2d.shape
            coordenada_bandeira_grid = (
                max(0, min(c_band, largura - 1)),
                max(0, min(l_band, altura - 1)),
            )

        # Pede para a Máquina de Estados qual é o próximo passo
        proximo_passo_grid = self.gerenciador_missao.atualizar_estado_e_caminho(
            self.mapa_2d,
            self.posicao_robo_grid,
            self.bandeira_detectada_vision,  # Use vision-based detection
            coordenada_bandeira_grid,
            self.distancia_frente,
            self.yaw_robo,
        )

        estado_atual = self.gerenciador_missao.ESTADO_ATUAL
        alvo_global = self.gerenciador_missao.alvo_atual

        # DEBUG 1: Estado atual da missão e alvos gerados
        self.get_logger().info(
            f"ESTADO: {estado_atual} | Pos Grid: {self.posicao_robo_grid} | "
            f"Alvo: {alvo_global} | Próximo Passo: {proximo_passo_grid}",
            throttle_duration_sec=1.5,
        )

        # --- MÓDULO DE DECISÃO DE MOVIMENTO COM LOGS ---
        if estado_atual == "PROCURANDO_BANDEIRA":
            # 360° rotation to find the flag
            self.velocidade_linear_desejada = 0.0
            self.velocidade_angular_desejada = 0.4

        elif estado_atual == "BANDEIRA_DETECTADA":
            # Brief transition state
            self.velocidade_linear_desejada = 0.0
            self.velocidade_angular_desejada = 0.0

        elif estado_atual == "POSICIONANDO_PARA_COLETA":
            # Fine-tuning alignment and approach to flag
            if self.obstaculo_a_frente and self.distancia_frente < 0.25:
                self.get_logger().warn(
                    "Obstacle blocking collection!", throttle_duration_sec=2.0
                )
                self.velocidade_linear_desejada = 0.0
                self.velocidade_angular_desejada = 0.0
            else:
                largura_imagem = frame.shape[1]
                if cx_bandeira is not None:
                    # Center the flag in the image
                    erro_x = (largura_imagem / 2) - cx_bandeira
                    if abs(erro_x) > 15:
                        # Rotate to center
                        self.velocidade_linear_desejada = 0.0
                        self.velocidade_angular_desejada = 0.003 * erro_x
                    else:
                        # Move towards centered flag
                        self.velocidade_linear_desejada = 0.1
                        self.velocidade_angular_desejada = 0.0
                else:
                    # Can't see flag, stop
                    self.velocidade_linear_desejada = 0.0
                    self.velocidade_angular_desejada = 0.0

        else:  # EXPLORANDO and NAVIGANDO_PARA_BANDEIRA
            # Emergency obstacle avoidance
            if self.obstaculo_a_frente and self.distancia_frente < 0.4:
                self.get_logger().warn(
                    "Emergency avoidance active!", throttle_duration_sec=1.0
                )
                self.velocidade_linear_desejada = 0.0
                self.velocidade_angular_desejada = 0.6

            # Try to use D* Lite path
            elif proximo_passo_grid:
                x_alvo_real = (
                    proximo_passo_grid[0] * self.resolucao_mapa
                ) + self.origem_mapa_x
                y_alvo_real = (
                    proximo_passo_grid[1] * self.resolucao_mapa
                ) + self.origem_mapa_y

                dx = x_alvo_real - self.x_real
                dy = y_alvo_real - self.y_real
                distancia_alvo = np.hypot(dx, dy)

                angulo_alvo = np.arctan2(dy, dx)
                erro_angular = angulo_alvo - self.yaw_robo
                erro_angular = (erro_angular + np.pi) % (2 * np.pi) - np.pi

                self.get_logger().debug(
                    f"D* Path: dx={dx:.2f}, dy={dy:.2f}, dist={distancia_alvo:.3f} | Ang Error={np.degrees(erro_angular):.1f}°",
                    throttle_duration_sec=1.5,
                )

                # Se alvo está muito perto (< 1 célula = 0.05m), pular para o próximo
                if distancia_alvo < 0.05:
                    self.get_logger().debug(
                        f"Next waypoint reached (dist={distancia_alvo:.4f}m), moving to next",
                        throttle_duration_sec=1.0,
                    )
                    self.velocidade_linear_desejada = 0.0
                    self.velocidade_angular_desejada = 0.0
                else:
                    KP_ANGULAR = 2.5
                    self.velocidade_angular_desejada = clip(
                        KP_ANGULAR * erro_angular,
                        -self.MAX_ANGULAR_VEL,
                        self.MAX_ANGULAR_VEL,
                    )

                    if abs(erro_angular) < 0.4:
                        KP_LINEAR = 0.6
                        self.velocidade_linear_desejada = clip(
                            KP_LINEAR * distancia_alvo, 0.0, 0.4
                        )
                    else:
                        self.velocidade_linear_desejada = 0.0

            else:
                # D* Lite failed, use fallback exploration heuristic
                self.get_logger().warn(
                    f"D* failed. Alvo: {alvo_global}. Using fallback heuristic.",
                    throttle_duration_sec=2.0,
                )
                linear_vel, angular_vel = self.compute_exploration_fallback()
                self.velocidade_linear_desejada = linear_vel
                self.velocidade_angular_desejada = angular_vel

    def move_robot(self):
        """
        Filtro Cinético: Aplica rampa de aceleração suave (Slew Rate) e publica no /cmd_vel
        """
        twist = Twist()

        # Filtro de rampa para velocidade linear
        erro_linear = self.velocidade_linear_desejada - self.linear_atual
        if abs(erro_linear) > self.MAX_LINEAR_ACCEL:
            self.linear_atual += np.sign(erro_linear) * self.MAX_LINEAR_ACCEL
        else:
            self.linear_atual = self.velocidade_linear_desejada

        # Filtro de rampa para velocidade angular
        erro_angular = self.velocidade_angular_desejada - self.angular_atual
        if abs(erro_angular) > self.MAX_ANGULAR_ACCEL:
            self.angular_atual += np.sign(erro_angular) * self.MAX_ANGULAR_ACCEL
        else:
            self.angular_atual = self.velocidade_angular_desejada

        # Saturação de segurança final
        self.linear_atual = clip(
            self.linear_atual, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL
        )
        self.angular_atual = clip(
            self.angular_atual, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL
        )

        # Publicação dos comandos
        twist.linear.x = self.linear_atual
        twist.angular.z = self.angular_atual
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
