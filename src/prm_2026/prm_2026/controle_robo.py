#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import cv2
import numpy as np

def clip(value, lower, upper):
    return max(lower, min(value, upper))

class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        self.bridge = CvBridge()
        
        # Timer aumentado para 20Hz (0.05s) para controle mais suave e rápido
        self.timer = self.create_timer(0.05, self.move_robot)

        # --- VARIÁVEIS DE ESTADO DO SENSOR ---
        self.obstaculo_a_frente = False
        
        # --- FILTROS DE VELOCIDADE (SUA PARTE) ---
        self.linear_atual = 0.0
        self.angular_atual = 0.0

        # Limites Físicos de Velocidade (Deixando o robô rápido)
        self.MAX_LINEAR_VEL = 0.8   # m/s (Antes era 0.1)
        self.MAX_ANGULAR_VEL = 1.5  # rad/s (Antes era 0.3)

        # Taxa de aceleração permitida por ciclo (Slew Rate)
        self.MAX_LINEAR_ACCEL = 0.04  # m/s² por ciclo
        self.MAX_ANGULAR_ACCEL = 0.1  # rad/s² por ciclo

        # --- INTERFACE PARA A HEURÍSTICA (O GRUPO MUDA AQUI) ---
        self.velocidade_linear_desejada = 0.0
        self.velocidade_angular_desejada = 0.0

    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Verifica obstáculo apenas na janela frontal (± 0.5 rad)
        self.obstaculo_a_frente = False
        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -0.5 <= angle <= 0.5: 
                if 0.05 < msg.ranges[i] < 0.6: # Ignora o próprio chassi (<0.05) e detecta até 0.6m
                    self.obstaculo_a_frente = True
                    break

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        pass

    def camera_callback(self, msg: Image):
        # Exemplo de código de visão (pode ser modificado pelos seus colegas)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        target_color = np.array([171, 242, 0])
        mask = cv2.inRange(frame, target_color, target_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Lógica Básica de Comportamento para teste de movimentação
        if self.obstaculo_a_frente:
            # Desvio de emergência: Para de ir pra frente e gira rápido
            self.velocidade_linear_desejada = 0.0
            self.velocidade_angular_desejada = 0.8
        elif len(contours) > 0:
            # Achou a bandeira: Vai em direção a ela
            self.velocidade_linear_desejada = 0.4
            self.velocidade_angular_desejada = 0.0 # Aqui o grupo pode colocar um controle Proporcional para centralizar
        else:
            # Explorando (andando e girando levemente)
            self.velocidade_linear_desejada = 0.3
            self.velocidade_angular_desejada = -0.3

    def move_robot(self):
        """
        Módulo Cinético: Filtra as velocidades desejadas e envia comandos suaves aos motores.
        """
        twist = Twist()

        # 1. Filtro Rampa (Aceleração Suave) Linear
        erro_linear = self.velocidade_linear_desejada - self.linear_atual
        if abs(erro_linear) > self.MAX_LINEAR_ACCEL:
            self.linear_atual += np.sign(erro_linear) * self.MAX_LINEAR_ACCEL
        else:
            self.linear_atual = self.velocidade_linear_desejada

        # 2. Filtro Rampa (Aceleração Suave) Angular
        erro_angular = self.velocidade_angular_desejada - self.angular_atual
        if abs(erro_angular) > self.MAX_ANGULAR_ACCEL:
            self.angular_atual += np.sign(erro_angular) * self.MAX_ANGULAR_ACCEL
        else:
            self.angular_atual = self.velocidade_angular_desejada

        # 3. Saturação Segura (Corte nos Limites)
        self.linear_atual = clip(self.linear_atual, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
        self.angular_atual = clip(self.angular_atual, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

        # 4. Publicação
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

if __name__ == '__main__':
    main()
