# bb8_control

Pacote de controle autônomo do robô para a disciplina SSC0712. O robô explora o ambiente seguindo paredes, detecta uma bandeira pela câmera semântica e navega até ela.

## Requisitos

```
ROS 2 Humble
ros-humble-ros-gz-bridge
ros-humble-ros-gz-sim
ros-humble-ign-ros2-control
ros-humble-diff-drive-controller
ros-humble-joint-state-broadcaster
ros-humble-position-controllers
ros-humble-robot-state-publisher
ros-humble-topic-tools
python3-opencv
python3-numpy
ros-humble-cv-bridge
```

Instalar dependências:

```bash
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ign-ros2-control \
  ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster \
  ros-humble-position-controllers ros-humble-topic-tools ros-humble-cv-bridge \
  python3-opencv python3-numpy
```

## Compilar

```bash
cd ~/progs/prm_ws
colcon build --packages-select bb8_control
source install/setup.bash
```

## Rodar

```bash
ros2 launch bb8_control missao_completa_launch.py
```

## Arquitetura e fluxo de controle

O sistema tem dois nodos: `vision_processor` e `controle_robo`.

**vision_processor** lê `/robot_cam/labels_map` (câmera semântica do Gazebo) a cada frame. Para cada pixel, verifica se o label corresponde à bandeira (label 25 = azul, já que é do time vermelho). Se a área de pixels detectados for maior que 40, calcula o centroide horizontal e converte para bearing em radianos. Publica em `/vision/flag_detection` (Pose2D com flag de detecção) e `/vision/flag_bearing` (Float32).

**controle_robo** roda a 20 Hz e implementa a FSM:

```
SEGUINDO_PAREDE
  ├─ sem parede no LIDAR (> 1.8m)  → avança reto a 0.35 m/s
  ├─ parede à direita detectada     → controle P: ω = -2.2 × (dist_direita - 0.5m)
  ├─ obstáculo frontal < 0.4m      → para e gira esquerda (override de segurança)
  ├─ célula visitada ≥ 6 vezes     → EVITANDO_LOOP
  └─ bandeira detectada (≥ 40px)   → DETECTOU_BANDEIRA

DETECTOU_BANDEIRA  (1 tick, apenas para logar a transição)
  └─ sempre → NAVEGANDO_PARA_BANDEIRA  (braço se estende)

NAVEGANDO_PARA_BANDEIRA
  ├─ bandeira perdida por > 10 ticks → SEGUINDO_PAREDE  (braço recolhe)
  ├─ alinhado (bearing < 0.06 rad) E frente < 1.0m → POSICIONANDO_FINAL
  ├─ não alinhado → gira no lugar: ω = 1.8 × bearing
  └─ alinhado     → avança a 0.28 m/s (reduz 40% se frente < 1.5m)

POSICIONANDO_FINAL
  ├─ frente > 0.75m → avança devagar (0.12 m/s) com correção de bearing
  └─ frente ≤ 0.75m → para, imprime "Congratulations"

EVITANDO_LOOP
  ├─ fase 0: gira esquerda 2.5s
  └─ fase 1: avança reto 3.5s → SEGUINDO_PAREDE
```

O braço é controlado via `/gripper_controller/commands`. Retraído: `[-1.5, -1.5, 0, 0]`. Estendido: `[0, 0, 0, 0]`. A ordem dos valores é `[gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint]`.

A posição do robô vem de `/odom_gt`, produzido pelo nodo `ground_truth_odometry` do pacote `prm_2026` a partir do ground truth do Gazebo.

## Tópicos relevantes

| Tópico | Tipo | Direção |
|---|---|---|
| `/scan` | LaserScan | entrada |
| `/odom_gt` | Odometry | entrada |
| `/robot_cam/labels_map` | Image | entrada |
| `/vision/flag_detection` | Pose2D | saída vision |
| `/vision/flag_bearing` | Float32 | saída vision |
| `/cmd_vel` | Twist | saída controle |
| `/gripper_controller/commands` | Float64MultiArray | saída controle |
