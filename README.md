# Workspace ROS2 do EduBot 2025

Este repositório contém um *stack ROS 2 completo* para um robô diferencial (2 rodas) com:

- **Raspberry Pi 3** (Ubuntu 22.04 headless) rodando ROS 2 Humble
- **Bluepill (STM32) + L298** controlando os motores e reportando **encoders absolutos**
- **MPU9250** via I2C publicando **IMU** (`/imu`)
- **ros2_control + diff_drive_controller** para controlar via `cmd_vel`
- **robot_localization (EKF)** para fundir odometria de rodas + IMU e publicar `/odometry/filtered` + TF `odom -> base_link`

---

## Estrutura do workspace

Dentro de `ros2_ws/src`:

- `bluepill_ros2_control/`  
  Plugin de hardware (`SystemInterface`) para conversar com o Bluepill via serial.

- `mybot_bringup/`  
  URDF/Xacro + controllers + EKF + launches.

- `ros2_mpu9250_driver/`  
  Driver da MPU9250 (terceiros). Mantém o README original do autor.

---

## Pré-requisitos

- Ubuntu 22.04
- ROS 2 Humble instalado em `/opt/ros/humble`
- Acesso ao serial do Bluepill (ex.: `/dev/serial0`)
- MPU9250 no barramento I2C (`/dev/i2c-1`), normalmente endereço `0x68`

---

## Instalação de dependências

```bash
sudo apt update
sudo apt install -y \
  build-essential git \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-robot-localization \
  i2c-tools libi2c-dev
```

Inicialize o `rosdep` (uma vez):

```bash
sudo rosdep init || true
rosdep update
```

---

## Build do workspace (recomendado para RPi3)

A RPi3 tem pouca RAM. Para evitar travamentos, compile *sequencial*:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -r -y --from-paths src --ignore-src
colcon build --symlink-install --executor sequential --parallel-workers 1
source install/setup.bash
```

### Dica (muito recomendada): swap para não travar compilando
Se a RPi não tiver swap, criar um swapfile de 2GB ajuda muito:

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

Verifique:

```bash
free -h
swapon --show
```

---

## Rodando (manual)

### Terminal 1 — Bringup (ros2_control + controllers + EKF)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mybot_bringup bringup.launch.py
```

### Terminal 2 — IMU (MPU9250)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mpu9250driver mpu9250driver_launch.py
```

---

## Comando de movimento

Este projeto está configurado com:

- `use_stamped_vel: false`

Então o `diff_drive_controller` assina:

- `/diff_drive_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`)

### Andar reto
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
"{linear: {x: 0.12}, angular: {z: 0.0}}"
```

### Girar no lugar
```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.8}}"
```

### Parar
```bash
ros2 topic pub -1 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

> Se você configurar `use_stamped_vel: true`, o controller passa a assinar **`/diff_drive_controller/cmd_vel`** (`geometry_msgs/msg/TwistStamped`).

---

## Tópicos principais

```bash
ros2 topic list | grep -E "imu|odom|filtered|cmd_vel"
```

Esperado:
- `/imu`
- `/diff_drive_controller/odom`
- `/odometry/filtered`
- `/diff_drive_controller/cmd_vel_unstamped`

---

## TF (odom -> base_link)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros tf2_echo odom base_link
```

É comum o `tf2_echo` reclamar “Invalid frame ID” *no começo*; depois que o TF chega, ele começa a imprimir o transform.

---

## Parâmetros do robô (atuais)

- `wheel_radius`: **0.0325 m**
- `wheel_separation`: **0.158 m**
- `ticks_per_rev`: **estimado 2112** (recomenda-se confirmar com 1 volta da roda)
- Encoders: **absolutos** (contagem acumulada; aumenta pra frente e diminui pra trás)

---

## Problemas comuns e como contornar (histórico real)

### 1) “O Pi trava compilando”
- Sintoma: SSH/VSCode cai, LED verde fica “preso”.
- Causa comum: pouca RAM + sem swap + I/O pesado do SD.
- Solução: build sequencial + criar swap (veja seção acima).

### 2) `plugin description file ... does not exist`
- Causa: `bluepill_ros2_control.xml` fora da pasta do pacote.
- Solução: manter a estrutura correta (ver README do pacote `bluepill_ros2_control`).

### 3) `RCLCPP_ERROR` / `HW_IF_POSITION` não encontrados
- Causa: includes faltando no plugin.
- Solução: adicionar `rclcpp/logging.hpp` e `hardware_interface_type_values.hpp`.

### 4) “Publiquei /cmd_vel e não andou”
- Causa: tópico errado (stamped/unstamped).
- Solução: usar `/diff_drive_controller/cmd_vel_unstamped` com `use_stamped_vel: false`.

### 5) “Uma roda girava ao contrário”
- Causa: lado invertido (fiação/ponte H/config).
- Solução (sem mexer no Bluepill): `right_wheel_radius_multiplier: -1.0` (ou o lado que estiver invertido).

---

## Qualidade de vida

### Carregar setup automaticamente ao abrir terminal
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

### Compilar sem “perder” o processo se o SSH cair (tmux)
```bash
sudo apt install -y tmux
tmux new -s build
# ... rode o colcon build dentro do tmux
# depois:
tmux attach -t build
```
