# My Robot Bringup

Pacote de *bringup* do robô: URDF/Xacro + controllers + EKF + launch.

Ele sobe:

- `robot_state_publisher` (TF do URDF)
- `ros2_control_node` com hardware `bluepill_ros2_control`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `robot_localization` (`ekf_node`) para `/odometry/filtered` + TF `odom -> base_link`

---

## Estrutura

```
mybot_bringup/
  urdf/
    mybot.urdf.xacro
  config/
    controllers.yaml
    ekf.yaml
  launch/
    bringup.launch.py
  setup.py
  package.xml
```

> O `setup.py` instala `config/` em `install/.../share/mybot_bringup/config/`.  
> Se aparecer “Parameter file path is not a file”, normalmente o `ekf.yaml`
> não está em `src/mybot_bringup/config/` ou o pacote não foi rebuildado.

---

## Parâmetros físicos (atuais)

- `wheel_radius`: **0.0325 m**
- `wheel_separation`: **0.155 m**
- `ticks_per_rev`: **estimado 2112** (recomendado confirmar)
- Encoders: **absolutos** (contagem acumulada)

---

## controllers.yaml — pontos críticos

### 1) Tópico de comando (Stamped vs Unstamped)
Para usar `geometry_msgs/Twist`:

```yaml
diff_drive_controller:
  ros__parameters:
    use_stamped_vel: false
```

Com isso, o controller assina:

- `/diff_drive_controller/cmd_vel_unstamped`

Se você colocar `use_stamped_vel: true`, ele assina:

- `/diff_drive_controller/cmd_vel` (tipo `TwistStamped`)

### 2) TF do odom
Se o EKF publica TF `odom -> base_link`, evite TF duplicado do diff_drive:

```yaml
enable_odom_tf: false
```

### 3) Roda invertida (sem mexer no Bluepill)
Se uma roda girar ao contrário quando manda “reto”, aplique multiplicador -1 no lado invertido.  
Exemplo (direita invertida):

```yaml
right_wheel_radius_multiplier: -1.0
left_wheel_radius_multiplier: 1.0
```

---

## ekf.yaml (robot_localization)

Configuração típica 2D, usando:

- `/diff_drive_controller/odom` (wheel odom)
- `/imu` (MPU9250)

Exemplo:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true

    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /diff_drive_controller/odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    imu0: /imu
    # aqui usamos principalmente yaw_rate (angular_velocity.z)
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
```

> O driver MPU9250 usado aqui publica `frame_id: base_link` e aceleração em m/s² (z ~ 9.8 parado).

---

## Executando o bringup

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mybot_bringup bringup.launch.py
```

---

## Verificando se está “tudo de pé”

### Controllers
```bash
ros2 control list_controllers
```

Esperado:
- `joint_state_broadcaster ... active`
- `diff_drive_controller ... active`

### Tópicos
```bash
ros2 topic list | grep -E "odom|filtered|cmd_vel"
```

Esperado:
- `/diff_drive_controller/odom`
- `/odometry/filtered`
- `/diff_drive_controller/cmd_vel_unstamped`

### Assinatura do cmd_vel
```bash
ros2 topic info /diff_drive_controller/cmd_vel_unstamped
```

Esperado:
- `Subscription count: 1`

---

## Movendo o robô

Andar reto:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
"{linear: {x: 0.12}, angular: {z: 0.0}}"
```

Girar:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.8}}"
```

Parar:

```bash
ros2 topic pub -1 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## TF (odom -> base_link)

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Se no começo aparecer “Invalid frame ID odom”, espere alguns segundos: é comum antes do TF entrar no buffer.

---

## Build (somente este pacote)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mybot_bringup \
  --executor sequential --parallel-workers 1
source install/setup.bash
```
