# mybot_apps — Comandos de teste e calibração

Este pacote contém pequenos *apps ROS 2* para testar encoders, odometria e ajudar na calibração de um robô diferencial usando `ros2_control` + `diff_drive_controller`.

> **Assumido:** você está em `~/ros2_ws` e está usando ROS 2 Humble.

---

## 0) Build / bringup (fluxo padrão)

Em um terminal (Terminal 1):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -r -y --from-paths src --ignore-src
colcon build --symlink-install --executor sequential --parallel-workers 1
source install/setup.bash

# bringup (URDF + ros2_control + diff_drive_controller + EKF)
ros2 launch mybot_bringup bringup.launch.py
```

---

## 1) Checagens rápidas (tópicos / controllers)

Em outro terminal (Terminal 2):

```bash
source ~/ros2_ws/install/setup.bash

# controllers
ros2 control list_controllers
ros2 control list_hardware_interfaces

# odom do diff_drive_controller
ros2 topic echo /diff_drive_controller/odom --once

# odom filtrada (EKF)
ros2 topic echo /odometry/filtered --once

# joint states (use QoS sensor_data)
ros2 topic echo /joint_states --once --qos-profile sensor_data

# taxa de odom
ros2 topic hz /diff_drive_controller/odom
```

---

## 2) Enviar comando de velocidade por tempo controlado (sem “timeout”)

O `ros2 topic pub` pode ficar esperando subscriber. Para evitar “timeout não andou”, use `--times` e *deixe ele terminar sozinho*.

**Exemplo:** publicar a 20 Hz por 5 s → `--times 100`.

```bash
# anda 5s
ros2 topic pub -r 20 --times 100 \
  /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"

# stop 1s (opcional)
ros2 topic pub -r 20 --times 20 \
  /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

> Dica: para “stop” não precisa ser 1 s perfeito — o mais importante é **enviar zero** depois.

---

## 3) Medir deslocamento (régua/trena) + conferir odom

Fluxo típico do teste “reto”:

```bash
# antes
ros2 topic echo /diff_drive_controller/odom --once

# manda andar (ex.: 5s)
ros2 topic pub -r 20 --times 100 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"

# depois
ros2 topic echo /diff_drive_controller/odom --once
```

> Para resetar a odom entre testes, normalmente você reinicia o bringup (`Ctrl+C` no Terminal 1 e roda o launch de novo).

---

## 4) joint_delta (medir quantas “voltas” o ROS acha que a roda fez)

Útil para conferir `ticks_per_rev` e direção do encoder.

```bash
# esquerda
ros2 run mybot_apps joint_delta --ros-args \
  -p joint_name:=left_wheel_joint \
  -p physical_revs:=10

# direita
ros2 run mybot_apps joint_delta --ros-args \
  -p joint_name:=right_wheel_joint \
  -p physical_revs:=10
```

Durante o teste você gira a roda manualmente e observa:
- `Δθ` em rad
- `rev_estimadas = Δθ / (2π)`

---

## 5) straight_calib (comparar ΔθL vs ΔθR num comando reto)

Esse teste publica `cmd_vel` por `duration` e compara os deslocamentos angulares medidos em `/joint_states`.

```bash
# exemplo no chão
ros2 run mybot_apps straight_calib --ros-args -p v:=0.10 -p duration:=5.0
```

Saída típica:
- `ΔθL`, `ΔθR`
- `ratio(L/R)`
- `yaw_est`

---

## 6) straight_hold (tentar segurar heading com feedback da odom)

Você pode usar odom do diff_drive_controller **ou** a odom filtrada do EKF.

```bash
# usando /diff_drive_controller/odom (default)
ros2 run mybot_apps straight_hold --ros-args \
  -p v:=0.10 -p duration:=5.0 -p k_yaw:=2.0

# usando /odometry/filtered (EKF)
ros2 run mybot_apps straight_hold --ros-args \
  -p odom_topic:=/odometry/filtered -p v:=0.10 -p duration:=5.0 -p k_yaw:=2.0
```

> Se o ganho estiver alto demais, pode começar a “oscilar/dançar” (vira para um lado e para o outro).

---

## 7) Parâmetros úteis para inspecionar (diff_drive_controller)

```bash
ros2 param get /diff_drive_controller left_wheel_radius_multiplier
ros2 param get /diff_drive_controller right_wheel_radius_multiplier

# (se existirem no teu YAML)
ros2 param get /diff_drive_controller wheel_separation_multiplier
ros2 param get /diff_drive_controller cmd_vel_timeout
```

---

## 8) QoS (quando usar `--qos-profile sensor_data`)

Sempre que ler `joint_states`:

```bash
ros2 topic echo /joint_states --once --qos-profile sensor_data
```

---

## 9) Observações rápidas (o que olhar quando “curva pra direita”)

Checklist mínimo de diagnóstico em campo:

1. Rodar o teste reto com `--times` (ex.: 5 s).
2. Medir distância e desvio lateral com trena.
3. Pegar `odom --once` antes/depois e comparar com sua medida.
4. Rodar `straight_calib` e comparar `ratio(L/R)`.


## 10) Girar apenas uma roda e a outra praticamente parada
```bash
source install/setup.bash

ros2 topic pub -r 20 --times 80 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: +1.03}}"

ros2 topic pub -r 20 --times 80 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: -1.03}}"
```