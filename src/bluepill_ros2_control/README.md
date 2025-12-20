# BluePill ROS2 Control
A BluePill é uma placa de desenvolvimento eletrônico baseada no microcontrolador STM32F103C8T6, usando arquitetura ARM Cortex-M3. Este pacote: ROS 2 com **hardware plugin** para `ros2_control` (`hardware_interface::SystemInterface`). Ele faz a ponte entre o ROS 2 e o **Bluepill (STM32)** via **serial**:

- **Envia comandos** para os motores no formato: `"<L,R>"`
- **Recebe encoders absolutos** no formato: `"<ENC_L,ENC_R>"`

O `diff_drive_controller` calcula velocidades de roda e o plugin converte e envia ao Bluepill.

---

## Estrutura correta do pacote

Garanta que os arquivos estejam dentro do pacote:

```
bluepill_ros2_control/
  bluepill_ros2_control.xml
  CMakeLists.txt
  package.xml
  include/bluepill_ros2_control/bluepill_system.hpp
  src/bluepill_system.cpp
```

> Se `bluepill_ros2_control.xml` ficar fora dessa pasta, o build falha com  
> `pluginlib_export_plugin_description_file ... does not exist`.

---

## Permissões de serial

Se der erro para abrir a porta (ex.: `/dev/serial0`):

```bash
sudo usermod -aG dialout $USER
```

Depois faça logout/login.

---

## Parâmetros típicos (via URDF ros2_control)

O plugin é configurado via `<ros2_control>` no URDF/Xacro. Exemplo:

```xml
<ros2_control name="MyBotSystem" type="system">
  <hardware>
    <plugin>bluepill_ros2_control/BluepillSystem</plugin>

    <param name="serial_port">/dev/serial0</param>
    <param name="baud">115200</param>

    <param name="ticks_per_rev">2112</param>
    <param name="max_wheel_angular_velocity">20.0</param>

    <!-- Bluepill envia encoder absoluto -->
    <param name="encoders_are_incremental">false</param>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## Build (somente este pacote)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bluepill_ros2_control \
  --executor sequential --parallel-workers 1
source install/setup.bash
```

---

## Diagnóstico

### Controllers ativos
```bash
ros2 control list_controllers
```

### Componentes do hardware
```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

---

## Erros comuns (e soluções)

### `RCLCPP_ERROR` / `RCLCPP_INFO` “not declared”
Inclua no header:

```cpp
#include <rclcpp/logging.hpp>
```

### `HW_IF_POSITION` / `HW_IF_VELOCITY` “not a member”
Inclua no header:

```cpp
#include <hardware_interface/types/hardware_interface_type_values.hpp>
```

### Serial abre, mas robô não anda
1) Confirme se o controller está ativo:

```bash
ros2 control list_controllers
```

2) Confirme se o tópico de comando correto está com **Subscription count = 1**:

```bash
ros2 topic info /diff_drive_controller/cmd_vel_unstamped
```

Se estiver `Subscription count: 0`, o controller não está assinando aquele tópico (ver README do `mybot_bringup`).
