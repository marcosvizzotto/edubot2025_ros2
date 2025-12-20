#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')

        # Parâmetros (mudáveis via --ros-args -p ...)
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('linear_speed', 0.12)   # m/s
        self.declare_parameter('angular_speed', 0.8)   # rad/s
        self.declare_parameter('side_time', 1.5)       # s
        self.declare_parameter('start_delay', 1.0)     # s (tempo pro controller subir)
        self.declare_parameter('rate_hz', 20.0)        # Hz
        self.declare_parameter('num_sides', 4)
        self.declare_parameter('turn_time_scale', 1.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        self.side_time = float(self.get_parameter('side_time').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.num_sides = int(self.get_parameter('num_sides').value)
        self.turn_time_scale = float(self.get_parameter('turn_time_scale').value)

        if abs(self.w) < 1e-6:
            raise ValueError("angular_speed (w) não pode ser 0.")

        self.turn_time = ((math.pi / 2.0) / abs(self.w)) * self.turn_time_scale  # 90 graus em rad / rad/s

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(
            f"Quadrado: topic={self.cmd_vel_topic} | v={self.v:.3f} m/s | w={self.w:.3f} rad/s | "
            f"side_time={self.side_time:.2f}s | turn_time={self.turn_time:.2f}s | start_delay={self.start_delay:.2f}s"
        )

        # Máquina de estados simples (sem sleeps bloqueando)
        self.state = 'WAIT'
        self.side_idx = 0
        self.deadline = self.get_clock().now() + Duration(seconds=self.start_delay)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

    def publish_twist(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def on_timer(self):
        now = self.get_clock().now()

        if self.state == 'WAIT':
            self.publish_twist(0.0, 0.0)
            if now >= self.deadline:
                self.state = 'DRIVE'
                self.deadline = now + Duration(seconds=self.side_time)
                self.get_logger().info(f"Lado {self.side_idx + 1}/4: andando reto")

        elif self.state == 'DRIVE':
            self.publish_twist(self.v, 0.0)
            if now >= self.deadline:
                self.state = 'TURN'
                self.deadline = now + Duration(seconds=self.turn_time)
                self.get_logger().info("Girando ~90°")

        elif self.state == 'TURN':
            # Mantém o sinal do w que você definiu
            self.publish_twist(0.0, self.w)
            if now >= self.deadline:
                self.side_idx += 1
                if self.side_idx >= self.num_sides:
                    self.state = 'DONE'
                    self.deadline = now + Duration(seconds=0.2)
                    self.get_logger().info("Quadrado finalizado. Parando...")
                else:
                    self.state = 'DRIVE'
                    self.deadline = now + Duration(seconds=self.side_time)
                    self.get_logger().info(f"Lado {self.side_idx + 1}/4: andando reto")

        elif self.state == 'DONE':
            self.publish_twist(0.0, 0.0)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
