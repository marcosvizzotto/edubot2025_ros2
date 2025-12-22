#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class StraightHoldHeading(Node):
    def __init__(self):
        super().__init__('straight_hold_heading')

        # tópicos
        self.declare_parameter('cmd_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('odom_topic', '/odometry/filtered')  # prefira EKF

        # movimento
        self.declare_parameter('v', 0.08)         # reduza p/ dar tempo de corrigir
        self.declare_parameter('duration', 6.0)
        self.declare_parameter('start_delay', 0.5)

        # controle PI de yaw
        self.declare_parameter('k_p', 6.0)
        self.declare_parameter('k_i', 1.0)
        self.declare_parameter('i_limit', 0.6)    # limita integral
        self.declare_parameter('w_limit', 1.5)    # limite angular (maior que antes)

        # deadband / quantização: “kick” mínimo
        self.declare_parameter('w_min', 0.25)     # angular mínimo quando precisar corrigir
        self.declare_parameter('e_min', 0.02)     # só aplica kick se erro > ~1.1°

        # debug
        self.declare_parameter('print_debug', True)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.v = float(self.get_parameter('v').value)
        self.duration = float(self.get_parameter('duration').value)
        self.start_delay = float(self.get_parameter('start_delay').value)

        self.k_p = float(self.get_parameter('k_p').value)
        self.k_i = float(self.get_parameter('k_i').value)
        self.i_limit = float(self.get_parameter('i_limit').value)
        self.w_limit = float(self.get_parameter('w_limit').value)

        self.w_min = float(self.get_parameter('w_min').value)
        self.e_min = float(self.get_parameter('e_min').value)

        self.print_debug = bool(self.get_parameter('print_debug').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.have_odom = False
        self.yaw = 0.0
        self.yaw0 = None

        self.i = 0.0  # integral

        self.get_logger().info(
            f"StraightHoldHeading(PI): cmd={self.cmd_topic} odom={self.odom_topic} "
            f"v={self.v} duration={self.duration}s k_p={self.k_p} k_i={self.k_i} "
            f"w_min={self.w_min} w_limit={self.w_limit}"
        )

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def publish(self, lin, ang):
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub.publish(m)

    def run(self):
        # espera odom
        t0 = time.time()
        while rclpy.ok() and (not self.have_odom):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 5.0:
                self.get_logger().error(f"Sem odom em {self.odom_topic}.")
                return

        time.sleep(self.start_delay)

        # referência
        self.yaw0 = self.yaw
        self.i = 0.0
        self.get_logger().info(f"Yaw referência = {self.yaw0:.3f} rad")

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        end = time.time() + self.duration

        k = 0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.001)

            e = wrap_pi(self.yaw - self.yaw0)     # erro de yaw
            self.i += e * dt                      # integral
            self.i = max(-self.i_limit, min(self.i_limit, self.i))

            w = -(self.k_p * e + self.k_i * self.i)

            # limita
            w = max(-self.w_limit, min(self.w_limit, w))

            # “kick” mínimo pra vencer deadband/quantização
            if abs(e) > self.e_min and abs(w) < self.w_min:
                w = math.copysign(self.w_min, w)

            self.publish(self.v, w)

            if self.print_debug and (k % 25 == 0):
                # a cada ~0.5s
                self.get_logger().info(f"e={e:+.3f} rad | w_cmd={w:+.3f} rad/s | i={self.i:+.3f}")
            k += 1

            time.sleep(dt)

        # stop
        for _ in range(10):
            self.publish(0.0, 0.0)
            time.sleep(dt)

        self.get_logger().info("Finalizado (stop).")


def main():
    rclpy.init()
    node = StraightHoldHeading()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
