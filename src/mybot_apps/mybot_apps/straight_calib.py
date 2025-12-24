#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

def get_yaw_from_dtheta(dtheta_l, dtheta_r, wheel_radius, wheel_separation):
    # yaw ≈ r*(ΔθR - ΔθL)/b
    return wheel_radius * (dtheta_r - dtheta_l) / wheel_separation

class StraightCalib(Node):
    def __init__(self):
        super().__init__('straight_calib')

        self.declare_parameter('cmd_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')

        self.declare_parameter('v', 0.10)          # m/s (comandado)
        self.declare_parameter('duration', 5.0)    # s
        self.declare_parameter('rate', 20.0)       # Hz

        self.declare_parameter('wheel_radius', 0.0325)      # m
        self.declare_parameter('wheel_separation', 0.155)   # m

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.js_topic  = self.get_parameter('joint_states_topic').value
        self.lj = self.get_parameter('left_joint').value
        self.rj = self.get_parameter('right_joint').value

        self.v = float(self.get_parameter('v').value)
        self.duration = float(self.get_parameter('duration').value)
        self.rate = float(self.get_parameter('rate').value)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.b = float(self.get_parameter('wheel_separation').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(JointState, self.js_topic, self.on_js, 10)

        self.have_start = False
        self.theta0_l = 0.0
        self.theta0_r = 0.0
        self.theta_l = 0.0
        self.theta_r = 0.0

        self.t_start = None
        self.timer = self.create_timer(1.0 / self.rate, self.loop)

        self.get_logger().info(
            f"StraightCalib: v={self.v} m/s, T={self.duration}s, rate={self.rate}Hz | "
            f"r={self.r} m, b={self.b} m | cmd={self.cmd_topic}"
        )

    def on_js(self, msg: JointState):
        try:
            iL = msg.name.index(self.lj)
            iR = msg.name.index(self.rj)
        except ValueError:
            return

        self.theta_l = float(msg.position[iL])
        self.theta_r = float(msg.position[iR])

        if not self.have_start:
            self.theta0_l = self.theta_l
            self.theta0_r = self.theta_r
            self.have_start = True
            self.get_logger().info("Recebi joint_states. Iniciando teste (publicando cmd).")

    def loop(self):
        if not self.have_start:
            return

        now = self.get_clock().now()
        if self.t_start is None:
            self.t_start = now

        t = (now - self.t_start).nanoseconds * 1e-9

        cmd = Twist()
        if t <= self.duration:
            cmd.linear.x = self.v
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        # stop e finaliza uma vez
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

        dtheta_l = self.theta_l - self.theta0_l
        dtheta_r = self.theta_r - self.theta0_r

        # “distância pelas rodas” (só pra referência)
        s_est = self.r * (dtheta_l + dtheta_r) * 0.5
        yaw_est = get_yaw_from_dtheta(dtheta_l, dtheta_r, self.r, self.b)

        # razão de “quanto uma roda girou a mais que a outra”
        # (se ratio > 1 -> esquerda girou mais -> robô tende a curvar pra direita)
        ratio = abs(dtheta_l) / (abs(dtheta_r) + 1e-9)

        self.get_logger().info(
            f"RESULT: ΔθL={dtheta_l:.3f} rad | ΔθR={dtheta_r:.3f} rad | "
            f"ratio(L/R)={ratio:.4f} | s_est={s_est:.3f} m | yaw_est={yaw_est:.3f} rad"
        )
        self.get_logger().info(
            "Sugestão: se ele curva pra DIREITA, normalmente ratio(L/R)>1. "
            "Tente aumentar levemente o lado DIREITO: cmd_scale_right *= ratio(L/R)."
        )

        rclpy.shutdown()

def main():
    rclpy.init()
    node = StraightCalib()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
