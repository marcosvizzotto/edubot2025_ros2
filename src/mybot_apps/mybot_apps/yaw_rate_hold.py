#!/usr/bin/env python3
import time
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class YawRateHold(Node):
    def __init__(self):
        super().__init__('yaw_rate_hold')

        # topics
        self.declare_parameter('cmd_in',  '/cmd_vel_in')
        self.declare_parameter('cmd_out', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('imu_topic','/imu/data')

        # when to hold heading
        self.declare_parameter('v_min', 0.15)          # só corrige se estiver andando
        self.declare_parameter('w_hold_thresh', 0.10)  # se usuário pedir curva, não atrapalha

        # controller (PI on yaw-rate)
        self.declare_parameter('k_p', 1.2)     # começa aqui
        self.declare_parameter('k_i', 0.6)     # começa aqui
        self.declare_parameter('i_limit', 0.8)
        self.declare_parameter('w_limit', 1.2)

        # gyro bias calibration
        self.declare_parameter('bias_calib_s', 1.5)    # parado no início
        self.declare_parameter('gyro_lpf_alpha', 0.2)  # filtro 0..1 (0.2 = ok)

        self.cmd_in = self.get_parameter('cmd_in').value
        self.cmd_out = self.get_parameter('cmd_out').value
        self.imu_topic = self.get_parameter('imu_topic').value

        self.v_min = float(self.get_parameter('v_min').value)
        self.w_hold_thresh = float(self.get_parameter('w_hold_thresh').value)

        self.k_p = float(self.get_parameter('k_p').value)
        self.k_i = float(self.get_parameter('k_i').value)
        self.i_limit = float(self.get_parameter('i_limit').value)
        self.w_limit = float(self.get_parameter('w_limit').value)

        self.bias_calib_s = float(self.get_parameter('bias_calib_s').value)
        self.alpha = float(self.get_parameter('gyro_lpf_alpha').value)

        self.pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_in, self.cmd_cb, 10)
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)

        self.last_cmd = Twist()
        self.have_cmd = True  # permite calibrar bias sem precisar receber cmd primeiro

        self.gz_raw = 0.0
        self.gz_filt = 0.0
        self.have_imu = False

        self.bias = 0.0
        self.bias_ready = False
        self.bias_t0 = time.time()
        self.bias_acc = 0.0
        self.bias_n = 0

        self.i = 0.0

        self.rate_hz = 100.0
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"YawRateHold: cmd_in={self.cmd_in} cmd_out={self.cmd_out} imu={self.imu_topic} "
            f"k_p={self.k_p} k_i={self.k_i} w_limit={self.w_limit}"
        )

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.have_cmd = True

    def imu_cb(self, msg: Imu):
        self.gz_raw = float(msg.angular_velocity.z) * (math.pi / 180.0)
        self.have_imu = True

    def tick(self):
        if not self.have_imu:
            return

        # bias calibration (keep robot still during first seconds)
        if not self.bias_ready:
            self.bias_acc += self.gz_raw
            self.bias_n += 1
            if (time.time() - self.bias_t0) >= self.bias_calib_s and self.bias_n > 10:
                self.bias = self.bias_acc / self.bias_n
                self.bias_ready = True
                self.get_logger().info(f"Gyro bias ready: bias={self.bias:+.5f} rad/s")
            # enquanto calibra, repassa comando sem correção (ou zera, se preferir)
            self.pub.publish(self.last_cmd)
            return

        # filter gyro z
        gz = self.gz_raw - self.bias
        self.gz_filt = (1.0 - self.alpha) * self.gz_filt + self.alpha * gz

        v = float(self.last_cmd.linear.x)
        w_user = float(self.last_cmd.angular.z)

        # only hold heading when user is trying to go straight
        hold = (abs(w_user) < self.w_hold_thresh) and (abs(v) > self.v_min)

        w_corr = 0.0
        if hold:
            # PI on yaw-rate: target yaw_rate = 0
            e = self.gz_filt
            self.i = clamp(self.i + e * self.dt, -self.i_limit, self.i_limit)

            w_corr = -(self.k_p * e + self.k_i * self.i)
            w_corr = clamp(w_corr, -self.w_limit, self.w_limit)
        else:
            # reset integral so it doesn't fight commanded turns
            self.i = 0.0

        out = Twist()
        out.linear.x = v
        out.angular.z = w_user + w_corr
        self.pub.publish(out)

def main():
    rclpy.init()
    node = YawRateHold()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
