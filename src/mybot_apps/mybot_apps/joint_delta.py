#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointDelta(Node):
    def __init__(self):
        super().__init__('joint_delta')

        self.declare_parameter('joint_name', 'right_wheel_joint')
        self.declare_parameter('ticks_per_rev_current', 2112)
        self.declare_parameter('physical_revs', 10)

        self.joint_name = self.get_parameter('joint_name').value
        self.ticks_per_rev_current = float(self.get_parameter('ticks_per_rev_current').value)
        self.physical_revs = float(self.get_parameter('physical_revs').value)

        self.idx = None
        self.start_pos = None
        self.last_print = self.get_clock().now()

        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.get_logger().info(
            f"Monitorando '{self.joint_name}'. "
            f"Gire a roda {self.physical_revs} voltas físicas e pressione Ctrl+C."
        )

    def cb(self, msg: JointState):
        if self.joint_name not in msg.name:
            return
        self.idx = msg.name.index(self.joint_name)
        pos = msg.position[self.idx]

        if self.start_pos is None:
            self.start_pos = pos
            return

        delta = pos - self.start_pos
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds > int(0.3 * 1e9):
            rev_est = delta / (2.0 * math.pi)
            self.get_logger().info(f"Δθ = {delta:.3f} rad | rev_estimadas = {rev_est:.3f}")
            self.last_print = now

    def finalize(self):
        if self.start_pos is None:
            self.get_logger().warn("Não recebi /joint_states ainda.")
            return
        # o último delta já foi impresso; vamos calcular de novo a partir do último msg
        # (simples: peça pro usuário olhar o último print de rev_estimadas)
        self.get_logger().info(
            "Olhe a última linha 'rev_estimadas'. "
            "Novo ticks_per_rev = ticks_atual * (rev_estimadas / rev_fisicas)."
        )

def main():
    rclpy.init()
    node = JointDelta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
