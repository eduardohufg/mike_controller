#!/usr/bin/env python3
"""
ITAE_logger.py ― Calcula ITAE = ∫ t·|e(t)| dt durante 30 s,
donde e(t) es el error de posición (norma Euclídea) entre:

  • /turtle1/pose          (marco global)
  • /odom_turtle           (odometría)

El desfase inicial entre marcos se elimina con la primera pareja de poses.
Al concluir escribe una fila CSV con ITAE y se apaga.
"""
import os, csv, math, rclpy, sys
from rclpy.node import Node
from turtlesim.msg import Pose

class ITAELogger(Node):
    def __init__(self):
        super().__init__('itae_logger')

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('pose_topic',  '/turtle1/pose')
        self.declare_parameter('odom_topic',  '/odom_turtle')
        self.declare_parameter('csv_path',    '~/itae.csv')
        self.declare_parameter('duration',    30.0)       # s

        pose_topic  = self.get_parameter('pose_topic').value
        odom_topic  = self.get_parameter('odom_topic').value
        self.csv    = os.path.expanduser(self.get_parameter('csv_path').value)
        self.t_max  = float(self.get_parameter('duration').value)

        qos = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(Pose, pose_topic, self.cb_abs,  qos)
        self.create_subscription(Pose, odom_topic, self.cb_odom, qos)

        # ── Estado ────────────────────────────────────────────────────
        self.abs_pose = self.odom_pose = None
        self.offset_x = self.offset_y = None

        self.started  = False
        self.t_start  = None
        self.t_last   = None
        self.ITAE     = 0.0

        self.get_logger().info('ITAE logger listo; esperando poses…')

    # ───────────────────────── callbacks ──────────────────────────────
    def cb_abs(self, msg: Pose):
        self.abs_pose = msg
        self.try_start()

    def cb_odom(self, msg: Pose):
        self.odom_pose = msg
        self.try_start()

    # ───────────────────────── lógica principal ───────────────────────
    def try_start(self):
        # Compensa offset y arranca el integrador una vez.
        if (self.offset_x is None and
            self.abs_pose is not None and
            self.odom_pose is not None):
            self.offset_x = self.abs_pose.x - self.odom_pose.x
            self.offset_y = self.abs_pose.y - self.odom_pose.y
            self.get_logger().info(
                f'Offset inicial: Δx={self.offset_x:.3f}, Δy={self.offset_y:.3f}')

        if (not self.started and
            self.offset_x is not None and
            self.abs_pose is not None and
            self.odom_pose is not None):
            self.started = True
            self.t_start = self.get_clock().now()
            self.t_last  = self.t_start
            # Timer a ~100 Hz para integrar
            self.create_timer(0.01, self.integrate_step)
            self.get_logger().info('Integración ITAE iniciada')

    def integrate_step(self):
        now = self.get_clock().now()
        t_elapsed = (now - self.t_start).nanoseconds * 1e-9
        dt        = (now - self.t_last ).nanoseconds * 1e-9
        self.t_last = now

        # Error absoluto en XY (odometría alineada)
        odom_x = self.odom_pose.x + self.offset_x
        odom_y = self.odom_pose.y + self.offset_y
        err    = math.hypot(self.abs_pose.x - odom_x,
                            self.abs_pose.y - odom_y)

        self.ITAE += t_elapsed * err * dt

        # Termina a los 30 s
        if t_elapsed >= self.t_max:
            self.save_and_exit()

    # ───────────────────────── salida CSV ─────────────────────────────
    def save_and_exit(self):
        first = not os.path.exists(self.csv)
        with open(self.csv, 'a', newline='') as f:
            w = csv.writer(f)
            if first:
                w.writerow(['ITAE', 'dur_s'])
            w.writerow([f'{self.ITAE:.6f}', f'{self.t_max:.3f}'])
        self.get_logger().info(
            f'ITAE = {self.ITAE:.6f} m·s² guardado en "{self.csv}"')
        rclpy.shutdown()

# ───────────────────────── main ───────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ITAELogger())
    sys.exit()

if __name__ == '__main__':
    main()
