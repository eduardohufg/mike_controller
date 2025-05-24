#!/usr/bin/env python3
import os, csv, rclpy, sys
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlesim.msg import Pose

class OdomErrorLogger(Node):
    """
    Registra en CSV el error (pose real – odometría compensada) cada vez
    que se recibe /arrived==True.  Se apaga cuando /path_finished==True.
    """

    def __init__(self):
        super().__init__('odom_error_logger')

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('pose_topic',        '/turtle1/pose')
        self.declare_parameter('odom_topic',        '/odom_turtle')
        self.declare_parameter('arrived_topic',     '/arrived')
        self.declare_parameter('path_finished_topic','/path_finished')
        self.declare_parameter('csv_path',          '~/odom_error.csv')

        pose_topic     = self.get_parameter('pose_topic').value
        odom_topic     = self.get_parameter('odom_topic').value
        arrived_topic  = self.get_parameter('arrived_topic').value
        finished_topic = self.get_parameter('path_finished_topic').value
        self.csv_path  = os.path.expanduser(
                             self.get_parameter('csv_path').value)

        qos = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(Pose, pose_topic,        self.cb_abs_pose,  qos)
        self.create_subscription(Pose, odom_topic,        self.cb_odom_pose, qos)
        self.create_subscription(Bool, arrived_topic,     self.cb_arrived,   qos)
        self.create_subscription(Bool, finished_topic,    self.cb_finished,  qos)

        # ── Estado ────────────────────────────────────────────────────
        self.abs_pose = self.odom_pose = None
        self.offset_x = self.offset_y = None
        self.waypoint_idx = 0
        self.finished = False

        # Crea archivo con cabecera si no existe
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'time_s', 'wp_idx',
                    'abs_x', 'abs_y',
                    'odom_x', 'odom_y',
                    'err_x', 'err_y'
                ])
        self.get_logger().info(f'✔ Registrando en "{self.csv_path}"')

    # ───────────────────────── Callbacks ──────────────────────────────
    def cb_abs_pose(self, msg: Pose):
        self.abs_pose = msg
        self.compute_offset_once()

    def cb_odom_pose(self, msg: Pose):
        self.odom_pose = msg
        self.compute_offset_once()

    def compute_offset_once(self):
        """Ajuste inicial de marcos (solo la primera vez)."""
        if (self.offset_x is None and
            self.abs_pose is not None and
            self.odom_pose is not None):
            self.offset_x = self.abs_pose.x - self.odom_pose.x
            self.offset_y = self.abs_pose.y - self.odom_pose.y
            self.get_logger().info(
                f'Desfase inicial: Δx={self.offset_x:.3f}, Δy={self.offset_y:.3f}')

    def cb_arrived(self, msg: Bool):
        if self.finished or not msg.data:
            return
        if (self.abs_pose is None or
            self.odom_pose is None or
            self.offset_x is None):
            self.get_logger().warn('Datos insuficientes; se omite este waypoint')
            return

        # Odom en el mismo marco global
        odom_x = self.odom_pose.x + self.offset_x
        odom_y = self.odom_pose.y + self.offset_y

        err_x = self.abs_pose.x - odom_x
        err_y = self.abs_pose.y - odom_y
        t_sec = self.get_clock().now().nanoseconds * 1e-9

        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                f'{t_sec:.3f}', self.waypoint_idx,
                f'{self.abs_pose.x:.3f}',  f'{self.abs_pose.y:.3f}',
                f'{odom_x:.3f}',          f'{odom_y:.3f}',
                f'{err_x:.3f}',           f'{err_y:.3f}'
            ])

        self.get_logger().info(
            f'WP {self.waypoint_idx}: err_x={err_x:.3f}, err_y={err_y:.3f}')
        self.waypoint_idx += 1

    def cb_finished(self, msg: Bool):
        if msg.data:
            self.finished = True
            self.get_logger().info('Ruta terminada; deteniendo registro')
            rclpy.shutdown()                   # apaga limpio

# ───────────────────────── main ───────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdomErrorLogger())
    sys.exit()

if __name__ == '__main__':
    main()
