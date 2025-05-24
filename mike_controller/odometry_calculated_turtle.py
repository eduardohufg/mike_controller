#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import Float64
from turtlesim.msg import Pose

class OdometryCalculatedTurtle(Node):
    def __init__(self):
        super().__init__('odometry_calculated_turtle')
        self.get_logger().info('Odometry node started')

        # 1) Parámetros
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base',   0.17)   # m
        self.r = self.get_parameter('wheel_radius').value
        self.l = self.get_parameter('wheel_base').value

        # 2) I/O
        qos = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(Float64,
                                 '/turtle1/left_wheel_velocity',
                                 self.cb_left, qos)
        self.create_subscription(Float64,
                                 '/turtle1/right_wheel_velocity',
                                 self.cb_right, qos)
        self.pub = self.create_publisher(Pose, '/odom_turtle', 10)

        # 3) Estado
        self.ωL = 0.0     # rad/s
        self.ωR = 0.0
        self.x  = 0.0     # m
        self.y  = 0.0
        self.q  = 0.0     # rad
        self.last = time.time()

        self.create_timer(0.1, self.update)   # 100 Hz

    # ───────────────────────── callbacks
    def cb_left (self, msg): self.ωL = msg.data
    def cb_right(self, msg): self.ωR = msg.data

    # ───────────────────────── integración
    def update(self):
        now = time.time()
        dt  = now - self.last
        self.last = now

        v = (self.r / 2.0) * (self.ωR + self.ωL)       # m/s
        w = (self.r / self.l) * (self.ωR - self.ωL)    # rad/s

        if abs(w) > 1e-6:
            # movimiento en arco
            dx = (v / w) * (math.sin(self.q + w*dt) - math.sin(self.q))
            dy = (v / w) * (-math.cos(self.q + w*dt) + math.cos(self.q))
        else:
            dx = v * dt * math.cos(self.q)
            dy = v * dt * math.sin(self.q)

        # integrar pose
        self.x += dx
        self.y += dy
        self.q += w * dt
        # opcional: normalizar q a [-π, π]
        self.q = (self.q + math.pi) % (2*math.pi) - math.pi

        # publicar
        pose = Pose()
        pose.x      = self.x
        pose.y      = self.y
        pose.theta  = self.q
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdometryCalculatedTurtle())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
