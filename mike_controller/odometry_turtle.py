import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class OdometryTurtle(Node):
    def __init__(self):
        super().__init__("odometry_turtle")
        self.get_logger().info("Odometry turtle node has started")
        self.create_subscription(Pose, "/turtle1/pose", self.callback_odom, 1)
        self.pub = self.create_publisher(Pose, "/odom_turtle", 1)
        self.create_timer(0.01, self.control_loop)

        self.x = None
        self.y = None
        self.theta = None

    def callback_odom(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def control_loop(self):
        if self.x is not None and self.y is not None and self.theta is not None:
            odom_msg = Pose()
            odom_msg.x = self.x
            odom_msg.y = self.y
            odom_msg.theta = self.theta
            self.pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()