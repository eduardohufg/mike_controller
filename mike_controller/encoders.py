import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Encoder(Node):
    def __init__(self):
        super().__init__("encoder")
        self.get_logger().info("Encoder node has started")

        self.create_subscription(Twist, "/turtle1/cmd_vel", self.callback_velocity, 10)

        self.pub_left = self.create_publisher(Float64, "/turtle1/left_wheel_velocity", 1)
        self.pub_right = self.create_publisher(Float64, "/turtle1/right_wheel_velocity", 1)
        self.create_timer(0.01, self.control_loop)
        self.linear_velocity = None
        self.angular_velocity = None
        self.left_velocity = None
        self.right_velocity = None
        self.left_msg = Float64()
        self.right_msg = Float64()

        self.r = 0.05
        self.l = 0.17


    def callback_velocity(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z



    def control_loop(self):

        if self.linear_velocity is not None and self.angular_velocity is not None:
            self.left_velocity = (self.linear_velocity - (self.angular_velocity * self.l / 2)) / self.r
            self.right_velocity = (self.linear_velocity + (self.angular_velocity * self.l / 2)) / self.r

            self.left_msg.data = self.left_velocity
            self.right_msg.data = self.right_velocity

            self.pub_left.publish(self.left_msg)
            self.pub_right.publish(self.right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Encoder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
