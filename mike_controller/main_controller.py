import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import tf_transformations
import sys

class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Main Controller node has started")

        self.declare_parameter("mode", 0)
        self.declare_parameter("controller", 0)
        self.declare_parameter("path", 0)

        self.mode = self.get_parameter("mode").value
        self.path = self.get_parameter("path").value
        self.controller = self.get_parameter("controller").value

        if self.mode == 0:
            self.create_subscription(Pose, "/odom_turtle", self.callback_odom, 1)

        elif self.mode == 1:
            self.create_subscription(Odometry, "/odom", self.callback_odom, 1)

        if self.path == 0:
            self.sub_points = self.create_subscription(Point, "/next_point", self.callback_points, 10) 

        elif self.path == 1:
            self.sub_points = self.create_subscription(Point, "/next_point_traj", self.callback_points, 10)


    
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_turtle = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)
   




        self.create_timer(0.01, self.control_loop)

        self.x = None
        self.y = None
        self.theta = None

        self.x_d = None
        self.y_d = None

        self.arrtived = Bool()


    def callback_odom(self, msg):

        if self.mode == 0:
            print("Odometry turtle")
            self.x = msg.x
            self.y = msg.y
            self.theta = msg.theta
        elif self.mode == 1:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y

            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.theta = tf_transformations.euler_from_quaternion(orientation_list)[2]

    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y

        self.get_logger().info(f"Next point: {self.x_d}, {self.y_d}")

    def control_loop(self):

        if self.controller == 0:
            self.turn_then_go(0.2, 1.0)
        elif self.controller == 1:
            self.turn_while_go(0.2, 1.0)
        elif self.controller == 2:
            self.laypunov()
        

    def turn_then_go(self, kv, kw):
        if self.x is not None and self.y is not None and self.x_d is not None and self.y_d is not None:
                msg = Twist()

                Dx = self.x_d - self.x
                Dy = self.y_d - self.y

                distance = math.sqrt(Dx**2 + Dy**2)

                angle_to_goal = math.atan2(Dy, Dx)
                angle_error = angle_to_goal - self.theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

                #self.get_logger().info(f"Distance to target: {distance}")

                if distance > 0.1:
                    if abs(angle_error) > 0.08:  # Fase de rotación
                        msg.linear.x = 0.0
                        msg.angular.z = kw * angle_error
                    else:  # Fase de avance
                        msg.linear.x = kv
                        msg.angular.z = kw * angle_error

                    self.arrtived.data = False
                    self.pub_arrived.publish(self.arrtived)
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0

                    self.arrtived.data = True
                    self.pub_arrived.publish(self.arrtived)

                self.pub.publish(msg)
                self.pub_turtle.publish(msg)
            
 

            

    def turn_while_go(self, kv, kw):
        if self.x is not None and self.y is not None and self.x_d is not None and self.y_d is not None:
                msg = Twist()

                Dx = self.x_d - self.x
                Dy = self.y_d - self.y

                distance = math.sqrt(Dx**2 + Dy**2)

                angle_to_goal = math.atan2(Dy, Dx)
                angle_error = angle_to_goal - self.theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

                #self.get_logger().info(f"Distance to target: {distance}")

                if distance > 0.1:
                    msg.linear.x = kv
                    msg.angular.z = kw * angle_error

                    self.arrtived.data = False
                    self.pub_arrived.publish(self.arrtived)
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0

                    self.arrtived.data = True
                    self.pub_arrived.publish(self.arrtived)

                self.pub.publish(msg)
                self.pub_turtle.publish(msg)
                
            
        
            

    def laypunov(self):
        K1 = 0.5
        K2 = 0.5
        q2 = 0.5
        EPS = 1e-3

        max_speed = 0.2
        max_angular_speed = 1.0          

        if None in (self.x, self.y, self.theta, self.x_d, self.y_d):
            return                                        

   
        dx   = self.x_d - self.x
        dy   = self.y_d - self.y
        l    = math.hypot(dx, dy)                             
        zeta = math.atan2(dy, dx) - self.theta              
        zeta = math.atan2(math.sin(zeta), math.cos(zeta))   

    
        psi  = zeta                                          

        u = K1 * math.cos(zeta) * l

        z_safe = zeta if abs(zeta) > EPS else EPS
        w = ( K2 * zeta
            + (K1 / z_safe) * math.cos(zeta) * math.sin(zeta)
              * (zeta + q2 * psi) )

        cmd = Twist()
        cmd.linear.x  = min(max(u, -max_speed), max_speed)
        cmd.angular.z = min(max(w, -max_angular_speed), max_angular_speed)

        self.pub.publish(cmd)
        self.pub_turtle.publish(cmd)

        arrived = abs(l) < 0.05                     
        self.arrtived.data = arrived
        self.pub_arrived.publish(self.arrtived)

                        
                       
        

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    