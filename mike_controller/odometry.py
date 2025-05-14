#!/usr/bin/env python3
import rclpy, math, time, tf_transformations
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy import qos

class OdometryClass(Node):
    def __init__(self):
        super().__init__("odometry")
        self.get_logger().info("Robot pose estimation by odometry node.")
        self.create_timer(0.1, self.odometry_callback)
        self.pub = self.create_publisher(Odometry, 'odom', 1)
        self.create_subscription(Float32, 'VelocityEncR', self.wR_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL', self.wL_cb, qos.qos_profile_sensor_data)
        
        # Initialize variables
        self.wR = 0.0 # Right wheel velocity
        self.wL = 0.0 # Left wheel velocity
        self.x = 0.0 # Robot x position
        self.y = 0.0 # Robot y position
        self.q = 0.0 # Robot orientation (yaw)
        self.t0 = time.time() # Initial time
        self.r = 0.0505  # Wheel radius in meters
        self.L = 0.1725  # Distance between wheels in meters
        
        # Initialize the odometry message
        self.odom_msg = Odometry()

    def wR_cb(self, msg):
        self.wR = msg.data
    
    def wL_cb(self, msg):
        self.wL = msg.data

    def odometry_callback(self):
        elapsed_time = time.time() - self.t0  # Calculate elapsed time
        self.t0 = time.time()  # Update the initial time
        # Calculate the robot's position and orientation
        v = (self.wR + self.wL)*self.r / 2.0
        w = (self.wR - self.wL)*self.r / self.L
        self.x += v * math.cos(self.q) * elapsed_time
        self.y += v * math.sin(self.q) * elapsed_time
        self.q += w * elapsed_time

        # Create odometry message
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.q)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    nodeh = OdometryClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()