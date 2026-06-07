import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from math import atan2, sqrt

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        # State [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3) * 0.1
        
        # TRUST PARAMETERS
        # Q: We trust our integration less (0.1) because wheels slip
        self.Q = np.diag([0.1, 0.1, 0.05])
        # R: We trust SLAM very much (0.01) - SLAM is the Absolute Truth
        self.R = np.diag([0.01, 0.01, 0.005])
        
        self.last_time = None
        self.initialized = False

        self.create_subscription(Odometry, '/rover/odometry', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.slam_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/kalman_pose', 10)
        self.create_timer(0.1, self.publish_pose)

        self.get_logger().info('Localization: Reality-Lock Mode Active.')

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0 or dt > 0.5: return

        # --- VELOCITY SANITIZATION ---
        # The rover physically cannot move faster than 0.5 m/s. 
        # If odom says 5.0 m/s, it's a slip. We cap it.
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        if abs(v) > 0.6: 
            v = np.sign(v) * 0.6 # Cap hallucinated speed
            
        # Prediction (Kinematic Model)
        self.x[0] += v * np.cos(self.x[2]) * dt
        self.x[1] += v * np.sin(self.x[2]) * dt
        self.x[2] += omega * dt
        self.x[2] = atan2(np.sin(self.x[2]), np.cos(self.x[2]))
        
        self.P = self.P + self.Q * dt
        self.initialized = True

    def slam_callback(self, msg):
        """ SLAM is the Authority - Force the brain to match reality """
        sx = msg.pose.pose.position.x
        sy = msg.pose.pose.position.y
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        sth = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

        # Kalman Correction
        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)

        z = np.array([sx, sy, sth])
        innovation = z - self.x
        innovation[2] = atan2(np.sin(innovation[2]), np.cos(innovation[2]))

        self.x = self.x + K @ innovation
        self.x[2] = atan2(np.sin(self.x[2]), np.cos(self.x[2]))
        self.P = (np.eye(3) - K) @ self.P

    def publish_pose(self):
        if not self.initialized: return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.x[0])
        msg.pose.position.y = float(self.x[1])
        msg.pose.orientation.z = float(np.sin(self.x[2]/2))
        msg.pose.orientation.w = float(np.cos(self.x[2]/2))
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
