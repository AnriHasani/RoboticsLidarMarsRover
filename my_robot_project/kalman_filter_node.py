import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2


class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        # State vector [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3) * 1.0
        self.I = np.eye(3)
        self.Q = np.diag([0.01, 0.01, 0.001])
        self.R = np.diag([0.05, 0.05, 0.01])
        self.H = np.eye(3)
        self.last_time = None
        self.log = []
        self.start_time = self.get_clock().now()
        self.odom_count = 0

        # SLAM reception tracking
        self.slam_received = False
        self.slam_log_count = 0
        self.scan_count = 0
        self.scan_have_valid = False

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/rover/odometry', self.odom_callback, 10)
        self.slam_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.slam_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/kalman_pose', 10)

        # Periodic status timer (every 5s)
        self.create_timer(5.0, self.log_slam_status)
        # Diagnostic timer (every 10s for topic checks)
        self.create_timer(10.0, self.diagnostic_check)

        # Timer to publish at 10Hz
        self.create_timer(0.1, self.publish_pose)

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║   Kalman Filter Node started        ║')
        self.get_logger().info('║   Subscribed to /pose (SLAM)        ║')
        self.get_logger().info('╚══════════════════════════════════════╝')

    def log_slam_status(self):
        if not self.slam_received:
            self.get_logger().warn('⏳ No /pose data from slam_toolbox yet — waiting...')
        else:
            self.get_logger().info(
                f'SLAM fusion OK — {self.slam_log_count} /pose updates processed')

    def diagnostic_check(self):
        topics = self.get_topic_names_and_types()
        has_scan = any('/scan' in t for t, _ in topics)
        has_pose_pub = any('/pose' in t for t, _ in topics)
        self.get_logger().info(
            f'DIAG: /scan topic exists={has_scan} | /pose publisher exists={has_pose_pub} | '
            f'scan_msgs_received={self.scan_count} | scan_has_valid={self.scan_have_valid} | '
            f'slam_msgs_received={self.slam_log_count}')

    def scan_callback(self, msg):
        self.scan_count += 1
        if not self.scan_have_valid:
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid:
                self.scan_have_valid = True
                self.get_logger().info(
                    f'SCAN: frame_id="{msg.header.frame_id}" | {len(valid)}/{len(msg.ranges)} valid rays | '
                    f'min={min(valid):.2f}m max={max(valid):.2f}m')

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.x[0] = msg.pose.pose.position.x
            self.x[1] = msg.pose.pose.position.y
            qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            self.x[2] = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            self.get_logger().info(f'Initialized Kalman Filter at: {self.x}')
            return

        self.odom_count += 1
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            return

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        x, y, theta = self.x
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + omega * dt
        theta_new = atan2(np.sin(theta_new), np.cos(theta_new))

        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1, v * np.cos(theta) * dt],
            [0, 0, 1]
        ])

        self.x = np.array([x_new, y_new, theta_new])
        self.P = F @ self.P @ F.T + self.Q

    def slam_callback(self, msg):
        self.slam_received = True
        self.slam_log_count += 1
        slam_x = msg.pose.pose.position.x
        slam_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        slam_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        self.get_logger().info(
            f'SLAM /pose #{self.slam_log_count}: '
            f'x={slam_x:.2f} y={slam_y:.2f} θ={slam_theta:.2f}',
            throttle_duration_sec=2.0
        )

        z = np.array([slam_x, slam_y, slam_theta])
        innovation = z - self.H @ self.x
        innovation[2] = atan2(np.sin(innovation[2]), np.cos(innovation[2]))

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.x[2] = atan2(np.sin(self.x[2]), np.cos(self.x[2]))
        self.P = (self.I - K @ self.H) @ self.P

    def publish_pose(self):
        if self.odom_count < 5:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.x[0])
        msg.pose.position.y = float(self.x[1])
        msg.pose.orientation.z = float(np.sin(self.x[2] / 2))
        msg.pose.orientation.w = float(np.cos(self.x[2] / 2))
        self.pose_pub.publish(msg)

    def save_log(self):
        import csv
        try:
            with open('/tmp/kalman_log.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'odom_x', 'odom_y', 'kalman_x', 'kalman_y'])
                writer.writerows(self.log)
            self.get_logger().info('Log saved to /tmp/kalman_log.csv')
        except Exception as e:
            self.get_logger().error(f'Failed to save log: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_log()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
