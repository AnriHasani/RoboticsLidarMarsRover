import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from math import atan2


class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3) * 1.0
        self.Q = np.diag([0.01, 0.01, 0.001])
        self.R = np.diag([0.05, 0.05, 0.01])
        self.H = np.eye(3)
        self.I = np.eye(3)

        self.last_time = None
        self.log = []
        self.start_time = self.get_clock().now()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/my_robot/odometry',
            self.odom_callback,
            10
        )

        self.slam_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_pose',
            self.slam_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/kalman_pose',
            10
        )

        self.create_timer(0.1, self.publish_pose)

        self.get_logger().info('Kalman Filter node started')
        self.get_logger().info(f'Q (process noise):\n{self.Q}')
        self.get_logger().info(f'R (measurement noise):\n{self.R}')

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
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

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        t = (now - self.start_time).nanoseconds / 1e9
        self.log.append([t, odom_x, odom_y, self.x[0], self.x[1]])

    def slam_callback(self, msg):
        slam_x = msg.pose.pose.position.x
        slam_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        slam_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        z = np.array([slam_x, slam_y, slam_theta])

        innovation = z - self.H @ self.x
        innovation[2] = atan2(np.sin(innovation[2]), np.cos(innovation[2]))

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.x[2] = atan2(np.sin(self.x[2]), np.cos(self.x[2]))
        self.P = (self.I - K @ self.H) @ self.P

        self.get_logger().info(
            f'KF updated | SLAM: ({slam_x:.3f}, {slam_y:.3f}) | '
            f'KF: ({self.x[0]:.3f}, {self.x[1]:.3f}) | '
            f'P trace: {np.trace(self.P):.4f}'
        )

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = float(self.x[0])
        msg.pose.position.y = float(self.x[1])
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(np.sin(self.x[2] / 2))
        msg.pose.orientation.w = float(np.cos(self.x[2] / 2))

        self.pose_pub.publish(msg)

    def save_log(self):
        import csv
        with open('/tmp/kalman_log.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'odom_x', 'odom_y', 'kalman_x', 'kalman_y'])
            writer.writerows(self.log)
        self.get_logger().info('Log saved to /tmp/kalman_log.csv')


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_log()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
