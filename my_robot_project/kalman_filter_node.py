import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from math import atan2

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        #state vector
        #initial position: rover starts at the origin , heading=0 (facing east)
        self.x=np.array([0.0, 0.0, 0.0])

        #covariance matrix P
        #initial uncertainty
        self.P=np.eye(3)*1.0

        #process noise Q
        #how much we distrust the odometry model
        #0.01 m² for x/y means ~10cm std dev per step —> realistic for Mars
        # 0.001 rad² for theta means ~1.8° std dev per step
        # increasing these if the filter is too slow to respond to SLAM corrections
        self.Q=np.diag([0.01,0.01,0.001])

        #measurement noise R
        #how much we distrust the SLAM pose measurements
        #increasing R if SLAM jumps around ,or decreasing if SLAM is
        # very accurate and we want the filter to track it closely
        self.R=np.diag([0.05, 0.05, 0.01])

        #measurement matrix H
        #maps state [x , y , theta] to the measurements [x , y , theta]
        #since SLAM measures the exact state variables -> H=identity
        self.H=np.eye(3)

        #timestamp tracking for dt calculation
        #dt=time between odometry messages  , used in motion model
        #converting velocity->position change
        self.last_time=None

        #data logging for report plots
        #storing raw odometry and kalman estimation over time
        self.log=[]
        self.start_time=self.get_clock().now()

        #Subscribers

        #Odometry-> arrives frequently (-50HZ from diff drive plugin)
        #this is what drives the PREDICT
        self.odom_sub=self.create_subscription(
            '/model/my_robot/odometry',
            self.odom_callback,
            10
        )

        #SLAM pose-> arrives when slam_toolbox has a new estimate (~1-5Hz)
        #this drives the UPDATE step
        self.slam_sub=self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_pose',
            self.slam_callback,
            10
        )

        #Publisher

        #we publish PoseStamped -> position + orientation + timestamp
        #the waypoint navigator will subscribe to this
        self.pose_pub=self.create_publisher(
            PoseStamped,
            '/kalman_pose',
            10
        )

        #timer-> log and publish at 10Hz regardless of message rates
        self.create_timer(0.1, self.publish_pose)

        self.get_logger().info('Kalman Filter node started')
        self.get_logger().info(f'Q (process noise):\n{self.Q}')
        self.get_logger().info(f'R (measurement noise):\n{self.R}')

        #Predict step

        def odom_callback(msg):
            """
             Called every time a new odometry message arrives.
        This implements the PREDICT step of the Kalman filter.

        We extract linear and angular velocity from the message,
        compute how much the rover moved in dt seconds,
        and update the state estimate and covariance.

        The motion model for a differential drive robot:
          x_new = x + v * cos(theta) * dt
          y_new = y + v * sin(theta) * dt
          theta_new = theta + omega * dt

            """

            #compute dt
            now = self.get_clock().now()
            if self.last_time is None:
                self.last_time = now
                return  # skip first message
            dt = (now - self.last_time).nanoseconds / 1e9  # convert ns → seconds
            self.last_time = now

            # Skip if dt is too large (node paused or just started)
            # A dt > 1 second means something is wrong — ignore it
            if dt <= 0 or dt > 1.0:
                return

                # ── Extract velocities from odometry message ──────────────────────────
                # twist.twist.linear.x = forward velocity in m/s
                # twist.twist.angular.z = rotation rate in rad/s (positive = left turn)
                v = msg.twist.twist.linear.x  # linear velocity
                omega = msg.twist.twist.angular.z  # angular velocity

                # ── Current state ─────────────────────────────────────────────────────
                x, y, theta = self.x

                # ── Motion model — predict new state ─────────────────────────────────
                # This is the f(x, u) function: given current state and control input,
                # predict next state. For differential drive:
                x_new = x + v * np.cos(theta) * dt
                y_new = y + v * np.sin(theta) * dt
                theta_new = theta + omega * dt

                # Normalise theta to [-π, π] — angles wrap around
                # Without this, theta could grow to 100π after many rotations
                theta_new = atan2(np.sin(theta_new), np.cos(theta_new))

                # ── State transition matrix F ─────────────────────────────────────────
                # F is the Jacobian of the motion model — how each state variable
                # affects the others after one step.
                # For our motion model:
                #   dx/dx=1, dx/dy=0, dx/dtheta = -v*sin(theta)*dt
                #   dy/dx=0, dy/dy=1, dy/dtheta =  v*cos(theta)*dt
                #   dtheta/dx=0, dtheta/dy=0, dtheta/dtheta=1
                F = np.array([
                    [1, 0, -v * np.sin(theta) * dt],
                    [0, 1, v * np.cos(theta) * dt],
                    [0, 0, 1]
                ])

                # ── Update state and covariance ───────────────────────────────────────
                self.x = np.array([x_new, y_new, theta_new])

                # P grows each predict step — we become more uncertain over time
                # because odometry drifts. Q adds the expected process noise.
                self.P = F @ self.P @ F.T + self.Q

                # ── Log for plotting ──────────────────────────────────────────────────
                # Raw odometry position (from the message directly, not our estimate)
                odom_x = msg.pose.pose.position.x
                odom_y = msg.pose.pose.position.y
                t = (now - self.start_time).nanoseconds / 1e9
                self.log.append([t, odom_x, odom_y, self.x[0], self.x[1]])

            # ── UPDATE STEP ──────────────────────────────────────────────────────────

            def slam_callback(self, msg):
                """
                Called every time slam_toolbox publishes a new pose estimate.
                This implements the UPDATE step of the Kalman filter.

                We extract the SLAM position [x, y, theta] as measurement z,
                compute the Kalman Gain K, and correct our prediction.

                The bigger K is, the more we trust this measurement over our prediction.
                K is computed from P (our uncertainty) and R (sensor uncertainty).
                """
                # ── Extract measurement z from SLAM pose ──────────────────────────────
                slam_x = msg.pose.pose.position.x
                slam_y = msg.pose.pose.position.y

                # Convert quaternion → yaw angle
                # ROS uses quaternions for orientation (4 numbers: x, y, z, w)
                # We only care about yaw (rotation around vertical Z axis)
                # This formula extracts yaw from the quaternion:
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                slam_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

                z = np.array([slam_x, slam_y, slam_theta])

                # ── Innovation: difference between measurement and prediction ─────────
                # This is "what did I expect vs what did I actually measure"
                # If innovation is small → prediction was good → small correction
                # If innovation is large → odometry drifted → large correction
                innovation = z - self.H @ self.x

                # Normalise the theta component of innovation — angles wrap around
                innovation[2] = atan2(np.sin(innovation[2]), np.cos(innovation[2]))

                # ── Innovation covariance S ───────────────────────────────────────────
                # Combines our prediction uncertainty (P) with sensor noise (R)
                # Used to compute the Kalman Gain
                S = self.H @ self.P @ self.H.T + self.R

                # ── Kalman Gain K ─────────────────────────────────────────────────────
                # K tells us how much to weight the measurement vs the prediction
                # K = P*H^T * S^-1
                # np.linalg.inv() computes matrix inverse
                K = self.P @ self.H.T @ np.linalg.inv(S)

                # ── Corrected state estimate ──────────────────────────────────────────
                # Move our estimate toward the measurement, weighted by K
                self.x = self.x + K @ innovation

                # Normalise theta again after update
                self.x[2] = atan2(np.sin(self.x[2]), np.cos(self.x[2]))

                # ── Updated covariance ────────────────────────────────────────────────
                # P shrinks after a measurement — we're more certain now
                # (I - K*H) is the shrink factor
                self.P = (self.I - K @ self.H) @ self.P

                self.get_logger().info(
                    f'KF updated | SLAM: ({slam_x:.3f}, {slam_y:.3f}) | '
                    f'KF: ({self.x[0]:.3f}, {self.x[1]:.3f}) | '
                    f'P trace: {np.trace(self.P):.4f}'
                )

            #Publish

            def publish_pose(self):
                """
                Publishes the current Kalman filter estimate as a PoseStamped.
                Called at 10Hz by the timer — independent of message arrival rates.
                This is what the waypoint navigator will subscribe to.
                """
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'  # pose is in the map coordinate frame

                msg.pose.position.x = float(self.x[0])
                msg.pose.position.y = float(self.x[1])
                msg.pose.position.z = 0.0

                # Convert yaw angle back to quaternion for ROS message format
                # For a yaw-only rotation: qx=0, qy=0, qz=sin(theta/2), qw=cos(theta/2)
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = float(np.sin(self.x[2] / 2))
                msg.pose.orientation.w = float(np.cos(self.x[2] / 2))

                self.pose_pub.publish(msg)

            def save_log(self):
                """
                Saves logged data to CSV for report plots.
                Call this before shutting down, or add a ROS service call to trigger it.
                Format: time, odom_x, odom_y, kalman_x, kalman_y
                """
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
                node.save_log()  # save data for plotting on Ctrl+C
            finally:
                node.destroy_node()
                rclpy.shutdown()

        if __name__ == '__main__':
            main()










