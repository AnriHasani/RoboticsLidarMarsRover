import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt, sin, cos, pi

class IntegratedRoverNavigator(Node):
    def __init__(self):
        super().__init__('integrated_navigator')

        # 1. Mission Configuration
        self.waypoints = [
            (5.0, 0.0), (5.0, 5.0), (0.0, 5.0),
            (-5.0, 5.0), (-5.0, 0.0), (0.0, 0.0)
        ]
        self.current_wp_idx = 0
        self.target_wp = self.waypoints[0]

        # 2. Control Parameters
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.8
        self.goal_tolerance = 0.6
        self.repulsive_gain = 2.5 # Stronger safety buffer
        self.obstacle_thresh = 3.0

        # 3. State Variables
        self.pose = None # [x, y, theta]
        self.slam_pose = None
        self.latest_scan = None
        self.last_slam_time = None
        self.state = "NAVIGATING" # NAVIGATING, RECOVERY, COMPLETED
        
        # Stuck Detection
        self.last_slam_pos = None
        self.stuck_counter = 0
        self.recovery_start = None

        # 4. ROS Infrastructure
        self.pose_sub = self.create_subscription(PoseStamped, '/kalman_pose', self.pose_callback, 10)
        self.slam_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.slam_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  PIVOT-FIRST Rover Navigator Active      ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

    def slam_callback(self, msg):
        self.slam_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.last_slam_time = self.get_clock().now()

    def pose_callback(self, msg):
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        theta = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        self.pose = [msg.pose.position.x, msg.pose.position.y, theta]

    def scan_callback(self, msg):
        self.latest_scan = msg

    def control_loop(self):
        if self.pose is None: return
        now = self.get_clock().now()

        # --- A. SLAM SAFETY CHECK ---
        if self.last_slam_time is None or (now - self.last_slam_time).nanoseconds / 1e9 > 15.0:
            self.get_logger().error('🚨 SLAM DEAD! Emergency Stop.', throttle_duration_sec=10.0)
            self.stop_rover()
            return

        if self.state == "COMPLETED":
            self.stop_rover()
            return

        # --- B. STUCK DETECTION (SLAM-BASED) ---
        if self.state == "NAVIGATING":
            if self.last_slam_pos:
                dist = sqrt((self.slam_pose[0]-self.last_slam_pos[0])**2 + (self.slam_pose[1]-self.last_slam_pos[1])**2)
                if dist < 0.002: self.stuck_counter += 1
                else: self.stuck_counter = 0
                
                if self.stuck_counter > 50: # 5 seconds of wheel spinning, no SLAM movement
                    self.get_logger().warn('🚨 STUCK! Triggering Recovery...')
                    self.state = "RECOVERY"
                    self.recovery_start = now
            self.last_slam_pos = self.slam_pose[:]

        # --- C. WAYPOINT VERIFICATION (STABILITY CHECK) ---
        dist_to_wp = sqrt((self.target_wp[0] - self.pose[0])**2 + (self.target_wp[1] - self.pose[1])**2)
        if dist_to_wp < self.goal_tolerance:
            # We don't advance until SLAM confirms we are at the coordinate
            slam_dist = sqrt((self.target_wp[0] - self.slam_pose[0])**2 + (self.target_wp[1] - self.slam_pose[1])**2)
            if slam_dist < 0.8:
                self.get_logger().info(f'✅ Waypoint {self.current_wp_idx} VERIFIED BY SLAM!')
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints): self.state = "COMPLETED"
                else: self.target_wp = self.waypoints[self.current_wp_idx]
                return

        # --- D. BEHAVIOR LOGIC ---
        cmd = Twist()
        if self.state == "RECOVERY":
            elapsed = (now - self.recovery_start).nanoseconds / 1e9
            if elapsed < 3.0: cmd.linear.x = -0.3 # Back out
            elif elapsed < 7.0: cmd.angular.z = 1.0 # Pivot hard
            else:
                self.state = "NAVIGATING"
                self.stuck_counter = 0
        
        elif self.state == "NAVIGATING":
            # 1. Calculate Target Direction
            dx, dy = self.target_wp[0] - self.pose[0], self.target_wp[1] - self.pose[1]
            goal_heading = atan2(dy, dx)
            
            # 2. Reactive Obstacle Offset (APF)
            rep_vec = np.array([0.0, 0.0])
            if self.latest_scan:
                scan = self.latest_scan
                angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
                for i, r in enumerate(scan.ranges):
                    if scan.range_min < r < self.obstacle_thresh:
                        weight = self.repulsive_gain * (1.0/r - 1.0/self.obstacle_thresh)**2
                        global_obs_angle = angles[i] + self.pose[2]
                        rep_vec[0] -= cos(global_obs_angle) * weight
                        rep_vec[1] -= sin(global_obs_angle) * weight
            
            # Combine Goal + Obstacle
            final_heading = atan2(sin(goal_heading) + rep_vec[1], cos(goal_heading) + rep_vec[0])
            
            # 3. Heading Error
            err = final_heading - self.pose[2]
            while err > pi: err -= 2*pi
            while err < -pi: err += 2*pi

            # --- PIVOT-FIRST LOGIC ---
            # If the error is more than 30 degrees, stop moving forward and just spin!
            if abs(err) > 0.5:
                cmd.linear.x = 0.0
                cmd.angular.z = float(np.clip(1.5 * err, -self.max_angular_vel, self.max_angular_vel))
            else:
                cmd.linear.x = self.max_linear_vel
                cmd.angular.z = float(np.clip(2.0 * err, -0.4, 0.4)) # Gentler steering when driving

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'WP {self.current_wp_idx} | dist={dist_to_wp:.1f}m | err={err:.2f} | SLAM=OK', throttle_duration_sec=2.0)

    def stop_rover(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedRoverNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_rover()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
