import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt


class KalmanFilter:
    def __init__(self):
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3) * 1.0
        self.I = np.eye(3)
        self.Q = np.diag([0.01, 0.01, 0.001])
        self.R = np.diag([0.05, 0.05, 0.01])
        self.H = np.eye(3)
        self.last_time = None
        self.initialized = False

    def predict(self, v, omega, dt):
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

    def update(self, z):
        innovation = z - self.H @ self.x
        innovation[2] = atan2(np.sin(innovation[2]), np.cos(innovation[2]))

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.x[2] = atan2(np.sin(self.x[2]), np.cos(self.x[2]))
        self.P = (self.I - K @ self.H) @ self.P

    def init_from_odom(self, msg):
        self.x[0] = msg.pose.pose.position.x
        self.x[1] = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        self.x[2] = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.initialized = True


class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.last_error = 0.0
        self.first_call = True

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        p_term = self.kp * error
        self.integral += error * dt
        integral_max = (self.output_max - self.output_min) / 2.0
        self.integral = np.clip(self.integral, -integral_max, integral_max)
        i_term = self.ki * self.integral
        if self.first_call:
            d_term = 0.0
            self.first_call = False
        else:
            derivative = (error - self.last_error) / dt
            d_term = self.kd * derivative
        self.last_error = error
        output = p_term + i_term + d_term
        return np.clip(output, self.output_min, self.output_max)

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.first_call = True


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # ── Kalman Filter ───────────────────────────────────────
        self.kf = KalmanFilter()

        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odometry', self.odom_callback, 10)
        self.slam_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.slam_callback, 10)

        self.slam_received = False
        self.slam_log_count = 0
        self.create_timer(5.0, self.log_slam_status)

        self.kalman_pose_pub = self.create_publisher(PoseStamped, '/kalman_pose', 10)

        # ── PID Control ─────────────────────────────────────────
        self.waypoints = [
            (5.0, 0.0), (5.0, 5.0), (0.0, 5.0),
            (-5.0, 5.0), (-5.0, 0.0), (0.0, 0.0),
        ]
        self.current_idx = 0
        self.waypoint_threshold = 0.3
        self.heading_threshold = 0.35

        self.angular_pid = PIDController(
            kp=1.5, ki=0.05, kd=0.5,
            output_min=-1.5, output_max=1.5
        )
        self.linear_pid = PIDController(
            kp=0.4, ki=0.01, kd=0.4,
            output_min=0.0, output_max=0.5
        )

        # ── Obstacle Avoidance ───────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.latest_scan_angles = None

        # Detection thresholds
        self.obstacle_distance = 2.5
        self.emergency_stop = 0.4
        self.side_detection = 1.2

        # Persistent avoidance state
        self.avoiding = False
        self.avoid_direction = 0
        self.avoid_start_time = None
        self.last_waypoint_dist = None
        self.stuck_start_time = None
        self.stuck_reversing = False
        self.reverse_start_time = None

        # ── Rover State ─────────────────────────────────────────
        self.rover_x = None
        self.rover_y = None
        self.rover_theta = None
        self.last_time = None

        # ── Publishers ──────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # ── Control Loop ────────────────────────────────────────
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║     MyNode started — Rock-ready         ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')
        self.get_logger().info(
            f'Angular PID: Kp={self.angular_pid.kp}, Ki={self.angular_pid.ki}, Kd={self.angular_pid.kd}')
        self.get_logger().info(
            f'Linear PID:  Kp={self.linear_pid.kp}, Ki={self.linear_pid.ki}, Kd={self.linear_pid.kd}')
        self.get_logger().info(f'Waypoint threshold: {self.waypoint_threshold}m')
        self.get_logger().info(f'Obstacle detection range: {self.obstacle_distance}m')

    # ── SLAM Fusion ─────────────────────────────────────────────

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if not self.kf.initialized:
            self.kf.init_from_odom(msg)
            self.kf.last_time = now
            self.get_logger().info(f'Kalman initialized at: {self.kf.x}')
            return

        dt = (now - self.kf.last_time).nanoseconds / 1e9
        self.kf.last_time = now
        if dt <= 0 or dt > 1.0:
            return

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.kf.predict(v, omega, dt)

    def slam_callback(self, msg):
        self.slam_received = True
        self.slam_log_count += 1
        slam_x = msg.pose.pose.position.x
        slam_y = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        slam_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        self.get_logger().info(
            f'SLAM /pose #{self.slam_log_count}: x={slam_x:.2f} y={slam_y:.2f} θ={slam_theta:.2f}',
            throttle_duration_sec=2.0
        )

        z = np.array([slam_x, slam_y, slam_theta])
        self.kf.update(z)

        msg_out = PoseStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'map'
        msg_out.pose.position.x = float(self.kf.x[0])
        msg_out.pose.position.y = float(self.kf.x[1])
        msg_out.pose.orientation.z = float(np.sin(self.kf.x[2] / 2))
        msg_out.pose.orientation.w = float(np.cos(self.kf.x[2] / 2))
        self.kalman_pose_pub.publish(msg_out)

        self.rover_x = float(self.kf.x[0])
        self.rover_y = float(self.kf.x[1])
        self.rover_theta = float(self.kf.x[2])

    def log_slam_status(self):
        if not self.slam_received:
            self.get_logger().warn('No /pose data from slam_toolbox yet')
        else:
            self.get_logger().info(f'SLAM fusion: {self.slam_log_count} /pose updates')

    # ── Scan Processing ─────────────────────────────────────────

    def scan_callback(self, msg):
        self.latest_scan = msg
        if self.latest_scan_angles is None:
            n = len(msg.ranges)
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            self.latest_scan_angles = np.array(
                [angle_min + i * angle_increment for i in range(n)]
            )

    def get_sector_ranges(self, scan, angle_center, half_angle):
        """Return valid ranges within a sector centered at angle_center ± half_angle (radians)."""
        if self.latest_scan_angles is None:
            return []
        angles = self.latest_scan_angles
        r_min = scan.range_min
        r_max = scan.range_max

        diff = np.arctan2(np.sin(angles - angle_center), np.cos(angles - angle_center))
        mask = np.abs(diff) <= half_angle
        ranges = np.array(scan.ranges)
        valid = mask & (ranges > r_min) & (ranges < r_max)
        return list(ranges[valid])

    def find_clearest_direction(self, scan):
        """Scan the front 180° in 10° bins and return the angle (relative to forward) 
        with the most open space. Used to pick the best escape route around a rock."""
        if self.latest_scan_angles is None:
            return 0.0

        angles = self.latest_scan_angles
        ranges = np.array(scan.ranges)
        r_min = scan.range_min
        r_max = scan.range_max
        valid_mask = (ranges > r_min) & (ranges < r_max)

        best_angle = 0.0
        best_score = 0.0

        for deg in range(-90, 91, 10):
            center_rad = np.radians(deg)
            diff = np.arctan2(np.sin(angles - center_rad), np.cos(angles - center_rad))
            sector_mask = np.abs(diff) <= np.radians(20)
            active = valid_mask & sector_mask
            if np.any(active):
                mean_range = float(np.mean(ranges[active]))
                min_range = float(np.min(ranges[active]))
                count = int(np.sum(active))
                # Score: farther mean range + farther min range = better
                score = mean_range * 1.0 + min_range * 2.0 + count * 0.01
                if score > best_score:
                    best_score = score
                    best_angle = center_rad

        return best_angle

    def check_clear_ahead(self, scan, dist):
        """Check if the path directly ahead is clear up to `dist` meters."""
        front = self.get_sector_ranges(scan, 0.0, np.radians(15))
        if not front:
            return True
        return min(front) > dist

    def get_obstacle_avoidance(self, distance_to_goal):
        """Rock-aware obstacle avoidance.
        
        Strategy:
        1. Detect obstacles in front (60° FOV) and sides
        2. If blocked, find the clearest path in the front 180°
        3. Commit to that direction until obstacle is cleared
        4. Monitor progress — if stuck for 5s, reverse & try other way
        5. Within 1.0m of goal: only avoid actual obstacles >0.5m, 
           ignore tiny waypoint spheres
        """
        if self.latest_scan is None:
            return 1.0, 0.0

        scan = self.latest_scan

        # Front sector: 60° for rock detection (wider than before)
        front_ranges = self.get_sector_ranges(scan, 0.0, np.radians(30))
        front_min = min(front_ranges) if front_ranges else float('inf')

        # Front-left (30° to 80°)
        fl_ranges = self.get_sector_ranges(scan, np.radians(55), np.radians(25))
        fl_min = min(fl_ranges) if fl_ranges else float('inf')

        # Front-right (-80° to -30°)
        fr_ranges = self.get_sector_ranges(scan, np.radians(-55), np.radians(25))
        fr_min = min(fr_ranges) if fr_ranges else float('inf')

        # Side left (80° to 120°)
        l_ranges = self.get_sector_ranges(scan, np.radians(100), np.radians(20))
        l_min = min(l_ranges) if l_ranges else float('inf')

        # Side right (-120° to -80°)
        r_ranges = self.get_sector_ranges(scan, np.radians(-100), np.radians(20))
        r_min = min(r_ranges) if r_ranges else float('inf')

        # ── Near-goal logic ─────────────────────────────────────
        # Within 1.0m of goal, only real obstacles (>0.5m projection)
        # trigger avoidance. Tiny waypoint spheres (0.2m radius) are
        # ignored since they're at the goal, not blocking the path.
        if distance_to_goal < 1.0:
            # We still need to avoid rocks near the waypoint
            # But tiny objects right at the goal don't matter
            if front_min < self.emergency_stop or fl_min < self.emergency_stop or fr_min < self.emergency_stop:
                front_clear = self.check_clear_ahead(scan, 0.5)
                if front_clear and front_min < self.emergency_stop:
                    # The only thing close is the waypoint sphere — drive through
                    return 0.3, 0.0
                # Real obstacle — back off
                return -0.3, 0.0
            return 1.0, 0.0

        # ── Rock ahead detection ─────────────────────────────────
        rock_ahead = front_min < self.obstacle_distance
        rock_fl = fl_min < self.obstacle_distance
        rock_fr = fr_min < self.obstacle_distance
        rock_l = l_min < self.side_detection
        rock_r = r_min < self.side_detection

        any_rock = rock_ahead or rock_fl or rock_fr or rock_l or rock_r

        if not any_rock:
            self.clear_avoidance_state()
            return 1.0, 0.0

        # ── On first detection, commit to a direction ────────────
        now = self.get_clock().now()
        if not self.avoiding:
            self.avoiding = True
            self.avoid_start_time = now
            self.last_waypoint_dist = distance_to_goal
            self.stuck_start_time = None
            self.stuck_reversing = False

            # Check front-left and front-right clearance
            fl_all = self.get_sector_ranges(scan, np.radians(45), np.radians(45))
            fr_all = self.get_sector_ranges(scan, np.radians(-45), np.radians(45))
            fl_avg = np.mean(fl_all) if fl_all else 0.0
            fr_avg = np.mean(fr_all) if fr_all else 0.0

            # Find the clearest path
            best_angle = self.find_clearest_direction(scan)
            if best_angle > 0.1:
                self.avoid_direction = 1
                self.get_logger().info(f'← Avoiding: steering LEFT (best={best_angle:.2f}rad)')
            elif best_angle < -0.1:
                self.avoid_direction = -1
                self.get_logger().info(f'→ Avoiding: steering RIGHT (best={best_angle:.2f}rad)')
            elif fl_avg > fr_avg:
                self.avoid_direction = 1
                self.get_logger().info('← Avoiding: LEFT side clearer')
            else:
                self.avoid_direction = -1
                self.get_logger().info('→ Avoiding: RIGHT side clearer')

            return self.compute_avoidance_command(
                front_min, fl_min, fr_min, l_min, r_min, distance_to_goal
            )

        # ── Already avoiding — check progress ────────────────────
        avoid_elapsed = (now - self.avoid_start_time).nanoseconds / 1e9
        progress = distance_to_goal < self.last_waypoint_dist - 0.2

        if avoid_elapsed > 5.0 and not progress:
            if self.stuck_start_time is None:
                self.stuck_start_time = now
                self.get_logger().warn('⚠ No progress for 5s — initiating stuck recovery')
            stuck_elapsed = (now - self.stuck_start_time).nanoseconds / 1e9

            if stuck_elapsed > 7.0 and not self.stuck_reversing:
                self.stuck_reversing = True
                self.reverse_start_time = now
                self.avoid_direction *= -1
                self.get_logger().warn(f'↩ Reversing & trying opposite direction (avoid_direction={self.avoid_direction})')
                return -0.5, self.avoid_direction * 0.8

            if self.stuck_reversing:
                reverse_elapsed = (now - self.reverse_start_time).nanoseconds / 1e9
                if reverse_elapsed > 3.0:
                    self.stuck_reversing = False
                    self.stuck_start_time = None
                    self.last_waypoint_dist = distance_to_goal
                    self.get_logger().info('↪ Finished reversing, resuming forward avoidance')
                else:
                    return -0.4, self.avoid_direction * 0.6
        else:
            self.stuck_start_time = None
            self.stuck_reversing = False
            self.last_waypoint_dist = distance_to_goal

        # Obstacle now cleared? Check if the way ahead is open
        if self.avoiding and self.check_clear_ahead(scan, self.obstacle_distance):
            self.get_logger().info('✓ Obstacle cleared — resuming normal navigation')
            self.clear_avoidance_state()
            return 1.0, 0.0

        return self.compute_avoidance_command(
            front_min, fl_min, fr_min, l_min, r_min, distance_to_goal
        )

    def compute_avoidance_command(self, front_min, fl_min, fr_min, l_min, r_min, distance):
        """Compute linear factor and steer correction based on current obstacle distances."""
        steer = self.avoid_direction * 0.6
        linear = 1.0

        # Emergency — obstacle extremely close
        if front_min < self.emergency_stop:
            linear = -0.4
            steer = self.avoid_direction * 0.9
            return linear, steer

        # Obstacle close ahead — slow down and steer hard
        if front_min < (self.obstacle_distance * 0.5):
            linear = 0.2
            steer = self.avoid_direction * 0.8
            return linear, steer

        # Obstacle ahead but not too close — moderate speed, moderate steer
        if front_min < self.obstacle_distance:
            linear = 0.4
            steer = self.avoid_direction * 0.6
            return linear, steer

        # Obstacle only on one flank — slight steer away
        if l_min < self.side_detection and r_min >= self.side_detection:
            return 0.6, -0.3
        if r_min < self.side_detection and l_min >= self.side_detection:
            return 0.6, 0.3

        return linear, steer

    def clear_avoidance_state(self):
        self.avoiding = False
        self.avoid_direction = 0
        self.avoid_start_time = None
        self.stuck_start_time = None
        self.stuck_reversing = False
        self.last_waypoint_dist = None

    # ── Control Loop ────────────────────────────────────────────

    def control_loop(self):
        if self.rover_x is None:
            self.get_logger().info('Waiting for pose estimate...', throttle_duration_sec=5.0)
            return

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!', throttle_duration_sec=10.0)
            self.stop_rover()
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0 or dt > 1.0:
            return

        goal_x, goal_y = self.waypoints[self.current_idx]
        dx = goal_x - self.rover_x
        dy = goal_y - self.rover_y
        distance = sqrt(dx * dx + dy * dy)

        bearing = atan2(dy, dx)
        heading_error = atan2(
            np.sin(bearing - self.rover_theta),
            np.cos(bearing - self.rover_theta)
        )

        # ── Waypoint reached ────────────────────────────────────
        if distance <= self.waypoint_threshold:
            self.get_logger().info(f'✓ Waypoint {self.current_idx} reached at ({goal_x}, {goal_y})')
            self.current_idx += 1
            self.angular_pid.reset()
            self.linear_pid.reset()
            self.clear_avoidance_state()
            return

        # ── Obstacle avoidance (may override PID output) ────────
        avoid_linear, avoid_steer = self.get_obstacle_avoidance(distance)

        if self.avoiding:
            # During active avoidance, use avoidance commands directly
            cmd = Twist()
            cmd.linear.x = max(0.0, min(0.5, avoid_linear))
            cmd.angular.z = np.clip(avoid_steer, -1.5, 1.5)

            # If we're reversing, allow negative linear
            if self.stuck_reversing:
                cmd.linear.x = avoid_linear

            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'⚠ AVOID: lin={cmd.linear.x:.2f} steer={cmd.angular.z:.2f} '
                f'dir={self.avoid_direction} | dist={distance:.2f}m',
                throttle_duration_sec=0.5
            )
            return

        # ── Normal PID navigation ───────────────────────────────
        angular_cmd = self.angular_pid.compute(heading_error, dt)

        if abs(heading_error) > self.heading_threshold:
            linear_cmd = 0.0
            self.linear_pid.reset()
        else:
            linear_cmd = self.linear_pid.compute(distance, dt)

        # Apply avoidance adjustments if any (non-avoiding case)
        if avoid_linear < 1.0 or avoid_steer != 0.0:
            linear_cmd *= avoid_linear
            angular_cmd += avoid_steer

        cmd = Twist()
        cmd.linear.x = np.clip(linear_cmd, 0.0, 0.5)
        cmd.angular.z = np.clip(angular_cmd, -1.5, 1.5)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'WP {self.current_idx} | dist={distance:.2f}m | '
            f'err={heading_error:.3f}rad | lin={cmd.linear.x:.3f} ang={cmd.angular.z:.3f}',
            throttle_duration_sec=2.0
        )

    def stop_rover(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
