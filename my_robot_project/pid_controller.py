import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt


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


class PIDNavigator(Node):
    def __init__(self):
        super().__init__('pid_navigator')

        self.waypoints = [
            (5.0, 0.0), (5.0, 5.0), (0.0, 5.0),
            (-5.0, 5.0), (-5.0, 0.0), (0.0, 0.0),
        ]
        self.current_idx = 0

        # Angular PID — increased Kd from 0.3 to 0.5 to prevent overshoot
        self.angular_pid = PIDController(
            kp=1.5, ki=0.05, kd=0.5,
            output_min=-1.5, output_max=1.5
        )

        # Linear PID — Kp 0.4→0.6 to maintain progress on rough terrain
        self.linear_pid = PIDController(
            kp=0.6, ki=0.01, kd=0.4,
            output_min=0.0, output_max=0.5
        )

        self.rover_x = None
        self.rover_y = None
        self.rover_theta = None
        self.last_time = None

        self.waypoint_threshold = 0.4
        self.heading_threshold = 0.35

        # --- Rock detection parameters ---
        self.obstacle_distance = 2.5
        self.emergency_stop = 0.4
        self.side_detection = 1.2

        # Avoidance state
        self.latest_scan = None
        self.latest_scan_angles = None
        self.avoiding = False
        self.avoid_direction = 0
        self.avoid_start_time = None

        self.mission_log = []
        self.waypoint_start_time = self.get_clock().now()

        self.pose_sub = self.create_subscription(PoseStamped, '/kalman_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  PID Navigator — Rock-Ready Edition     ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')
        self.get_logger().info(
            f'Angular PID: Kp={self.angular_pid.kp}, Ki={self.angular_pid.ki}, Kd={self.angular_pid.kd}')
        self.get_logger().info(
            f'Linear PID:  Kp={self.linear_pid.kp}, Ki={self.linear_pid.ki}, Kd={self.linear_pid.kd}')
        self.get_logger().info(f'Obstacle detection: {self.obstacle_distance}m, threshold: {self.waypoint_threshold}m')
        self.get_logger().info('Use "ros2 topic echo /scan --once" to check LiDAR returns')

    def pose_callback(self, msg):
        self.rover_x = msg.pose.position.x
        self.rover_y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.rover_theta = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def scan_callback(self, msg):
        self.latest_scan = msg
        if self.latest_scan_angles is None:
            n = len(msg.ranges)
            self.latest_scan_angles = np.array([
                msg.angle_min + i * msg.angle_increment for i in range(n)
            ])

    # ── Sector helpers ──────────────────────────────────────────

    def get_sector_ranges(self, scan, angle_center, half_angle):
        if self.latest_scan_angles is None:
            return []
        angles = self.latest_scan_angles
        diff = np.arctan2(np.sin(angles - angle_center), np.cos(angles - angle_center))
        mask = np.abs(diff) <= half_angle
        ranges = np.array(scan.ranges)
        valid = mask & (ranges > scan.range_min) & (ranges < scan.range_max)
        return list(ranges[valid])

    # ── Rock-aware obstacle avoidance ───────────────────────────

    def get_obstacle_avoidance(self, distance):
        if self.latest_scan is None:
            return 1.0, 0.0

        scan = self.latest_scan

        # Detection zones (three frontal sectors)
        front = self.get_sector_ranges(scan, 0.0, np.radians(25))
        front_min = min(front) if front else float('inf')
        fl = self.get_sector_ranges(scan, np.radians(50), np.radians(20))
        fl_min = min(fl) if fl else float('inf')
        fr = self.get_sector_ranges(scan, np.radians(-50), np.radians(20))
        fr_min = min(fr) if fr else float('inf')

        rock_ahead = front_min < self.obstacle_distance
        rock_fl = fl_min < self.obstacle_distance
        rock_fr = fr_min < self.obstacle_distance
        any_rock = rock_ahead or rock_fl or rock_fr

        now = self.get_clock().now()
        elapsed = (now - self.avoid_start_time).nanoseconds / 1e9 if self.avoid_start_time else 0.0

        # ── No threat: either clear avoidance or coast ──
        if not any_rock:
            if self.avoiding:
                if elapsed > 3.0:
                    self.get_logger().info('✓ Path clear, resume')
                    self.clear_avoidance()
                    return 1.0, 0.0
                return 0.3, self.avoid_direction * 0.4
            return 1.0, 0.0

        # ── Obstacle detected ──────────────────────────
        self.get_logger().info(
            f'LIDAR: front={front_min:.2f} fl={fl_min:.2f} fr={fr_min:.2f}',
            throttle_duration_sec=1.0
        )

        if not self.avoiding:
            self.avoiding = True
            self.avoid_start_time = now

            fl_avg = np.mean(fl) if fl else 0.0
            fr_avg = np.mean(fr) if fr else 0.0

            if fl_avg > fr_avg and fl_min > self.emergency_stop:
                self.avoid_direction = 1
                self.get_logger().info(f'← Avoid LEFT (fl={fl_avg:.1f}m)')
            elif fr_avg > fl_avg and fr_min > self.emergency_stop:
                self.avoid_direction = -1
                self.get_logger().info(f'→ Avoid RIGHT (fr={fr_avg:.1f}m)')
            else:
                self.avoid_direction = 1
                self.get_logger().info(f'← Avoid LEFT (default)')

        # ── Aggressive turn for first 2s ───────────────
        if elapsed < 2.0:
            if front_min < self.emergency_stop:
                return -0.3, self.avoid_direction * 1.0
            return 0.0, self.avoid_direction * 1.0

        # ── Forward while turning ──────────────────────
        if rock_ahead:
            if front_min < self.emergency_stop:
                return -0.3, self.avoid_direction * 1.0
            return 0.2, self.avoid_direction * 0.8

        return 0.3, self.avoid_direction * 0.4

    def clear_avoidance(self):
        self.avoiding = False
        self.avoid_direction = 0
        self.avoid_start_time = None

    # ── Control Loop ────────────────────────────────────────────

    def control_loop(self):
        if self.rover_x is None:
            self.get_logger().info('Waiting for /kalman_pose...', throttle_duration_sec=5.0)
            return

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Mission complete.', throttle_duration_sec=10.0)
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

        if distance <= self.waypoint_threshold:
            elapsed = (now - self.waypoint_start_time).nanoseconds / 1e9
            self.mission_log.append({
                'waypoint': self.current_idx,
                'position': (goal_x, goal_y),
                'time_taken': elapsed,
            })
            self.get_logger().info(
                f'✓ Waypoint {self.current_idx} reached: ({goal_x}, {goal_y}) in {elapsed:.1f}s')
            self.current_idx += 1
            self.waypoint_start_time = now
            self.angular_pid.reset()
            self.linear_pid.reset()
            self.clear_avoidance()
            return

        # ── Obstacle avoidance (takes over during active avoidance) ──
        avoid_linear, avoid_steer = self.get_obstacle_avoidance(distance)

        if self.avoiding:
            cmd = Twist()
            cmd.linear.x = float(np.clip(avoid_linear, -0.5, 0.5))
            cmd.angular.z = float(np.clip(avoid_steer, -1.5, 1.5))
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'⚠ ROCK: lin={cmd.linear.x:.2f} steer={cmd.angular.z:.2f} '
                f'dir={self.avoid_direction} dist={distance:.2f}m',
                throttle_duration_sec=0.5
            )
            return

        # ── Normal PID navigation ────────────────────────────────
        angular_cmd = self.angular_pid.compute(heading_error, dt)

        if abs(heading_error) > self.heading_threshold:
            linear_cmd = 0.0
            self.linear_pid.reset()
        else:
            linear_cmd = self.linear_pid.compute(distance, dt)

        if avoid_linear < 1.0 or avoid_steer != 0.0:
            linear_cmd *= avoid_linear
            angular_cmd += avoid_steer

        cmd = Twist()
        cmd.linear.x = float(np.clip(linear_cmd, 0.0, 0.5))
        cmd.angular.z = float(np.clip(angular_cmd, -1.5, 1.5))
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'WP {self.current_idx} | dist={distance:.2f}m | '
            f'err={heading_error:.3f}rad | lin={cmd.linear.x:.3f} ang={cmd.angular.z:.3f}',
            throttle_duration_sec=2.0
        )

    def stop_rover(self):
        self.cmd_pub.publish(Twist())

    def save_mission_log(self):
        import csv
        try:
            with open('/tmp/pid_mission_log.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['waypoint', 'position', 'time_taken'])
                writer.writeheader()
                writer.writerows(self.mission_log)
            self.get_logger().info('Mission log saved to /tmp/pid_mission_log.csv')
        except Exception as e:
            self.get_logger().error(f'Failed to save log: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PIDNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_mission_log()
    finally:
        node.stop_rover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
