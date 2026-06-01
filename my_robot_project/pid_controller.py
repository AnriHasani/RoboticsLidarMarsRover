import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt


class PIDController:
    """
    A reusable PID controller class.

    We make this a separate class so we can create
    multiple instances — one for angular control, one for linear control.
    Each instance maintains its own error history independently.

    Parameters:
        kp: Proportional gain — how strongly to react to current error
        ki: Integral gain — how strongly to react to accumulated error
        kd: Derivative gain — how strongly to react to error rate of change
        output_min: minimum allowed output (prevents unsafe commands)
        output_max: maximum allowed output (prevents unsafe commands)
    """

    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        # Integral accumulator — sum of all past errors × dt
        # This grows when error persists and shrinks when error is corrected
        self.integral = 0.0

        # Last error — needed to compute the derivative term
        self.last_error = 0.0

        # Flag to handle the first call — no derivative on first step
        # because we have no "previous error" to compare against
        self.first_call = True

    def compute(self, error, dt):
        """
        Compute PID output given current error and time step.

        Parameters:
            error: current error (desired - actual)
            dt:    time since last call in seconds

        Returns:
            output: the control command, clamped to [output_min, output_max]
        """

        if dt <= 0:
            return 0.0

        # P term
        # Immediate reaction to current error.
        # Large error -> large P term -> strong correction.
        p_term = self.kp * error

        # I term
        # Accumulate error over time.
        # If error is consistently +0.1 rad for 5 seconds with dt=0.1:
        self.integral += error * dt

        # Anti-windup: clamp the integral to prevent it from growing
        # unboundedly when the rover is blocked or stuck.
        # Without this, integral could accumulate to huge values during
        # a pause, then cause a violent lurch when motion resumes.
        # We clamp it to a fraction of the output range.
        integral_max = (self.output_max - self.output_min) / 2.0
        self.integral = np.clip(self.integral, -integral_max, integral_max)

        i_term = self.ki * self.integral

        # D term
        # Rate of change of error.
        # If error went from 0.5 to 0.1 in 0.1 seconds:
        # This prevents the rover from overshooting the target heading.
        if self.first_call:
            # No previous error — derivative is undefined on first call
            d_term = 0.0
            self.first_call = False
        else:
            derivative = (error - self.last_error) / dt
            d_term = self.kd * derivative

        self.last_error = error

        # Combined output
        output = p_term + i_term + d_term

        # Clamp to safe operating range
        output = np.clip(output, self.output_min, self.output_max)

        return output

    def reset(self):
        """
        Reset the PID state.
        Called when switching to a new waypoint — the accumulated integral
        from the previous waypoint is irrelevant to the new one.
        Without this, the rover would start each new waypoint with
        leftover integral from the last one, causing erratic behaviour.
        """
        self.integral = 0.0
        self.last_error = 0.0
        self.first_call = True


class PIDNavigator(Node):
    """
    Waypoint navigator with full PID control.

    Replaces the simple proportional controller in waypoint_navigator.py
    with proper PID for both angular and linear velocity control.

    Subscribes to:
        /kalman_pose   — filtered rover position from Kalman filter node

    Publishes to:
        /rover/cmd_vel — velocity commands to the rover's diff drive

    Architecture:
        Angular PID: heading_error → angular.z (turning rate)
        Linear PID:  distance_error → linear.x (forward speed)

    The two PIDs work together:
        - When heading error is large: angular PID turns fast, linear PID
          slows forward speed (no point driving forward while facing wrong way)
        - When aligned: angular PID settles, linear PID drives forward
        - When close to waypoint: linear PID decelerates smoothly
    """

    def __init__(self):
        super().__init__('pid_navigator')

        # Waypoints
        # Same waypoints as your partner's navigator — matching the
        # yellow markers placed in the Gazebo Mars world
        self.waypoints = [
            (5.0,  0.0),
            (5.0,  5.0),
            (0.0,  5.0),
            (-5.0, 5.0),
            (-5.0, 0.0),
            (0.0,  0.0),   # return to start
        ]
        self.current_idx = 0

        # Angular PID
        # Controls turning rate to correct heading error.
        #
        # Kp=1.5: fairly aggressive turning — rover reacts quickly to heading error
        # Ki=0.05: small integral — corrects slow persistent drift
        # Kd=0.3: moderate damping — prevents oscillation around target heading
        #
        # output_max=1.5 rad/s: maximum turning speed (safe for differential drive)
        # Tune these if the rover oscillates (reduce Kp or increase Kd)
        # or if it's too slow to turn (increase Kp)
        self.angular_pid = PIDController(
            kp=1.5, ki=0.05, kd=0.3,
            output_min=-1.5, output_max=1.5
        )

        # Linear PID
        # Controls forward speed based on distance to waypoint.
        #
        # Kp=0.4: gentle acceleration — rover speeds up proportionally to distance
        # Ki=0.01: very small integral — distance rarely has persistent bias
        # Kd=0.2: damping to prevent overshooting the waypoint
        #
        # output_max=0.5 m/s: maximum forward speed
        # We cap this low because Mars terrain is rough — safety first
        self.linear_pid = PIDController(
            kp=0.4, ki=0.01, kd=0.2,
            output_min=0.0, output_max=0.5
        )

        # Rover state
        self.rover_x = None
        self.rover_y = None
        self.rover_theta = None

        # Timing
        self.last_time = None

        # Waypoint thresholds
        # How close the rover needs to be to consider a waypoint "reached"
        self.waypoint_threshold = 0.3   # metres

        # How aligned the rover must be before it starts moving forward
        # If heading error > this, rover turns in place first
        self.heading_threshold = 0.3    # radians

        # Obstacle avoidance parameters
        self.obstacle_distance = 2.0       # metres — start avoiding at this range
        self.emergency_stop = 0.5          # metres — stop immediately
        self.latest_scan = None
        self.avoid_start_time = None       # when continuous avoidance started
        self.avoid_direction = 0           # current avoidance steer sign
        self.last_avoid_distance = None    # distance at start of avoidance

        # Mission logging
        # Track data for your report — distance travelled, time per waypoint
        self.mission_log = []
        self.waypoint_start_time = self.get_clock().now()

        # ROS connections
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/kalman_pose',
            self.pose_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/rover/cmd_vel',
            10
        )

        # Control loop at 10Hz — matches Kalman filter publish rate
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('PID Navigator started')
        self.get_logger().info(
            f'Kp_angular={self.angular_pid.kp}, '
            f'Ki_angular={self.angular_pid.ki}, '
            f'Kd_angular={self.angular_pid.kd}'
        )
        self.get_logger().info(
            f'Kp_linear={self.linear_pid.kp}, '
            f'Ki_linear={self.linear_pid.ki}, '
            f'Kd_linear={self.linear_pid.kd}'
        )

    def pose_callback(self, msg):
        """Update rover pose from Kalman filter output."""
        self.rover_x = msg.pose.position.x
        self.rover_y = msg.pose.position.y

        # Extract yaw from quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.rover_theta = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def scan_callback(self, msg):
        """Store latest laser scan for obstacle avoidance."""
        self.latest_scan = msg

    def get_obstacle_avoidance(self, distance):
        """Check LiDAR scan and return (linear_factor, steer_correction).

        Args:
            distance: current distance to waypoint (for stuck detection)

        Returns:
            linear_factor: negative = reverse, 0.0 = stop, positive = forward
            steer_correction: positive = turn left, negative = turn right (rad/s)
        """
        if self.latest_scan is None:
            self.avoid_start_time = None
            self.avoid_direction = 0
            return 1.0, 0.0

        scan = self.latest_scan
        n = len(scan.ranges)
        mid = n // 2

        # Front sector: ±30° (centered at 0° = forward)
        front_start = int(mid - n * 30 / 360)
        front_end = int(mid + n * 30 / 360)
        front_ranges = [r for r in scan.ranges[front_start:front_end]
                        if scan.range_min < r < scan.range_max]
        front_min = min(front_ranges) if front_ranges else float('inf')

        # Left sector: 30° to 90°
        left_start = int(mid + n * 30 / 360)
        left_end = int(mid + n * 90 / 360)
        left_ranges = [r for r in scan.ranges[left_start:left_end]
                       if scan.range_min < r < scan.range_max]
        left_min = min(left_ranges) if left_ranges else float('inf')

        # Right sector: -90° to -30°
        right_start = int(mid - n * 90 / 360)
        right_end = int(mid - n * 30 / 360)
        right_ranges = [r for r in scan.ranges[right_start:right_end]
                        if scan.range_min < r < scan.range_max]
        right_min = min(right_ranges) if right_ranges else float('inf')

        # Side detection radius (closer than front — side obstacles are less critical)
        side_distance = 1.0

        detecting = (front_min < self.obstacle_distance or
                     left_min < side_distance or
                     right_min < side_distance)

        if not detecting:
            self.avoid_start_time = None
            self.avoid_direction = 0
            return 1.0, 0.0

        # Track continuous avoidance duration
        now = self.get_clock().now()
        if self.avoid_start_time is None:
            self.avoid_start_time = now
            self.avoid_direction = 0
            self.last_avoid_distance = distance
        avoid_elapsed = (now - self.avoid_start_time).nanoseconds / 1e9

        # If no progress for 5+ seconds, reverse hard
        no_progress = (distance >= self.last_avoid_distance - 0.3 and
                       avoid_elapsed > 5.0)

        # After 3s of same-direction avoidance, switch direction
        if avoid_elapsed > 3.0 and avoid_elapsed < 5.0 and not no_progress:
            # Try the less-obstructed side
            if left_min > right_min:
                self.avoid_direction = 1  # go left
            else:
                self.avoid_direction = -1  # go right

        linear_factor = 1.0
        steer = 0.0

        if no_progress:
            linear_factor = -0.5
            steer = 0.8 if self.avoid_direction >= 0 else -0.8
            return linear_factor, steer

        # Obstacle very close ahead — reverse and turn away
        if front_min < self.emergency_stop:
            linear_factor = -0.3
            if self.avoid_direction != 0:
                steer = self.avoid_direction * 0.8
            elif left_min > right_min:
                steer = 0.8
            else:
                steer = -0.8
            return linear_factor, steer

        # Obstacle in front — slow down and steer away
        if front_min < self.obstacle_distance:
            linear_factor = max(0.0, (front_min - self.emergency_stop)
                                / (self.obstacle_distance - self.emergency_stop))
            if self.avoid_direction != 0:
                steer = self.avoid_direction * 0.5
            elif left_min > right_min:
                steer = 0.5
            else:
                steer = -0.5

        # Obstacle only on one side — slight steer away
        if left_min < side_distance and right_min >= side_distance:
            steer = -0.3
        elif right_min < side_distance and left_min >= side_distance:
            steer = 0.3

        return linear_factor, steer

    def control_loop(self):
        """
        Main PID control loop — runs at 10Hz.

        Each iteration:
        1. Compute errors (distance and heading to current waypoint)
        2. Feed errors into PID controllers
        3. Publish velocity command
        4. Check if waypoint is reached
        """

        # Wait until we have a pose from the Kalman filter
        if self.rover_x is None:
            self.get_logger().info(
                'Waiting for /kalman_pose...',
                throttle_duration_sec=5.0
            )
            return

        # All waypoints done — mission complete
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info(
                'All waypoints reached! Mission complete.',
                throttle_duration_sec=10.0
            )
            self.stop_rover()
            return

        # Compute dt
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            return

        # Current waypoint
        goal_x, goal_y = self.waypoints[self.current_idx]

        # Distance error
        # Euclidean distance from rover to waypoint
        dx = goal_x - self.rover_x
        dy = goal_y - self.rover_y
        distance = sqrt(dx * dx + dy * dy)

        # Heading error
        # Angle from rover's current heading to the direction of the waypoint.
        # atan2 gives the absolute angle to the waypoint in the global frame.
        # Subtracting rover_theta gives the error relative to rover's frame.
        # atan2(sin, cos) normalizes the result to [-π, π] — critical to avoid
        # discontinuities at the ±π boundary (e.g. 179° and -179° are only 2° apart)
        bearing = atan2(dy, dx)
        heading_error = atan2(
            np.sin(bearing - self.rover_theta),
            np.cos(bearing - self.rover_theta)
        )

        # Waypoint reached check
        if distance < self.waypoint_threshold:
            elapsed = (now - self.waypoint_start_time).nanoseconds / 1e9
            self.mission_log.append({
                'waypoint': self.current_idx,
                'position': (goal_x, goal_y),
                'time_taken': elapsed,
            })
            self.get_logger().info(
                f'✓ Waypoint {self.current_idx} reached: '
                f'({goal_x}, {goal_y}) in {elapsed:.1f}s'
            )
            self.current_idx += 1
            self.waypoint_start_time = now

            # Reset both PIDs when switching waypoints
            # The integral accumulated for the old waypoint is irrelevant now
            self.angular_pid.reset()
            self.linear_pid.reset()
            return

        # Angular PID
        # Feed heading error into angular PID → get turning rate command
        angular_cmd = self.angular_pid.compute(heading_error, dt)

        # Linear PID
        # Feed distance into linear PID → get forward speed command
        # But: if heading error is too large, don't drive forward at all.
        # The rover should turn to face the waypoint before moving toward it.
        # This prevents the rover from driving in a wide arc instead of
        # turning in place and then going straight.
        if abs(heading_error) > self.heading_threshold:
            # Too misaligned — turn in place, don't move forward
            linear_cmd = 0.0
            self.linear_pid.reset()   # don't accumulate integral while stopped
        else:
            # Aligned — drive forward, PID controls speed based on distance
            linear_cmd = self.linear_pid.compute(distance, dt)

        # Obstacle avoidance from LiDAR
        avoid_linear, avoid_steer = self.get_obstacle_avoidance(distance)
        if avoid_linear < 1.0 or avoid_steer != 0.0:
            linear_cmd *= avoid_linear
            angular_cmd += avoid_steer
            self.get_logger().info(
                f'⚠ Avoiding obstacle: lin_factor={avoid_linear:.2f} '
                f'steer={avoid_steer:.2f}',
                throttle_duration_sec=1.0
            )

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

        # Log every 2 seconds to avoid flooding the terminal
        self.get_logger().info(
            f'WP {self.current_idx} | '
            f'dist={distance:.2f}m | '
            f'heading_err={heading_error:.3f}rad | '
            f'lin={linear_cmd:.3f} ang={angular_cmd:.3f}',
            throttle_duration_sec=2.0
        )

    def stop_rover(self):
        """Send zero velocity — stops the rover immediately."""
        self.cmd_pub.publish(Twist())

    def save_mission_log(self):
        """Save mission timing data to CSV for report analysis."""
        import csv
        try:
            with open('/tmp/pid_mission_log.csv', 'w', newline='') as f:
                writer = csv.DictWriter(
                    f, fieldnames=['waypoint', 'position', 'time_taken']
                )
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