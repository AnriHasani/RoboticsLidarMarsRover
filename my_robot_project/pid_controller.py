import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt, pi

class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.integral, self.last_error = 0.0, 0.0
        self.first_call = True

    def compute(self, error, dt):
        if dt <= 0: return 0.0
        p = self.kp * error
        self.integral = np.clip(self.integral + error * dt, -0.2, 0.2)
        i = self.ki * self.integral
        d = 0.0 if self.first_call else self.kd * (error - self.last_error) / dt
        self.first_call, self.last_error = False, error
        return np.clip(p + i + d, self.output_min, self.output_max)

    def reset(self):
        self.integral, self.last_error, self.first_call = 0.0, 0.0, True

class PIDNavigator(Node):
    def __init__(self):
        super().__init__('pid_navigator')

        self.waypoints = [(10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (-10.0, 10.0), (-10.0, 0.0), (0.0, 0.0)]
        self.current_idx = 0

        # High damping (Kd) to stop overshooting
        self.angular_pid = PIDController(kp=1.2, ki=0.02, kd=0.8, output_min=-1.0, output_max=1.0)
        self.linear_pid = PIDController(kp=0.8, ki=0.02, kd=0.5, output_min=0.0, output_max=0.5)

        self.rover_x, self.rover_y, self.rover_theta = None, None, None
        self.last_time = None
        self.waypoint_threshold = 0.8
        
        self.latest_scan = None
        self.last_best_rel_angle = 0.0

        self.create_subscription(PoseStamped, '/kalman_pose', self.pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('PID Navigator: Damped Arrival Mode Active.')

    def pose_callback(self, msg):
        self.rover_x, self.rover_y = msg.pose.position.x, msg.pose.position.y
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        self.rover_theta = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

    def scan_callback(self, msg):
        self.latest_scan = msg

    def find_best_gap_angle(self, target_bearing):
        if self.latest_scan is None: return target_bearing
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
        
        clear_threshold = 1.8
        is_clear = ranges > clear_threshold
        rel_target = atan2(np.sin(target_bearing - self.rover_theta), np.cos(target_bearing - self.rover_theta))
        
        front_indices = np.where((angles > -pi/2) & (angles < pi/2))[0]
        gaps = []
        current_gap = []
        for idx in front_indices:
            if is_clear[idx]: current_gap.append(angles[idx])
            else:
                if len(current_gap) > 8: gaps.append(current_gap)
                current_gap = []
        if current_gap: gaps.append(current_gap)
        
        if not gaps: return rel_target
        
        best_angle, min_error = rel_target, float('inf')
        for gap in gaps:
            center = np.mean(gap)
            error = abs(center - rel_target) + abs(center - self.last_best_rel_angle) * 0.4
            if error < min_error: min_error = error; best_angle = center
        
        self.last_best_rel_angle = best_angle
        return self.rover_theta + best_angle

    def control_loop(self):
        if self.rover_x is None: return

        if self.current_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now()
        if self.last_time is None: self.last_time = now; return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        goal_x, goal_y = self.waypoints[self.current_idx]
        dx, dy = goal_x - self.rover_x, goal_y - self.rover_y
        dist = sqrt(dx*dx + dy*dy)

        if dist <= self.waypoint_threshold:
            self.get_logger().info(f'WP {self.current_idx} Cleared')
            self.current_idx += 1
            return

        target_bearing = atan2(dy, dx)
        planned_heading = self.find_best_gap_angle(target_bearing)
        err = atan2(np.sin(planned_heading - self.rover_theta), np.cos(planned_heading - self.rover_theta))

        cmd = Twist()
        # Smooth Steering
        cmd.angular.z = float(self.angular_pid.compute(err, dt))
        
        # --- ARRIVAL DAMPING ---
        # Slow down as we approach the goal to prevent overshooting
        max_v = 0.5
        if dist < 2.5: max_v = 0.25 # Pre-arrival speed
        if dist < 1.2: max_v = 0.15 # Final creep
            
        if abs(err) < 0.6:
            cmd.linear.x = float(np.clip(self.linear_pid.compute(dist, dt), 0.0, max_v))
        
        self.cmd_pub.publish(cmd)
        if self.get_clock().now().nanoseconds % 50 == 0:
            self.get_logger().info(f'WP {self.current_idx} | dist: {dist:.1f}m | speed: {cmd.linear.x:.2f}')

def main():
    rclpy.init(); rclpy.spin(PIDNavigator()); rclpy.shutdown()

if __name__ == '__main__': main()
