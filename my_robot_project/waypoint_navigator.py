import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from math import atan2, sqrt

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Waypoints (x, y) - Hardcoded positions
        self.waypoints = [
            (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (-5.0, 5.0), (-5.0, 0.0), (0.0, 0.0)
        ]
        self.current_waypoint_idx = 0
        
        # Current pose estimate from Kalman Filter
        self.rover_x = None
        self.rover_y = None
        self.rover_theta = None
        
        # Subscription to Kalman pose
        self.pose_sub = self.create_subscription(
            PoseStamped, '/kalman_pose', self.pose_callback, 10
        )
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist, '/rover/cmd_vel', 10
        )
        
        # Timer for control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Waypoint Navigator started. Navigating to {len(self.waypoints)} points.')

    def pose_callback(self, msg):
        """Update the current rover pose from the Kalman Filter output."""
        self.rover_x = msg.pose.position.x
        self.rover_y = msg.pose.position.y
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        self.rover_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    def control_loop(self):
        """Main control loop to drive the rover toward the current waypoint."""
        if self.rover_x is None:
            self.get_logger().info('Waiting for /kalman_pose...', throttle_duration_sec=5.0)
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Stopping.', throttle_duration_sec=10.0)
            self.stop_rover()
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]
        dist = sqrt((goal_x - self.rover_x)**2 + (goal_y - self.rover_y)**2)
        bearing = atan2(goal_y - self.rover_y, goal_x - self.rover_x)
        heading_error = atan2(np.sin(bearing - self.rover_theta), np.cos(bearing - self.rover_theta))
        
        # Reached waypoint logic
        if dist < 0.3:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}: ({goal_x}, {goal_y})')
            self.current_waypoint_idx += 1
            return

        # Simple proportional controller
        msg = Twist()
        msg.angular.z = 2.0 * heading_error
        msg.linear.x = 0.5 if abs(heading_error) < 0.3 else 0.1
            
        self.get_logger().info(
            f'WP {self.current_waypoint_idx} | Dist: {dist:.2f}m | Err: {heading_error:.2f}rad | Yaw: {self.rover_theta:.2f}rad',
            throttle_duration_sec=1.0
        )
        self.cmd_pub.publish(msg)

    def stop_rover(self):
        """Publishes zero velocity to stop the rover."""
        msg = Twist()
        try:
            self.cmd_pub.publish(msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
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
